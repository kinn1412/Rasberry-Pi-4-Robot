"""Shared, thread-safe control state between the dashboard (Flask threads) and the
100 Hz supervisor loop.

Dashboard -> loop : command (v, w), mode, fuzzy on/off, runtime tuning, scenarios.
Loop -> dashboard : the latest telemetry/supervisor snapshot.

A slew-rate limiter on v/w means a slider jump or a START never steps the wheels.
The runtime tuning here is intentionally only the knobs that DON'T need a LUT
rebuild (output gains + slip sensitivity); rule tables / MF shapes are design-time.
"""
import threading

from probot_pi.bsp import params as P

MODE_NAMES = {P.MODE_IDLE: "IDLE", P.MODE_RUN: "RUN", P.MODE_ESTOP: "ESTOP"}


def _slew(cur, tgt, max_step):
    d = tgt - cur
    if d > max_step:
        return cur + max_step
    if d < -max_step:
        return cur - max_step
    return tgt


def default_tuning():
    return {
        "k_yaw": 1.0,          # gain on Δω_yaw output (0..2)
        "k_trac": 1.0,         # traction-cut authority (0..2): λ' = 1 - k_trac*(1-λ)
        "slip_scale_dps": P.SLIP_SCALE_DPS,   # smaller = more slip-sensitive
        "slip_expect_gain": P.SLIP_EXPECT_GAIN,
    }


class ControlState:
    def __init__(self, hz=P.COMM_LOOP_HZ, v_slew=0.5, w_slew=4.0):
        self._lock = threading.Lock()
        self.dt = 1.0 / hz
        self._v_tgt = 0.0
        self._w_tgt = 0.0
        self._v = 0.0
        self._w = 0.0
        self._mode = P.MODE_IDLE          # start safe; dashboard START -> RUN
        self._fuzzy = True
        self._tuning = default_tuning()
        self._scenario = None
        self._loop_snap = {}
        self.v_slew = v_slew
        self.w_slew = w_slew

    # ---- dashboard -> loop (setters) --------------------------------------
    def set_command(self, v=None, w=None):
        with self._lock:
            if v is not None:
                self._v_tgt = float(v)
            if w is not None:
                self._w_tgt = float(w)

    def set_mode(self, mode):
        with self._lock:
            self._mode = int(mode)
            if self._mode != P.MODE_RUN:           # stopping clears any scenario
                self._scenario = None
                self._v_tgt = self._w_tgt = 0.0

    def estop(self):
        with self._lock:
            self._mode = P.MODE_ESTOP
            self._scenario = None
            self._v_tgt = self._w_tgt = 0.0
            self._v = self._w = 0.0

    def set_fuzzy(self, enabled):
        with self._lock:
            self._fuzzy = bool(enabled)

    def set_tuning(self, **kw):
        with self._lock:
            for k, v in kw.items():
                if k in self._tuning and v is not None:
                    self._tuning[k] = float(v)

    def start_scenario(self, runner):
        with self._lock:
            self._scenario = runner
            self._mode = P.MODE_RUN

    def stop(self):
        self.set_mode(P.MODE_IDLE)

    # ---- loop -> dashboard ------------------------------------------------
    def set_loop_snapshot(self, snap):
        with self._lock:
            self._loop_snap = snap

    # ---- loop reads (called at 100 Hz) ------------------------------------
    def get_command(self):
        """Return (v, w, mode) with slew limiting + scenario stepping."""
        with self._lock:
            if self._scenario is not None and self._mode == P.MODE_RUN:
                v, w, done = self._scenario.step(self.dt)
                self._v_tgt, self._w_tgt = v, w
                if done:
                    self._scenario = None
                    self._mode = P.MODE_IDLE
                    self._v_tgt = self._w_tgt = 0.0
            tgt_v = self._v_tgt if self._mode == P.MODE_RUN else 0.0
            tgt_w = self._w_tgt if self._mode == P.MODE_RUN else 0.0
            self._v = _slew(self._v, tgt_v, self.v_slew * self.dt)
            self._w = _slew(self._w, tgt_w, self.w_slew * self.dt)
            return self._v, self._w, self._mode

    def get_tuning(self):
        with self._lock:
            return {"fuzzy_enabled": self._fuzzy, **self._tuning}

    # ---- dashboard SSE payload (loop data + current settings) -------------
    def get_snapshot(self):
        with self._lock:
            s = dict(self._loop_snap)
            s["mode"] = MODE_NAMES.get(self._mode, "?")
            s["fuzzy_enabled"] = self._fuzzy
            s["v_target"] = round(self._v_tgt, 3)
            s["w_target"] = round(self._w_tgt, 3)
            s["tuning"] = dict(self._tuning)
            s["scenario"] = self._scenario.label if self._scenario else None
            s["scenario_progress"] = self._scenario.progress() if self._scenario else None
            return s
