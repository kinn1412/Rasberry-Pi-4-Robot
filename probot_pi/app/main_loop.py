"""The supervisor loop — runs at COMM_LOOP_HZ (100 Hz).

Each tick: read the freshest telemetry, run the fuzzy supervisor, send the
modulated wheel setpoints. If telemetry is stale (link down) it commands IDLE;
the ESP's own command watchdog independently stops the motors after 200 ms, so
this is defence in depth, not the only guard.
"""
import math
import time

from probot_pi.bsp import params as P
from probot_pi.control.supervisor import Supervisor


class MainLoop:
    def __init__(self, link, state, command_source, hz=P.COMM_LOOP_HZ,
                 fuzzy_enabled=True, logger=None, verbose=True, print_hz=2.0,
                 backend="lut", tuning_source=None, snapshot_sink=None):
        self.link = link
        self.state = state
        self.command = command_source        # callable() -> (v_cmd, w_cmd, mode)
        self.tuning_source = tuning_source   # callable() -> tuning dict, or None
        self.snapshot_sink = snapshot_sink   # callable(snap_dict), or None
        self.dt = 1.0 / hz
        self.sup = Supervisor(self.dt, fuzzy_enabled=fuzzy_enabled, backend=backend)
        self.logger = logger
        self.verbose = verbose
        self.seq = 0
        self._running = False
        self._print_every = max(1, int(round(hz / print_hz)))
        self._overruns = 0                   # ticks that blew the period budget
        self._tick = 0
        self._t_last_print = 0.0
        self._tick_at_last_print = 0
        self._dt_ema = self.dt               # smoothed loop period -> live rate
        self._t_prev = None

    def _apply_tuning(self, tun):
        self.sup.fuzzy_enabled = tun["fuzzy_enabled"]
        self.sup.k_yaw = tun["k_yaw"]
        self.sup.k_trac = tun["k_trac"]
        self.sup.slip.scale_dps = tun["slip_scale_dps"]
        self.sup.slip.expect_gain = tun["slip_expect_gain"]

    def run(self):
        self._running = True
        next_t = time.monotonic()
        self._t_last_print = next_t
        while self._running:
            next_t += self.dt
            now = time.monotonic()
            if self._t_prev is not None:
                self._dt_ema = 0.9 * self._dt_ema + 0.1 * (now - self._t_prev)
            self._t_prev = now

            if self.tuning_source is not None:
                self._apply_tuning(self.tuning_source())
            v_cmd, w_cmd, mode = self.command()
            telem, _ = self.state.latest()

            dbg = None
            if telem is None or not self.state.link_ok(P.CMD_TIMEOUT_S):
                self.link.send_cmd(0.0, 0.0, P.MODE_IDLE, self.seq)
            else:
                wl, wr, dbg = self.sup.step(v_cmd, w_cmd, telem)
                self.link.send_cmd(wl, wr, mode, self.seq)
                if self.logger:
                    self.logger.log(self.seq, v_cmd, w_cmd, telem, dbg)
            self.seq = (self.seq + 1) & 0xFFFF

            if self.snapshot_sink is not None:
                self.snapshot_sink(self._snapshot(v_cmd, w_cmd, telem, dbg))

            self._tick += 1
            if self.verbose and self._tick % self._print_every == 0:
                self._print_status(v_cmd, w_cmd, telem, dbg)

            slack = next_t - time.monotonic()
            if slack > 0:
                time.sleep(slack)
            else:
                self._overruns += 1
                next_t = time.monotonic()    # fell behind -> resync, don't spiral

    def _print_status(self, v_cmd, w_cmd, telem, dbg):
        now = time.monotonic()
        span = now - self._t_last_print
        rate = (self._tick - self._tick_at_last_print) / span if span > 0 else 0.0
        self._t_last_print = now
        self._tick_at_last_print = self._tick

        if dbg is None:
            print(f"[{rate:5.0f}Hz over={self._overruns}] link down -> commanding IDLE")
            return
        print(
            f"[{rate:5.0f}Hz over={self._overruns}] "
            f"cmd(v={v_cmd:+.2f} w={w_cmd:+.2f}) | "
            f"sig={dbg['sigma_err']:.2f} epsi={dbg['e_psi_deg']:+5.1f} rerr={dbg['r_err_dps']:+6.1f} | "
            f"lam={dbg['lam']:.2f} dw={dbg['dw_yaw']:+.2f} | "
            f"ref={dbg['omega_ref_l']:+5.2f}/{dbg['omega_ref_r']:+5.2f} "
            f"meas={telem['omega_meas_l']:+5.2f}/{telem['omega_meas_r']:+5.2f}"
        )

    def _snapshot(self, v_cmd, w_cmd, telem, dbg):
        snap = {
            "seq": self.seq,
            "rate": round(1.0 / self._dt_ema, 1) if self._dt_ema > 0 else 0.0,
            "overruns": self._overruns,
            "v_cmd": round(v_cmd, 3), "w_cmd": round(w_cmd, 3),
            "link_ok": self.state.link_ok(P.CMD_TIMEOUT_S),
            "rx_frames": self.link.stats.get("rx_frames", 0),
            "rx_bad": self.link.stats.get("rx_bad", 0),
            "tx_frames": self.link.stats.get("tx_frames", 0),
            "drops": self.state.dropped(),
        }
        if telem is not None:
            snap.update(
                omega_meas_l=round(telem["omega_meas_l"], 3),
                omega_meas_r=round(telem["omega_meas_r"], 3),
                yaw_deg=round(math.degrees(telem["yaw"]), 2),
                yaw_rate_dps=round(math.degrees(telem["yaw_rate"]), 2),
                pwm_l=round(telem["pwm_l"], 3), pwm_r=round(telem["pwm_r"], 3),
                vbat=round(telem["vbat"], 2),
                fault_flags=telem["fault_flags"],
                fault_names=P.fault_names(telem["fault_flags"]),
            )
        if dbg is not None:
            snap.update(
                omega_ref_l=round(dbg["omega_ref_l"], 3),
                omega_ref_r=round(dbg["omega_ref_r"], 3),
                e_psi_deg=round(dbg["e_psi_deg"], 2),
                r_err_dps=round(dbg["r_err_dps"], 2),
                sigma_err=round(dbg["sigma_err"], 3),
                dw_yaw=round(dbg["dw_yaw"], 3),
                lam=round(dbg["lam"], 3),
            )
        return snap

    def stop(self):
        self._running = False
