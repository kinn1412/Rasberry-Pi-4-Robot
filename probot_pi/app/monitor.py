"""Read-only link verification for Phase 7 bring-up — NO motion.

Sends MODE_IDLE heartbeats: this keeps the ESP command watchdog fed (so motors
stay stopped instead of tripping FAULT_CMD_TIMEOUT) AND exercises the Pi->ESP TX
path, while we decode the ESP->Pi telemetry and report link health.

What "good" looks like:
  - rx rate ~= TELEM_LOOP_HZ (50 Hz), age low, link OK
  - bad == 0            (every frame passes CRC -> framing/baud correct)
  - drop == 0           (no telemetry seq gaps -> no lost frames)
  - flt has no CMD_TIMEOUT (the ESP is receiving our heartbeats -> TX works)
  - wL/wR ~ 0, duty ~ 0 (idle: nothing is driving the wheels)

This module imports only bsp.params, so it runs with no numpy/skfuzzy installed.
"""
import math
import time

from probot_pi.bsp import params as P


class MonitorLoop:
    def __init__(self, link, state, hz=20, print_hz=5, heartbeat=True):
        self.link = link
        self.state = state
        self.dt = 1.0 / hz
        self.print_every = max(1, int(round(hz / print_hz)))
        self.heartbeat = heartbeat
        self.seq = 0
        self._running = False
        self._rx_at_last_print = 0
        self._t_last_print = time.monotonic()

    def run(self):
        self._running = True
        mode = "IDLE heartbeats + " if self.heartbeat else "passive "
        print(f"monitor: {mode}telemetry listen (Ctrl-C to stop)\n")
        next_t = time.monotonic()
        tick = 0
        while self._running:
            next_t += self.dt
            if self.heartbeat:
                self.link.send_cmd(0.0, 0.0, P.MODE_IDLE, self.seq)
                self.seq = (self.seq + 1) & 0xFFFF

            tick += 1
            if tick % self.print_every == 0:
                self._print_status()

            slack = next_t - time.monotonic()
            if slack > 0:
                time.sleep(slack)
            else:
                next_t = time.monotonic()

    def _print_status(self):
        telem, ts = self.state.latest()
        now = time.monotonic()

        rx = self.link.stats.get("rx_frames", 0)
        span = now - self._t_last_print
        rate = (rx - self._rx_at_last_print) / span if span > 0 else 0.0
        self._rx_at_last_print = rx
        self._t_last_print = now

        bad = self.link.stats.get("rx_bad", 0)
        tx = self.link.stats.get("tx_frames", 0)

        if telem is None:
            print(f"[no telemetry yet]  tx={tx} rx={rx} bad={bad}")
            return

        drops = self.state.dropped()      # counted per-frame in RobotState.update
        faults = P.fault_names(telem["fault_flags"])
        fault_str = ",".join(faults) if faults else "none"
        age_ms = (now - ts) * 1e3
        link = "OK   " if self.state.link_ok(P.CMD_TIMEOUT_S) else "STALE"

        print(
            f"[{link} {rate:4.0f}Hz age={age_ms:5.1f}ms] "
            f"wL={telem['omega_meas_l']:+6.2f} wR={telem['omega_meas_r']:+6.2f} | "
            f"yaw={math.degrees(telem['yaw']):+7.1f} yr={math.degrees(telem['yaw_rate']):+7.1f}deg/s | "
            f"duty={telem['pwm_l']:+5.2f}/{telem['pwm_r']:+5.2f} vbat={telem['vbat']:4.1f}V | "
            f"tx={tx} rx={rx} bad={bad} drop={drops} flt={fault_str}"
        )

    def stop(self):
        self._running = False
