"""A minimal in-process differential-drive plant — a drop-in for SerialLink so
the whole supervisor pipeline runs with no hardware.

First-order wheel dynamics + a scripted right-wheel traction loss (1 s out of
every 6 s). During the slip window the encoders still read the commanded wheel
speed, but the IMU integrates the *effective* (reduced) speed — so the
wheel-vs-IMU residual rises and the slip/yaw fuzzy blocks have something real to
react to. NOT physically accurate; a wiring/behaviour test only.
"""
import threading
import time

from probot_pi.bsp import params as P
from probot_pi.services import kinematics as kin


class SimLink:
    def __init__(self, state, hz=P.TELEM_LOOP_HZ, disturb=True):
        self.state = state
        self.dt = 1.0 / hz
        self.disturb = disturb
        self._wl = 0.0
        self._wr = 0.0
        self._cmd = (0.0, 0.0)
        self._yaw = 0.0
        self._t = 0.0
        self._seq = 0
        self._running = False
        self.stats = {"rx_frames": 0, "rx_bad": 0, "tx_frames": 0}

    def start(self):
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def stop(self):
        self._running = False

    def send_cmd(self, wl_ref, wr_ref, mode, seq):
        self.stats["tx_frames"] += 1
        self._cmd = (wl_ref, wr_ref) if mode == P.MODE_RUN else (0.0, 0.0)

    def _loop(self):
        tau = 0.08
        while self._running:
            wl_ref, wr_ref = self._cmd
            a = self.dt / (tau + self.dt)
            self._wl += a * (wl_ref - self._wl)
            self._wr += a * (wr_ref - self._wr)

            slip = 1.0
            if self.disturb and 2.0 <= (self._t % 6.0) < 3.0:
                slip = 0.5                       # right wheel loses ~half its grip

            _, w_body = kin.forward(self._wl, self._wr * slip)
            self._yaw += w_body * self.dt

            self.state.update({
                "omega_meas_l": self._wl, "omega_meas_r": self._wr,
                "yaw": self._yaw, "yaw_rate": w_body,
                "pwm_l": 0.0, "pwm_r": 0.0, "vbat": P.VBAT_NOMINAL,
                "fault_flags": 0, "seq": self._seq & 0xFFFF,
            })
            self.stats["rx_frames"] += 1
            self._seq += 1
            self._t += self.dt
            time.sleep(self.dt)
