"""Command-derived expectations: reference yaw rate and a slip-error proxy.

The robot has no independent ground-speed sensor, so a classic slip ratio (wheel
speed vs vehicle speed) is unobservable on the Pi. Instead we use a *kinematic
consistency residual*: if a wheel slips, the yaw rate the wheels IMPLY,
R*(wr-wl)/L, diverges from the yaw rate the IMU actually MEASURES. The
normalised magnitude of that gap is a usable slip proxy.

sigma_expected subtracts the part explained by an intentional turn so a clean
commanded curve is not flagged as slip (the "loại false positive khi cua" rule).
This is an initial model — refine SLIP_SCALE_DPS / SLIP_EXPECT_GAIN with real
T1-T4 data in Phase 9/11.
"""
import math

from probot_pi.bsp import params as P
from probot_pi.services import kinematics as kin


def expected_yaw_rate(w_cmd):
    """Reference body yaw rate (rad/s) for the current command."""
    return w_cmd


class SlipEstimator:
    def __init__(self, scale_dps=P.SLIP_SCALE_DPS, expect_gain=P.SLIP_EXPECT_GAIN):
        self.scale_dps = scale_dps
        self.expect_gain = expect_gain

    def sigma_err(self, omega_l, omega_r, r_imu, w_cmd):
        """Slip-error in [0,1] (sigma_measured - sigma_expected, clamped)."""
        r_wheel = kin.wheel_yaw_rate(omega_l, omega_r)          # rad/s
        resid_dps = math.degrees(abs(r_wheel - r_imu))
        sigma_meas = min(1.0, resid_dps / max(1e-6, self.scale_dps))
        sigma_exp = min(1.0, self.expect_gain * abs(math.degrees(w_cmd)))
        return max(0.0, min(1.0, sigma_meas - sigma_exp))
