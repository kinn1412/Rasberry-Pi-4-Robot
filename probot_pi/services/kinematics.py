"""Differential-drive kinematics — a mirror of components/services/src/odometry.c.

Sign convention (matches the firmware + IMU): omega_body > 0 is CCW (turning
left), which is yaw-positive. A left turn means the right wheel runs faster than
the left, so omega_body = R*(wr - wl)/L > 0. Consistent end to end.
"""
from probot_pi.bsp import params as P

R = P.WHEEL_RADIUS_M
L = P.WHEEL_BASE_M


def forward(omega_l, omega_r):
    """Wheel speeds (rad/s) -> (v m/s, omega_body rad/s)."""
    v = R * (omega_r + omega_l) * 0.5
    w = R * (omega_r - omega_l) / L
    return v, w


def inverse(v, omega_body):
    """(v m/s, omega_body rad/s) -> (omega_l_ref, omega_r_ref) rad/s."""
    wl = (v - omega_body * L * 0.5) / R
    wr = (v + omega_body * L * 0.5) / R
    return wl, wr


def wheel_yaw_rate(omega_l, omega_r):
    """Kinematic yaw rate implied by the wheel speeds alone (rad/s)."""
    return R * (omega_r - omega_l) / L
