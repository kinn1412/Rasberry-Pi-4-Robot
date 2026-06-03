"""The fuzzy supervisor: combine the two Mamdani blocks and inject setpoints.

Per cycle:
  1. nominal wheel refs from the command via inverse kinematics  -> ωE_L, ωE_R
  2. heading-hold error e_psi  = wrap(ψ_ref - yaw),  ψ_ref integrates w_cmd
  3. yaw-rate error r_err      = yaw_rate_meas - w_cmd
  4. slip error σ_err          from the wheel-vs-IMU residual
  5. Block 1 (e_psi, r_err)    -> Δω_yaw      [rad/s]
     Block 2 (σ_err, |r_err|)  -> λ           [0.4, 1.0]
  6. inject:  ω_ref_L' = λ·ωE_L − Δω_yaw,  ω_ref_R' = λ·ωE_R + Δω_yaw

With fuzzy disabled the supervisor passes through (λ=1, Δω_yaw=0), i.e. plain
inverse-kinematics setpoints — the PID-only baseline for the demo toggle.
"""
import math

from probot_pi.services import kinematics as kin
from probot_pi.services.expected import SlipEstimator, expected_yaw_rate
from probot_pi.control.fuzzy_yaw import YawFuzzy
from probot_pi.control.fuzzy_traction import TractionFuzzy


def _wrap(angle_rad):
    """Wrap to (-pi, pi]."""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


class Supervisor:
    def __init__(self, dt, fuzzy_enabled=True):
        self.dt = dt
        self.fuzzy_enabled = fuzzy_enabled
        self.yaw = YawFuzzy()
        self.trac = TractionFuzzy()
        self.slip = SlipEstimator()
        self.psi_ref = 0.0      # integrated reference heading (rad)
        self.last = {}

    def reset(self):
        self.psi_ref = 0.0

    def step(self, v_cmd, w_cmd, telem):
        """v_cmd (m/s), w_cmd (rad/s), telem dict -> (wl_ref, wr_ref, debug)."""
        we_l, we_r = kin.inverse(v_cmd, w_cmd)

        # heading hold: ψ_ref tracks the commanded yaw rate, so an intentional
        # turn keeps e_psi ~ 0 and is not "corrected" against.
        self.psi_ref += w_cmd * self.dt
        e_psi = _wrap(self.psi_ref - telem["yaw"])             # rad
        e_psi_deg = math.degrees(e_psi)

        r_ref = expected_yaw_rate(w_cmd)                       # rad/s
        r_err_dps = math.degrees(telem["yaw_rate"] - r_ref)

        sigma_err = self.slip.sigma_err(
            telem["omega_meas_l"], telem["omega_meas_r"], telem["yaw_rate"], w_cmd)

        if self.fuzzy_enabled:
            dw_yaw = self.yaw.compute(e_psi_deg, r_err_dps)        # rad/s
            lam = self.trac.compute(sigma_err, abs(r_err_dps))     # [0.4,1]
        else:
            dw_yaw, lam = 0.0, 1.0

        wl_ref = lam * we_l - dw_yaw
        wr_ref = lam * we_r + dw_yaw

        self.last = {
            "wE_l": we_l, "wE_r": we_r,
            "e_psi_deg": e_psi_deg, "r_err_dps": r_err_dps, "sigma_err": sigma_err,
            "dw_yaw": dw_yaw, "lam": lam,
            "omega_ref_l": wl_ref, "omega_ref_r": wr_ref,
        }
        return wl_ref, wr_ref, self.last
