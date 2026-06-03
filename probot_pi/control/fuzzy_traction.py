"""Block 2 — traction Mamdani controller (scikit-fuzzy).

Inputs : sigma_err  slip error       [0, 1]      3 MFs (LOW/MED/HIGH)
         abs_r_err  |yaw-rate error| [0, 120]deg/s 3 MFs (SMALL/MED/LARGE)
Output : lam        traction scale   [0.4, 1.0]  3 MFs (LOW/MED/HIGH)
Rules  : full 3x3 grid = 9 rules, min-max inference, centroid defuzz.

λ scales the nominal wheel setpoints (ω_ref' = λ·ωE ∓ Δω_yaw). More slip -> lower
λ -> less torque demanded, letting the wheel regain grip. A large yaw-rate error
together with slip is treated as the worst case (strongest cut).
"""
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

from probot_pi.bsp import params as P


def _build():
    Smax, Amax = P.TRAC_SIGMA_MAX, P.TRAC_ABSR_MAX
    Lmin, Lmax = P.LAMBDA_MIN, P.LAMBDA_MAX
    Lmid = 0.5 * (Lmin + Lmax)

    sigma = ctrl.Antecedent(np.linspace(0, Smax, 101), "sigma_err")
    absr = ctrl.Antecedent(np.linspace(0, Amax, 101), "abs_r_err")
    lam = ctrl.Consequent(np.linspace(Lmin, Lmax, 101), "lam")

    sigma["LOW"] = fuzz.trimf(sigma.universe, [0, 0, Smax * 0.5])
    sigma["MED"] = fuzz.trimf(sigma.universe, [0, Smax * 0.5, Smax])
    sigma["HIGH"] = fuzz.trimf(sigma.universe, [Smax * 0.5, Smax, Smax])

    absr["SMALL"] = fuzz.trimf(absr.universe, [0, 0, Amax * 0.5])
    absr["MED"] = fuzz.trimf(absr.universe, [0, Amax * 0.5, Amax])
    absr["LARGE"] = fuzz.trimf(absr.universe, [Amax * 0.5, Amax, Amax])

    lam["LOW"] = fuzz.trimf(lam.universe, [Lmin, Lmin, Lmid])
    lam["MED"] = fuzz.trimf(lam.universe, [Lmin, Lmid, Lmax])
    lam["HIGH"] = fuzz.trimf(lam.universe, [Lmid, Lmax, Lmax])
    lam.defuzzify_method = "centroid"

    # FAM: rows = sigma_err, cols = abs_r_err.
    table = {
        "LOW":  {"SMALL": "HIGH", "MED": "HIGH", "LARGE": "MED"},
        "MED":  {"SMALL": "HIGH", "MED": "MED",  "LARGE": "LOW"},
        "HIGH": {"SMALL": "MED",  "MED": "LOW",  "LARGE": "LOW"},
    }
    rules = [ctrl.Rule(sigma[s] & absr[a], lam[table[s][a]])
             for s in table for a in table[s]]
    return ctrl.ControlSystemSimulation(ctrl.ControlSystem(rules))


class TractionFuzzy:
    def __init__(self):
        self._sim = _build()

    def compute(self, sigma_err, abs_r_err_dps):
        self._sim.input["sigma_err"] = float(np.clip(sigma_err, 0.0, P.TRAC_SIGMA_MAX))
        self._sim.input["abs_r_err"] = float(np.clip(abs_r_err_dps, 0.0, P.TRAC_ABSR_MAX))
        self._sim.compute()
        return float(self._sim.output["lam"])
