"""Block 1 — yaw-stability Mamdani controller (scikit-fuzzy).

Inputs : e_psi  heading error          [-30, 30]  deg, 5 MFs
         r_err  yaw-rate error         [-120,120] deg/s, 5 MFs
Output : dw_yaw yaw bias               [-YAW_DW_MAX, +YAW_DW_MAX] rad/s, 5 MFs
Rules  : full 5x5 grid = 25 rules, min-max inference, centroid defuzz.

It is a fuzzy PD on heading: the FAM is out_level = sat(e_level - r_err_level).
Δω_yaw is injected as  ω_ref_R' += Δω_yaw,  ω_ref_L' -= Δω_yaw, so positive
Δω_yaw drives the robot CCW (yaw-increasing). Feeding the yaw-rate *error*
(measured - reference) instead of raw r means an intentional turn does not get
fought — only uncommanded yaw is corrected.
"""
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

from probot_pi.bsp import params as P

_LABELS = ["NB", "NS", "ZE", "PS", "PB"]   # levels -2..+2
_LEVEL = {lab: i - 2 for i, lab in enumerate(_LABELS)}
_INV = {i - 2: lab for i, lab in enumerate(_LABELS)}


def _five_mf(var, span):
    """Five evenly spaced triangular MFs NB..PB over [-span, span]."""
    centers = np.linspace(-span, span, 5)
    half = span / 2.0
    for lab, c in zip(_LABELS, centers):
        var[lab] = fuzz.trimf(var.universe, [c - half, c, c + half])


def _build():
    E, Rr, D = P.YAW_EPSI_RANGE, P.YAW_RERR_RANGE, P.YAW_DW_MAX
    e_psi = ctrl.Antecedent(np.linspace(-E, E, 201), "e_psi")
    r_err = ctrl.Antecedent(np.linspace(-Rr, Rr, 201), "r_err")
    dw = ctrl.Consequent(np.linspace(-D, D, 201), "dw_yaw")

    _five_mf(e_psi, E)
    _five_mf(r_err, Rr)
    _five_mf(dw, D)
    dw.defuzzify_method = "centroid"

    def sat(x):
        return max(-2, min(2, x))

    rules = []
    for el in _LABELS:
        for rl in _LABELS:
            out = _INV[sat(_LEVEL[el] - _LEVEL[rl])]
            rules.append(ctrl.Rule(e_psi[el] & r_err[rl], dw[out]))
    return ctrl.ControlSystemSimulation(ctrl.ControlSystem(rules))


class YawFuzzy:
    """Stateless evaluator wrapping a skfuzzy ControlSystemSimulation."""

    def __init__(self):
        self._sim = _build()

    def compute(self, e_psi_deg, r_err_dps):
        self._sim.input["e_psi"] = float(np.clip(e_psi_deg, -P.YAW_EPSI_RANGE, P.YAW_EPSI_RANGE))
        self._sim.input["r_err"] = float(np.clip(r_err_dps, -P.YAW_RERR_RANGE, P.YAW_RERR_RANGE))
        self._sim.compute()
        return float(self._sim.output["dw_yaw"])
