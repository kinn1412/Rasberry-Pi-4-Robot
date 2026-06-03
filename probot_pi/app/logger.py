"""CSV logger for the supervisor pipeline — one row per command tick.

Captures command, raw telemetry, and every intermediate fuzzy signal so a run
can be replayed/plotted offline (Phase 8 step 5, Phase 11 results).
"""
import csv
import time

FIELDS = [
    "t", "seq", "v_cmd", "w_cmd",
    "omega_meas_l", "omega_meas_r", "yaw", "yaw_rate",
    "wE_l", "wE_r", "e_psi_deg", "r_err_dps", "sigma_err",
    "dw_yaw", "lam", "omega_ref_l", "omega_ref_r",
    "pwm_l", "pwm_r", "vbat", "fault_flags",
]
_DBG_KEYS = ("wE_l", "wE_r", "e_psi_deg", "r_err_dps", "sigma_err",
             "dw_yaw", "lam", "omega_ref_l", "omega_ref_r")


class CsvLogger:
    def __init__(self, path):
        self._f = open(path, "w", newline="")
        self._w = csv.DictWriter(self._f, fieldnames=FIELDS)
        self._w.writeheader()
        self._t0 = time.monotonic()

    def log(self, seq, v_cmd, w_cmd, telem, dbg):
        row = {
            "t": round(time.monotonic() - self._t0, 4), "seq": seq,
            "v_cmd": v_cmd, "w_cmd": w_cmd,
            "omega_meas_l": round(telem["omega_meas_l"], 5),
            "omega_meas_r": round(telem["omega_meas_r"], 5),
            "yaw": round(telem["yaw"], 5), "yaw_rate": round(telem["yaw_rate"], 5),
            "pwm_l": telem["pwm_l"], "pwm_r": telem["pwm_r"],
            "vbat": telem["vbat"], "fault_flags": telem["fault_flags"],
        }
        row.update({k: round(dbg[k], 5) for k in _DBG_KEYS})
        self._w.writerow(row)

    def close(self):
        self._f.close()
