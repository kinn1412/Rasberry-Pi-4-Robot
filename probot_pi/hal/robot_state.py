"""Thread-safe holder for the latest telemetry from the ESP + link health.

The SerialLink RX thread calls `update()`; the supervisor loop calls `latest()`
and `link_ok()`. A plain lock is enough here (low rate, tiny payload) — no need
for the seqlock the firmware uses cross-core.
"""
import threading
import time


class RobotState:
    def __init__(self):
        self._lock = threading.Lock()
        self._telem = None
        self._ts = 0.0
        self._last_seq = None
        self._drops = 0          # telemetry frames lost (seq gaps), counted per-frame

    def update(self, telem):
        with self._lock:
            s = telem["seq"]
            if self._last_seq is not None:
                gap = (s - self._last_seq - 1) & 0xFFFF
                if 0 < gap < 1000:          # ignore wrap/resync glitches
                    self._drops += gap
            self._last_seq = s
            self._telem = telem
            self._ts = time.monotonic()

    def dropped(self):
        with self._lock:
            return self._drops

    def latest(self):
        """Return (telem_dict_copy_or_None, monotonic_timestamp)."""
        with self._lock:
            return (dict(self._telem) if self._telem else None), self._ts

    def age(self):
        with self._lock:
            return (time.monotonic() - self._ts) if self._telem else float("inf")

    def link_ok(self, timeout_s=0.2):
        return self.age() <= timeout_s
