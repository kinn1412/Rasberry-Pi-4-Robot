"""The supervisor loop — runs at COMM_LOOP_HZ (100 Hz).

Each tick: read the freshest telemetry, run the fuzzy supervisor, send the
modulated wheel setpoints. If telemetry is stale (link down) it commands IDLE;
the ESP's own command watchdog independently stops the motors after 200 ms, so
this is defence in depth, not the only guard.
"""
import time

from probot_pi.bsp import params as P
from probot_pi.control.supervisor import Supervisor


class MainLoop:
    def __init__(self, link, state, command_source, hz=P.COMM_LOOP_HZ,
                 fuzzy_enabled=True, logger=None):
        self.link = link
        self.state = state
        self.command = command_source        # callable() -> (v_cmd, w_cmd, mode)
        self.dt = 1.0 / hz
        self.sup = Supervisor(self.dt, fuzzy_enabled=fuzzy_enabled)
        self.logger = logger
        self.seq = 0
        self._running = False

    def run(self):
        self._running = True
        next_t = time.monotonic()
        while self._running:
            next_t += self.dt
            v_cmd, w_cmd, mode = self.command()
            telem, _ = self.state.latest()

            if telem is None or not self.state.link_ok(P.CMD_TIMEOUT_S):
                self.link.send_cmd(0.0, 0.0, P.MODE_IDLE, self.seq)
            else:
                wl, wr, dbg = self.sup.step(v_cmd, w_cmd, telem)
                self.link.send_cmd(wl, wr, mode, self.seq)
                if self.logger:
                    self.logger.log(self.seq, v_cmd, w_cmd, telem, dbg)
            self.seq = (self.seq + 1) & 0xFFFF

            slack = next_t - time.monotonic()
            if slack > 0:
                time.sleep(slack)
            else:
                next_t = time.monotonic()    # fell behind -> resync, don't spiral

    def stop(self):
        self._running = False
