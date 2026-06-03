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
                 fuzzy_enabled=True, logger=None, verbose=True, print_hz=2.0):
        self.link = link
        self.state = state
        self.command = command_source        # callable() -> (v_cmd, w_cmd, mode)
        self.dt = 1.0 / hz
        self.sup = Supervisor(self.dt, fuzzy_enabled=fuzzy_enabled)
        self.logger = logger
        self.verbose = verbose
        self.seq = 0
        self._running = False
        self._print_every = max(1, int(round(hz / print_hz)))
        self._overruns = 0                   # ticks that blew the period budget
        self._tick = 0
        self._t_last_print = 0.0
        self._tick_at_last_print = 0

    def run(self):
        self._running = True
        next_t = time.monotonic()
        self._t_last_print = next_t
        while self._running:
            next_t += self.dt
            v_cmd, w_cmd, mode = self.command()
            telem, _ = self.state.latest()

            dbg = None
            if telem is None or not self.state.link_ok(P.CMD_TIMEOUT_S):
                self.link.send_cmd(0.0, 0.0, P.MODE_IDLE, self.seq)
            else:
                wl, wr, dbg = self.sup.step(v_cmd, w_cmd, telem)
                self.link.send_cmd(wl, wr, mode, self.seq)
                if self.logger:
                    self.logger.log(self.seq, v_cmd, w_cmd, telem, dbg)
            self.seq = (self.seq + 1) & 0xFFFF

            self._tick += 1
            if self.verbose and self._tick % self._print_every == 0:
                self._print_status(v_cmd, w_cmd, telem, dbg)

            slack = next_t - time.monotonic()
            if slack > 0:
                time.sleep(slack)
            else:
                self._overruns += 1
                next_t = time.monotonic()    # fell behind -> resync, don't spiral

    def _print_status(self, v_cmd, w_cmd, telem, dbg):
        now = time.monotonic()
        span = now - self._t_last_print
        rate = (self._tick - self._tick_at_last_print) / span if span > 0 else 0.0
        self._t_last_print = now
        self._tick_at_last_print = self._tick

        if dbg is None:
            print(f"[{rate:5.0f}Hz over={self._overruns}] link down -> commanding IDLE")
            return
        print(
            f"[{rate:5.0f}Hz over={self._overruns}] "
            f"cmd(v={v_cmd:+.2f} w={w_cmd:+.2f}) | "
            f"sig={dbg['sigma_err']:.2f} epsi={dbg['e_psi_deg']:+5.1f} rerr={dbg['r_err_dps']:+6.1f} | "
            f"lam={dbg['lam']:.2f} dw={dbg['dw_yaw']:+.2f} | "
            f"ref={dbg['omega_ref_l']:+5.2f}/{dbg['omega_ref_r']:+5.2f} "
            f"meas={telem['omega_meas_l']:+5.2f}/{telem['omega_meas_r']:+5.2f}"
        )

    def stop(self):
        self._running = False
