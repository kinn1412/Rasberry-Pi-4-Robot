"""probot_pi entry point — wire the serial (or sim) link, shared state, and the
fuzzy supervisor loop (or the read-only monitor) together.

  python -m probot_pi --monitor                       # Phase-7 link check, NO motion
  python -m probot_pi --sim --monitor --duration 3    # dry-run the monitor, no HW
  python -m probot_pi --sim --v 0.3 --log run.csv --duration 8
  python -m probot_pi --port /dev/serial0 --baud 921600 --v 0.3
  python -m probot_pi --no-fuzzy ...                  # PID-only baseline (pass-through)

The heavy stack (skfuzzy/numpy) is imported lazily, so --monitor works even
before scikit-fuzzy is installed.
"""
import argparse
import signal
import threading

from probot_pi.bsp import params as P
from probot_pi.hal.robot_state import RobotState


def _parse_args(argv):
    ap = argparse.ArgumentParser(prog="probot_pi", description="probot fuzzy supervisor (Pi side)")
    ap.add_argument("--port", default=P.PI_UART_PORT, help="serial device")
    ap.add_argument("--baud", type=int, default=P.PI_UART_BAUD)
    ap.add_argument("--rate", type=int, default=P.COMM_LOOP_HZ, help="supervisor loop Hz")
    ap.add_argument("--v", type=float, default=0.0, help="commanded body speed v (m/s)")
    ap.add_argument("--w", type=float, default=0.0, help="commanded body yaw rate (rad/s)")
    ap.add_argument("--no-fuzzy", action="store_true", help="pass-through (PID-only baseline)")
    ap.add_argument("--no-lut", action="store_true",
                    help="use exact skfuzzy each tick instead of the precomputed LUT (slow)")
    ap.add_argument("--sim", action="store_true", help="offline plant, no hardware")
    ap.add_argument("--log", default=None, help="CSV log path")
    ap.add_argument("--duration", type=float, default=0.0, help="auto-stop after N s (0=run forever)")
    ap.add_argument("--quiet", action="store_true", help="suppress the ~2 Hz status line")
    # read-only link verification (Phase 7)
    ap.add_argument("--monitor", action="store_true",
                    help="read-only link check: IDLE heartbeats + telemetry, NO motion")
    ap.add_argument("--no-heartbeat", action="store_true",
                    help="with --monitor: listen passively, do not send heartbeats")
    return ap.parse_args(argv)


def _make_link(args, state):
    if args.sim:
        from probot_pi.app.sim import SimLink
        return SimLink(state)
    from probot_pi.hal.serial_link import SerialLink
    return SerialLink(args.port, args.baud, on_telem=state.update)


def main(argv=None):
    args = _parse_args(argv)
    state = RobotState()
    link = _make_link(args, state)
    link.start()

    src = "SIM" if args.sim else f"{args.port}@{args.baud}"

    if args.monitor:
        from probot_pi.app.monitor import MonitorLoop
        loop = MonitorLoop(link, state, heartbeat=not args.no_heartbeat)
        print(f"probot_pi monitor: link={src}")
        _wire_stop(loop, args)
        try:
            loop.run()
        finally:
            link.stop()
            print(f"stopped. link stats: {link.stats}")
        return

    from probot_pi.app.main_loop import MainLoop
    from probot_pi.app.logger import CsvLogger

    logger = CsvLogger(args.log) if args.log else None
    command = lambda: (args.v, args.w, P.MODE_RUN)
    loop = MainLoop(link, state, command, hz=args.rate,
                    fuzzy_enabled=not args.no_fuzzy, logger=logger, verbose=not args.quiet,
                    backend="skfuzzy" if args.no_lut else "lut")

    print(f"probot_pi up: link={src}  fuzzy={'OFF' if args.no_fuzzy else 'ON'}  "
          f"rate={args.rate}Hz  cmd(v={args.v}, w={args.w})")
    _wire_stop(loop, args)
    try:
        loop.run()
    finally:
        link.stop()
        if logger:
            logger.close()
        print(f"stopped. link stats: {link.stats}")


def _wire_stop(loop, args):
    signal.signal(signal.SIGINT, lambda *_: loop.stop())
    if args.duration > 0:
        threading.Timer(args.duration, loop.stop).start()


if __name__ == "__main__":
    main()
