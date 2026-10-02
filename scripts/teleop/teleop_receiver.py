"""Drive the XGO Lite from gamepad commands streamed as JSON lines on stdin.

Runs on the robot. Each line is {"x": fwd, "y": left, "yaw": ccw, "enable": bool}
with axes in -1..1; gamepad_sender.py produces them on the Mac and pipes them
over SSH. The robot stops when the sticks centre, enable drops, input goes
quiet for --timeout seconds, the stream ends, or anything goes wrong.
"""

import argparse
import json
import math
import queue
import signal
import sys
import threading
from importlib import import_module
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "sensor_monitor"))

AXES = ("x", "y", "yaw")
LIMITS = {"x": 25, "y": 18, "yaw": 100}  # xgolib VX/VY/VYAW limits for the Lite
EOF = object()


def bounded_number(value, *, minimum, maximum, label):
    try:
        number = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"{label} must be a number") from exc
    if not math.isfinite(number) or not minimum <= number <= maximum:
        raise argparse.ArgumentTypeError(f"{label} must be between {minimum} and {maximum}")
    return number


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    for flag, axis, default, unit in (("max-x", "x", 12, "mm"), ("max-y", "y", 8, "mm"),
                                      ("max-turn", "yaw", 40, "degrees/s")):
        maximum = LIMITS[axis]
        parser.add_argument(
            f"--{flag}", type=lambda v, m=maximum, name=flag: bounded_number(
                v, minimum=1, maximum=m, label=name),
            default=default, metavar="VALUE",
            help=f"full-stick {axis} speed (1–{maximum} {unit}; default: {default})",
        )
    parser.add_argument(
        "--timeout", type=lambda v: bounded_number(v, minimum=0.1, maximum=2, label="timeout"),
        default=0.5, metavar="SECONDS", help="stop if no command arrives this long (default: 0.5)",
    )
    parser.add_argument("--min-battery", type=int, default=20, metavar="PCT",
                        help="refuse to start below this battery percentage (default: 20)")
    parser.add_argument("--port", default="/dev/ttyAMA0", help="serial port (default: /dev/ttyAMA0)")
    return parser


def parse_command(line):
    """Return {axis: -1..1}, all zero unless enable is true. Raises ValueError on bad input."""
    message = json.loads(line)
    if not isinstance(message, dict):
        raise ValueError("command must be a JSON object")
    if message.get("enable") is not True:
        return dict.fromkeys(AXES, 0.0)
    command = {}
    for axis in AXES:
        value = message.get(axis, 0)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f"{axis} must be a finite number")
        command[axis] = max(-1.0, min(1.0, float(value)))
    return command


def scale(command, args):
    """Convert normalised axes to whole xgolib steps (small jitter then doesn't resend)."""
    maxima = {"x": args.max_x, "y": args.max_y, "yaw": args.max_turn}
    return {axis: int(round(command[axis] * maxima[axis])) for axis in AXES}


class Driver:
    """Send only changed axes; a single stop() when everything returns to zero."""

    def __init__(self, dog, args):
        self.dog = dog
        self.args = args
        self.current = dict.fromkeys(AXES, 0)

    def apply(self, steps):
        if steps == self.current:
            return
        if not any(steps.values()):
            self.stop()
            return
        senders = {"x": self.dog.move_x, "y": self.dog.move_y, "yaw": self.dog.turn}
        for axis in AXES:
            if steps[axis] != self.current[axis]:
                senders[axis](steps[axis])
        self.current = dict(steps)

    def stop(self):
        self.dog.stop()
        self.current = dict.fromkeys(AXES, 0)


def read_lines(stream, lines):
    for line in stream:
        lines.put(line)
    lines.put(EOF)


def latest(lines, line):
    """Skip queued commands that went stale while the UART was busy; never skip EOF."""
    while line is not EOF:
        try:
            line = lines.get_nowait()
        except queue.Empty:
            break
    return line


def run(driver, lines, timeout, log=print, first=None):
    """Apply commands until the stream ends; always leaves the robot stopped."""
    try:
        while True:
            if first is not None:
                line, first = first, None
            else:
                try:
                    line = lines.get(timeout=timeout)
                except queue.Empty:
                    if any(driver.current.values()):
                        log("No command for {:.1f}s; stopping.".format(timeout))
                        driver.stop()
                    continue
            line = latest(lines, line)
            if line is EOF:
                log("Command stream ended; stopping.")
                return
            try:
                steps = scale(parse_command(line), driver.args)
            except ValueError as exc:
                log(f"Bad command ({exc}); stopping.")
                driver.stop()
                continue
            driver.apply(steps)
    finally:
        driver.stop()


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    log = lambda text: print(text, file=sys.stderr, flush=True)  # noqa: E731 - stdout stays unused
    try:
        from stream_sensors import require_free_port
        require_free_port(args.port)
        xgolib = import_module("xgolib")
    except (ImportError, RuntimeError) as exc:
        parser.exit(2, f"teleop_receiver: {exc}\n")

    for signum in (signal.SIGHUP, signal.SIGTERM):  # SSH teardown: still run stop() below
        signal.signal(signum, lambda *_: sys.exit(1))
    lines = queue.Queue()
    threading.Thread(target=read_lines, args=(sys.stdin, lines), daemon=True).start()
    first = lines.get()  # the sender only starts streaming after the operator confirms
    if first is EOF:
        parser.exit(0, "teleop_receiver: no commands received; robot untouched.\n")

    log("Connecting to the robot (XGO init resets and stands the dog)...")
    dog = xgolib.XGO(port=args.port, version="xgolite")
    driver = Driver(dog, args)
    try:
        battery = dog.read_battery() if hasattr(dog, "read_battery") else None
        if isinstance(battery, (int, float)) and 0 < battery < args.min_battery:
            log(f"Battery {battery}% is below {args.min_battery}%; refusing to drive.")
            return 3
        log(f"Ready (battery {battery}%). Limits x={args.max_x} y={args.max_y} turn={args.max_turn}.")
        run(driver, lines, args.timeout, log, first=first)
    finally:
        driver.stop()
        log("Robot stopped.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
