"""Run short, explicitly selected XGO Lite movements from the command line."""

import argparse
import math
import time
from importlib import import_module

MOVEMENTS = ("forward", "backward", "left", "right", "turn-left", "turn-right")


def bounded_number(value, *, minimum, maximum, label):
    try:
        number = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"{label} must be a number") from exc
    if not math.isfinite(number) or not minimum <= number <= maximum:
        raise argparse.ArgumentTypeError(f"{label} must be between {minimum} and {maximum}")
    return number


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--movement", action="append", required=True, choices=(*MOVEMENTS, "all"),
        help="movement to test; repeat the flag to run multiple in order, or use 'all'",
    )
    parser.add_argument(
        "--duration", type=lambda v: bounded_number(v, minimum=0.1, maximum=5, label="duration"),
        default=1.0, metavar="SECONDS", help="time per movement (0.1–5; default: 1)",
    )
    parser.add_argument(
        "--pause", type=lambda v: bounded_number(v, minimum=0, maximum=60, label="pause"),
        default=0.5, metavar="SECONDS", help="rest between movements (0–60; default: 0.5)",
    )
    for flag, maximum, default, unit in (
        ("x-step", 25, 10, "mm"),
        ("y-step", 18, 8, "mm"),
        ("turn-speed", 100, 30, "degrees/s"),
    ):
        parser.add_argument(
            f"--{flag}", type=lambda v, m=maximum, name=flag: bounded_number(
                v, minimum=1, maximum=m, label=name
            ),
            default=default, metavar="VALUE",
            help=f"{flag} magnitude (1–{maximum} {unit}; default: {default})",
        )
    parser.add_argument("--port", default="/dev/ttyAMA0", help="serial port (default: /dev/ttyAMA0)")
    parser.add_argument("--yes", action="store_true", help="skip the safety confirmation prompt")
    return parser


def run_test(dog, args):
    commands = {
        "forward": (dog.move_x, args.x_step),
        "backward": (dog.move_x, -args.x_step),
        "left": (dog.move_y, args.y_step),
        "right": (dog.move_y, -args.y_step),
        "turn-left": (dog.turn, args.turn_speed),
        "turn-right": (dog.turn, -args.turn_speed),
    }
    selected = MOVEMENTS if args.movement == ["all"] else args.movement
    try:
        for index, name in enumerate(selected):
            command, value = commands[name]
            print(f"Testing {name}...")
            try:
                command(value)
                time.sleep(args.duration)
            finally:
                dog.stop()  # movement commands persist until explicitly stopped
            if index < len(selected) - 1:
                time.sleep(args.pause)
    finally:
        dog.stop()  # stop on Ctrl+C or errors, too
    print("Movement test complete.")


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    if "all" in args.movement and len(args.movement) != 1:
        parser.error("'all' cannot be combined with other movements")
    try:
        xgolib = import_module("xgolib")
    except ImportError:
        parser.exit(2, "xgolib is missing; run ./setup.sh and use .venv/bin/python.\n")

    if not args.yes:
        try:
            input("Clear a level area around the robot. Press Enter to start (Ctrl+C to cancel)... ")
        except EOFError:
            parser.error("confirmation requires a terminal; use --yes only when it is safe")

    dog = xgolib.XGO(port=args.port, version="xgolite")
    run_test(dog, args)


if __name__ == "__main__":
    main()
