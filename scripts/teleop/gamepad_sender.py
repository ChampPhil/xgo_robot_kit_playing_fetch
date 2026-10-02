"""Stream gamepad sticks from this computer to the XGO Lite over SSH.

Left stick: forward/back and strafe left/right. Right stick (left/right): turn.
Hold LB to drive (dead-man switch); release it and the robot stops. Press
START or Ctrl+C to quit. Commands go to teleop_receiver.py on the robot through
the stdin of an SSH session, so the existing Tailscale SSH access is the link.
"""

import argparse
import json
import math
import os
import shlex
import subprocess
import sys
import time

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")  # no window to focus

DEFAULT_HOST = "pi@100.74.30.90"
DEFAULT_REMOTE_DIR = "~/project/xgo_robot_kit_playing_fetch"
AXIS_MAX = 32767


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
    parser.add_argument("--host", default=DEFAULT_HOST, help=f"SSH target (default: {DEFAULT_HOST})")
    parser.add_argument("--remote-dir", default=DEFAULT_REMOTE_DIR,
                        help=f"repo checkout on the robot (default: {DEFAULT_REMOTE_DIR})")
    parser.add_argument("--print", dest="print_only", action="store_true",
                        help="write commands to stdout instead of starting SSH (no robot)")
    parser.add_argument("--rate", type=lambda v: bounded_number(v, minimum=5, maximum=50, label="rate"),
                        default=20, metavar="HZ", help="commands per second (default: 20)")
    parser.add_argument("--deadzone", type=lambda v: bounded_number(v, minimum=0, maximum=0.5,
                                                                    label="deadzone"),
                        default=0.15, metavar="FRACTION", help="stick deadzone (default: 0.15)")
    parser.add_argument("--no-deadman", action="store_true",
                        help="drive without holding LB (sticks must be centred first)")
    parser.add_argument("--yes", action="store_true", help="skip the safety confirmation prompt")
    for flag in ("max-x", "max-y", "max-turn"):
        parser.add_argument(f"--{flag}", metavar="VALUE",
                            help="passed to teleop_receiver.py (see its --help for limits)")
    return parser


def apply_deadzone(value, deadzone):
    """Zero inside the deadzone, then rescale so output ramps smoothly from 0 to ±1."""
    magnitude = abs(value)
    if magnitude <= deadzone:
        return 0.0
    return math.copysign(min(1.0, (magnitude - deadzone) / (1.0 - deadzone)), value)


def make_command(left_x, left_y, right_x, enable, deadzone):
    """Map normalised stick axes (SDL: +x right, +y down) to robot axes.

    Robot axes follow xgolib: x forward, y left, yaw counter-clockwise (turn left).
    """
    command = {
        "x": apply_deadzone(-left_y, deadzone),
        "y": apply_deadzone(-left_x, deadzone),
        "yaw": apply_deadzone(-right_x, deadzone),
    }
    command = {axis: round(value, 3) + 0.0 for axis, value in command.items()}  # no -0.0
    command["enable"] = bool(enable)
    return command


class Arming:
    """Without a dead-man switch, refuse to drive until all sticks have been centred once."""

    def __init__(self, deadman):
        self.deadman = deadman
        self.centred_once = deadman

    def enable(self, command, lb_held):
        if self.deadman:
            return lb_held
        if not any(command[axis] for axis in ("x", "y", "yaw")):
            self.centred_once = True
        return self.centred_once


def open_controller():
    import pygame
    from pygame._sdl2 import controller

    pygame.init()
    controller.init()
    for index in range(controller.get_count()):
        if controller.is_controller(index):
            return pygame, controller.Controller(index)
    raise RuntimeError(
        "no gamepad found. Plug it in; on a Logitech F310 set the back switch to D "
        "(macOS cannot read X/XInput mode)."
    )


def read_sticks(pygame, pad):
    pygame.event.pump()
    axis = lambda code: max(-1.0, pad.get_axis(code) / AXIS_MAX)  # noqa: E731
    return (axis(pygame.CONTROLLER_AXIS_LEFTX), axis(pygame.CONTROLLER_AXIS_LEFTY),
            axis(pygame.CONTROLLER_AXIS_RIGHTX),
            bool(pad.get_button(pygame.CONTROLLER_BUTTON_LEFTSHOULDER)),
            bool(pad.get_button(pygame.CONTROLLER_BUTTON_START)))


def remote_command(args):
    receiver = ["python3", "-u", "scripts/teleop/teleop_receiver.py"]
    for flag in ("max_x", "max_y", "max_turn"):
        if getattr(args, flag) is not None:
            receiver += ["--" + flag.replace("_", "-"), getattr(args, flag)]
    # remote_dir is left unquoted so the robot's shell expands "~".
    return f"cd {args.remote_dir} && exec {' '.join(shlex.quote(part) for part in receiver)}"


def start_ssh(args):
    return subprocess.Popen(
        ["ssh", "-T", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8",
         "-o", "ServerAliveInterval=2", "-o", "ServerAliveCountMax=3",
         args.host, remote_command(args)],
        stdin=subprocess.PIPE,
        start_new_session=True,  # Ctrl+C reaches only us; we then close stdin cleanly
    )


def status_line(command):
    state = "DRIVE" if command["enable"] else "hold "
    return "\r{}  fwd {:+.2f}  left {:+.2f}  turn {:+.2f} ".format(
        state, command["x"], command["y"], command["yaw"])


def stream(pygame, pad, out, args, ssh=None):
    arming = Arming(deadman=not args.no_deadman)
    period = 1.0 / args.rate
    next_tick = time.monotonic()
    while True:
        if ssh is not None and ssh.poll() is not None:
            raise RuntimeError(f"SSH session ended (exit {ssh.returncode})")
        left_x, left_y, right_x, lb_held, quit_pressed = read_sticks(pygame, pad)
        if quit_pressed:
            return
        command = make_command(left_x, left_y, right_x, enable=False, deadzone=args.deadzone)
        command["enable"] = arming.enable(command, lb_held)
        out.write(json.dumps(command) + "\n")
        out.flush()
        if ssh is not None:
            sys.stderr.write(status_line(command))
            sys.stderr.flush()
        next_tick += period
        time.sleep(max(0.0, next_tick - time.monotonic()))


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        pygame, pad = open_controller()
    except ImportError:
        parser.exit(2, "pygame-ce is missing; run: .venv/bin/python -m pip install -r "
                       "scripts/teleop/requirements-mac.txt\n")
    except RuntimeError as exc:
        parser.exit(2, f"gamepad_sender: {exc}\n")
    print(f"Gamepad: {pad.name}", file=sys.stderr)

    if args.print_only:
        try:
            stream(pygame, pad, sys.stdout, args)
        except (KeyboardInterrupt, BrokenPipeError):
            pass
        return 0

    if not args.yes:
        try:
            input("Robot on the floor (not a table), area clear? The robot will stand up. "
                  "Press Enter to connect (Ctrl+C to cancel)... ")
        except EOFError:
            parser.error("confirmation requires a terminal; use --yes only when it is safe")
    ssh = start_ssh(args)
    print("Hold LB and use the sticks. Release LB to stop; START or Ctrl+C quits."
          if not args.no_deadman else "Centre the sticks to arm. START or Ctrl+C quits.",
          file=sys.stderr)
    try:
        stream(pygame, pad, ssh.stdin, args, ssh)
    except KeyboardInterrupt:
        pass
    except (BrokenPipeError, RuntimeError) as exc:
        print(f"\ngamepad_sender: {exc}", file=sys.stderr)
    finally:
        try:
            ssh.stdin.close()  # EOF makes the receiver stop the robot and exit
        except BrokenPipeError:
            pass
        try:
            ssh.wait(timeout=5)
        except subprocess.TimeoutExpired:
            ssh.terminate()
        print("", file=sys.stderr)
    return 0 if ssh.returncode in (0, None) else ssh.returncode


if __name__ == "__main__":
    sys.exit(main())
