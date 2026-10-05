"""Stream gamepad sticks from this computer to the XGO Lite over SSH.

Hold LB to walk (dead-man switch): left stick forward/back and strafe, right
stick left/right turns. Release LB and the robot stops.
Hold RB for arm mode (walking stops): left stick up/down raises/lowers the arm,
right/left reaches out/in; right stick right/left closes/opens the claw.
Hold LT and press D-pad down/up to kneel the front down / stand back up.
B barks. X starts/stops recording a pickup, Y replays it. START or Ctrl+C quits. Commands go to teleop_receiver.py on the robot through
the stdin of an SSH session, so the existing Tailscale SSH access is the link.
"""

import argparse
import json
import math
import os
import shlex
import subprocess
import sys
import threading
import time
import webbrowser

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")  # no window to focus

DEFAULT_HOST = "pi@100.74.30.90"
DEFAULT_REMOTE_DIR = "~/project/xgo_robot_kit_playing_fetch"
AXIS_MAX = 32767
# Passed through unchanged to teleop_receiver.py, which validates them.
RECEIVER_FLAGS = ("max-x", "max-y", "max-turn", "ramp", "ramp-start", "arm-speed", "claw-speed",
                  "arm-home-x", "arm-home-z", "claw-start", "kneel-pitch", "kneel-height",
                  "stand-height", "kneel-time")


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
    parser.add_argument("--video", action="store_true",
                        help="show the robot camera live in the browser (port 8090)")
    for flag in RECEIVER_FLAGS:
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


def make_message(state, arming, deadzone, barks, records=0, replays=0):
    """Full command line: walking (LB), arm mode (RB, which overrides walking) and barks."""
    command = make_command(state["left_x"], state["left_y"], state["right_x"], False, deadzone)
    command["enable"] = arming.enable(command, state["lb"]) and not state["rb"]
    command["arm"] = state["rb"]
    # Arm mode, seen from the robot's side: stick right = reach out (+x), up = raise (+z);
    # right stick right = close the claw (towards 255).
    for key, value in (("ax", state["left_x"]), ("az", -state["left_y"]),
                       ("claw", state["right_x"])):
        command[key] = round(apply_deadzone(value, deadzone), 3) + 0.0 if state["rb"] else 0.0
    command["bark"] = barks
    command["record"] = records  # X presses: toggle recording
    command["replay"] = replays  # Y presses: replay the latest recording
    # LT + D-pad: kneel. Independent of LB/RB, so the arm still works while knelt.
    command["kneel"] = (int(state["dpad_down"]) - int(state["dpad_up"])) if state["lt"] else 0
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
    deadline = time.monotonic() + 2.0  # macOS reports devices only after a few event pumps
    while time.monotonic() < deadline:
        pygame.event.pump()
        for index in range(controller.get_count()):
            if controller.is_controller(index):
                return pygame, controller.Controller(index)
        time.sleep(0.05)
    raise RuntimeError(
        "no gamepad found. Plug it in; on a Logitech F310 set the back switch to D "
        "(macOS cannot read X/XInput mode)."
    )


def read_sticks(pygame, pad):
    pygame.event.pump()
    axis = lambda code: max(-1.0, pad.get_axis(code) / AXIS_MAX)  # noqa: E731
    button = lambda code: bool(pad.get_button(code))  # noqa: E731
    return {
        "left_x": axis(pygame.CONTROLLER_AXIS_LEFTX),
        "left_y": axis(pygame.CONTROLLER_AXIS_LEFTY),
        "right_x": axis(pygame.CONTROLLER_AXIS_RIGHTX),
        "lb": button(pygame.CONTROLLER_BUTTON_LEFTSHOULDER),
        "rb": button(pygame.CONTROLLER_BUTTON_RIGHTSHOULDER),
        "b": button(pygame.CONTROLLER_BUTTON_B),
        "x": button(pygame.CONTROLLER_BUTTON_X),
        "y": button(pygame.CONTROLLER_BUTTON_Y),
        "start": button(pygame.CONTROLLER_BUTTON_START),
        # The F310's triggers are digital in D mode; SDL reports them as 0 or full-scale axes.
        "lt": pad.get_axis(pygame.CONTROLLER_AXIS_TRIGGERLEFT) > AXIS_MAX // 2,
        "dpad_up": button(pygame.CONTROLLER_BUTTON_DPAD_UP),
        "dpad_down": button(pygame.CONTROLLER_BUTTON_DPAD_DOWN),
    }


def remote_command(args):
    receiver = ["python3", "-u", "scripts/teleop/teleop_receiver.py"]
    for flag in RECEIVER_FLAGS:
        value = getattr(args, flag.replace("-", "_"))
        if value is not None:
            receiver += ["--" + flag, value]
    if args.video:
        receiver += ["--video-port", str(VIDEO_PORT)]
    # remote_dir is left unquoted so the robot's shell expands "~".
    return f"cd {args.remote_dir} && exec {' '.join(shlex.quote(part) for part in receiver)}"


VIDEO_PORT = 8090


def video_url(args):
    return f"http://localhost:{VIDEO_PORT}/"  # forwarded through the SSH session


def start_ssh(args):
    # The tailnet only allows SSH to the robot, so live video is forwarded inside the session.
    tunnel = ["-L", f"{VIDEO_PORT}:127.0.0.1:{VIDEO_PORT}"] if args.video else []
    return subprocess.Popen(
        ["ssh", "-T", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8",
         "-o", "ServerAliveInterval=2", "-o", "ServerAliveCountMax=3", *tunnel,
         args.host, remote_command(args)],
        stdin=subprocess.PIPE,
        text=True,
        start_new_session=True,  # Ctrl+C reaches only us; we then close stdin cleanly
    )


def status_line(command):
    if command["arm"]:
        return "\rARM    reach {:+.2f}  lift {:+.2f}  claw {:+.2f}  barks {} ".format(
            command["ax"], command["az"], command["claw"], command["bark"])
    state = "DRIVE" if command["enable"] else "hold "
    kneel = {1: "  KNEEL v", -1: "  STAND ^"}.get(command["kneel"], "")
    return "\r{}  fwd {:+.2f}  left {:+.2f}  turn {:+.2f}  barks {}{} ".format(
        state, command["x"], command["y"], command["yaw"], command["bark"], kneel)


def stream(pygame, pad, out, args, ssh=None):
    arming = Arming(deadman=not args.no_deadman)
    period = 1.0 / args.rate
    presses = {"b": 0, "x": 0, "y": 0}
    was_down = dict.fromkeys(presses, False)
    next_tick = time.monotonic()
    while True:
        if ssh is not None and ssh.poll() is not None:
            raise RuntimeError(f"SSH session ended (exit {ssh.returncode})")
        state = read_sticks(pygame, pad)
        if state["start"]:
            return
        for button in presses:  # counters, so a skipped line cannot lose a press
            if state[button] and not was_down[button]:
                presses[button] += 1
            was_down[button] = state[button]
        command = make_message(state, arming, args.deadzone, presses["b"],
                               records=presses["x"], replays=presses["y"])
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
    if args.video:
        url = video_url(args)
        print(f"Live video: {url} (opens once the robot is ready)", file=sys.stderr)
        threading.Timer(4.0, webbrowser.open, args=(url,)).start()
    print(("Hold LB to walk" if not args.no_deadman else "Centre the sticks, then walk")
          + "; hold RB to move the arm/claw; LT + D-pad down/up kneels/stands; B barks; "
          "X records, Y replays; "
          "START or Ctrl+C quits.", file=sys.stderr)
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
