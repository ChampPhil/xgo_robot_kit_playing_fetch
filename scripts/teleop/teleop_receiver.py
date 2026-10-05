"""Drive the XGO Lite from gamepad commands streamed as JSON lines on stdin.

Runs on the robot. Each line is a JSON object from gamepad_sender.py, piped over
SSH, with axes in -1..1:

  {"x": fwd, "y": left, "yaw": ccw, "enable": bool,     walking (enable = LB held)
   "arm": bool, "ax": reach, "az": lift, "claw": close,  arm mode (arm = RB held)
   "bark": presses,                                      B-button press counter
   "kneel": 1 | 0 | -1}                                  LT + D-pad down / up

Walking stops when the sticks centre, enable drops, arm mode starts, input goes
quiet for --timeout seconds, the stream ends, or anything goes wrong. In arm mode
the sticks set arm/claw *speed*; the arm holds its position when they centre.
Kneeling pitches the front down and lowers the body (the vendor's floor-pickup
posture); it holds when released and returns to standing on exit.
"""

import argparse
import json
import math
import queue
import signal
import subprocess
import sys
import threading
import time
from importlib import import_module
from pathlib import Path
from typing import NamedTuple, Optional

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "sensor_monitor"))

AXES = ("x", "y", "yaw")
ARM_AXES = ("ax", "az", "claw")
LIMITS = {"x": 25, "y": 18, "yaw": 100}  # xgolib VX/VY/VYAW limits for the Lite
ARM_LIMITS = {"x": (-80, 155), "z": (-95, 155), "claw": (0, 255)}  # arm(x, z) mm; claw 0=open
# The arm only reaches a ring around its base; the firmware silently ignores targets outside
# it, so the arm "freezes". Measured: (140, 30) and (80, 115) followed, (150, 30), (80, 130)
# and (55, 30) ignored; xgolib's arm_polar documents radius 80-140 mm. Kept in front (x >= 0).
ARM_REACH = (80, 140)
MAX_ARM_STEP = 0.1  # seconds; a late message never makes the arm jump to catch up
BARK_SOUND = Path(__file__).resolve().parent / "sounds" / "bark.wav"
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
        "--arm-speed", type=lambda v: bounded_number(v, minimum=5, maximum=150, label="arm-speed"),
        default=60, metavar="MM_PER_S", help="full-stick arm speed (5–150 mm/s; default: 60)")
    parser.add_argument(
        "--claw-speed", type=lambda v: bounded_number(v, minimum=20, maximum=500, label="claw-speed"),
        default=170, metavar="UNITS_PER_S",
        help="full-stick claw speed (20–500 of 0–255 per s; default: 170)")
    for axis, default in (("x", 80), ("z", 30)):  # measured reachable
        low, high = ARM_LIMITS[axis]
        parser.add_argument(
            f"--arm-home-{axis}", type=lambda v, lo=low, hi=high, name=axis: bounded_number(
                v, minimum=lo, maximum=hi, label=f"arm-home-{name}"),
            default=default, metavar="MM",
            help=f"arm {axis} the first arm-mode stick move starts from ({low}…{high}; default: {default})")
    parser.add_argument("--claw-start", type=lambda v: bounded_number(v, minimum=0, maximum=255,
                                                                      label="claw-start"),
                        default=128, metavar="VALUE",
                        help="assumed claw position before the first claw move (default: 128)")
    parser.add_argument(
        "--ramp", type=lambda v: bounded_number(v, minimum=0, maximum=10, label="ramp"),
        default=0, metavar="SECONDS",
        help="grow forward/back and side-step strides to the full --max-x/--max-y over this "
             "long while held in one direction (0 = off, the default)")
    parser.add_argument(
        "--ramp-start", type=lambda v: bounded_number(v, minimum=0.1, maximum=1, label="ramp-start"),
        default=0.4, metavar="FRACTION",
        help="fraction of the full stride a ramped move starts at (default: 0.4)")
    parser.add_argument(
        "--kneel-pitch", type=lambda v: bounded_number(v, minimum=0, maximum=10, label="kneel-pitch"),
        default=10, metavar="DEGREES", help="front-down pitch when fully knelt (0–10; default: 10)")
    parser.add_argument(
        "--kneel-height", type=lambda v: bounded_number(v, minimum=60, maximum=110,
                                                        label="kneel-height"),
        default=70, metavar="MM", help="body height when fully knelt (60–110; default: 70)")
    parser.add_argument(
        "--stand-height", type=lambda v: bounded_number(v, minimum=60, maximum=110,
                                                        label="stand-height"),
        default=85, metavar="MM",
        help="body height when standing; 85 is xgolib's neutral value (60–110; default: 85)")
    parser.add_argument(
        "--kneel-time", type=lambda v: bounded_number(v, minimum=0.5, maximum=10, label="kneel-time"),
        default=2, metavar="SECONDS", help="time from standing to fully knelt (default: 2)")
    parser.add_argument(
        "--timeout", type=lambda v: bounded_number(v, minimum=0.1, maximum=2, label="timeout"),
        default=0.5, metavar="SECONDS", help="stop if no command arrives this long (default: 0.5)",
    )
    parser.add_argument("--min-battery", type=int, default=20, metavar="PCT",
                        help="refuse to start below this battery percentage (default: 20)")
    parser.add_argument("--port", default="/dev/ttyAMA0", help="serial port (default: /dev/ttyAMA0)")
    return parser


def axis_value(message, axis):
    value = message.get(axis, 0)
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{axis} must be a finite number")
    return max(-1.0, min(1.0, float(value)))


class Message(NamedTuple):
    walk: dict
    arm: Optional[dict]
    bark: int
    kneel: int
    record: int
    replay: int
    raw: dict


def counter(message, key):
    value = message.get(key, 0)
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{key} must be a non-negative integer")
    return value


def parse_message(line):
    """Parse one JSON command line into a Message. Raises ValueError on bad input.

    walk is {axis: -1..1}, all zero unless enable is true and arm mode is off.
    arm is {ax, az, claw: -1..1} in arm mode, else None. bark/record/replay are press
    counters. kneel is 1 (kneel down), -1 (stand up) or 0.
    """
    message = json.loads(line)
    if not isinstance(message, dict):
        raise ValueError("command must be a JSON object")
    bark, record, replay = (counter(message, key) for key in ("bark", "record", "replay"))
    kneel = message.get("kneel", 0)
    if isinstance(kneel, bool) or kneel not in (-1, 0, 1):
        raise ValueError("kneel must be -1, 0 or 1")
    arm = None
    if message.get("arm") is True:
        arm = {axis: axis_value(message, axis) for axis in ARM_AXES}
    walk = dict.fromkeys(AXES, 0.0)
    if message.get("enable") is True and arm is None:
        walk = {axis: axis_value(message, axis) for axis in AXES}
    return Message(walk, arm, bark, kneel, record, replay, message)


def parse_command(line):
    """Walking part of a message only (see parse_message)."""
    return parse_message(line).walk


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


class Ramp:
    """Stride multiplier for x/y that grows the longer one direction is held."""

    def __init__(self, seconds, start, clock=time.monotonic):
        self.seconds = seconds
        self.start = start
        self.clock = clock
        self.since = {}  # axis -> (sign, time the direction started)

    def reset(self):
        self.since.clear()

    def apply(self, walk):
        if not self.seconds:
            return walk
        now = self.clock()
        walk = dict(walk)
        for axis in ("x", "y"):
            sign = (walk[axis] > 0) - (walk[axis] < 0)
            if not sign:
                self.since.pop(axis, None)
                continue
            if self.since.get(axis, (None,))[0] != sign:
                self.since[axis] = (sign, now)
            held = now - self.since[axis][1]
            walk[axis] *= self.start + (1 - self.start) * min(held / self.seconds, 1.0)
        return walk


class ArmController:
    """Integrate stick speeds into arm(x, z)/claw targets; nothing is sent until a stick moves.

    The arm's real pose cannot be read back, so the first move starts from --arm-home-x/z
    (the arm travels there first) and the claw from --claw-start.
    """

    def __init__(self, dog, args, clock=time.monotonic):
        self.dog = dog
        self.args = args
        self.clock = clock
        self.last = None
        self.pose = None  # [x, z] once the arm has been commanded
        self.grip = None
        self.sent = {"arm": None, "claw": None}
        self.listener = None  # called as listener(kind, value) after each send (recording)

    def idle(self):
        self.last = None

    def update(self, command):
        if not any(command.values()):  # centred sticks: hold, and don't bank elapsed time
            self.idle()
            return
        now = self.clock()
        dt = 0.0 if self.last is None else min(now - self.last, MAX_ARM_STEP)
        self.last = now
        if command["ax"] or command["az"]:
            if self.pose is None:
                self.pose = [self.args.arm_home_x, self.args.arm_home_z]
            step = self.args.arm_speed * dt
            self.pose = move_within_reach(self.pose, command["ax"] * step, command["az"] * step)
        if command["claw"]:
            if self.grip is None:
                self.grip = self.args.claw_start
            self.grip = clamp(self.grip + command["claw"] * self.args.claw_speed * dt,
                              ARM_LIMITS["claw"])
        self.send()

    def set_pose(self, x, z):
        """Jump the arm target (used by replay); manual control continues from here."""
        self.pose = [float(x), float(z)]
        self.idle()
        self.send()

    def set_claw(self, value):
        self.grip = float(clamp(value, ARM_LIMITS["claw"]))
        self.send()

    def send(self):
        if self.pose is not None:
            target = (int(round(self.pose[0])), int(round(self.pose[1])))
            if target != self.sent["arm"]:
                self.dog.arm(*target)
                self.sent["arm"] = target
                if self.listener:
                    self.listener("arm", list(target))
        if self.grip is not None:
            target = int(round(self.grip))
            if target != self.sent["claw"]:
                self.dog.claw(target)
                self.sent["claw"] = target
                if self.listener:
                    self.listener("claw", target)


def clamp(value, limits):
    return max(limits[0], min(limits[1], value))


def in_reach(x, z):
    """True for targets the arm follows: in front, inside the reach ring and the API box."""
    return (x >= 0 and ARM_REACH[0] <= math.hypot(x, z) <= ARM_REACH[1]
            and ARM_LIMITS["x"][0] <= x <= ARM_LIMITS["x"][1]
            and ARM_LIMITS["z"][0] <= z <= ARM_LIMITS["z"][1])


def move_within_reach(pose, dx, dz):
    """Move as far along (dx, dz) as stays in reach, then stop at the edge (no sliding).

    The target therefore never leaves what the arm can follow, so reversing a stick
    always moves the arm straight away.
    """
    x, z = pose
    if not in_reach(x, z):  # e.g. an unreachable --arm-home: take the step as asked
        return [clamp(x + dx, ARM_LIMITS["x"]), clamp(z + dz, ARM_LIMITS["z"])]
    if in_reach(x + dx, z + dz):
        return [x + dx, z + dz]
    low, high = 0.0, 1.0  # bisect for the furthest reachable fraction of the step
    for _ in range(20):
        middle = (low + high) / 2
        if in_reach(x + dx * middle, z + dz * middle):
            low = middle
        else:
            high = middle
    return [x + dx * low, z + dz * low]


class Posture:
    """Kneel level 0 (standing) .. 1 (front pitched down, body lowered), changed at a fixed rate."""

    def __init__(self, dog, args, clock=time.monotonic):
        self.dog = dog
        self.args = args
        self.clock = clock
        self.level = 0.0
        self.last = None
        self.sent = None
        self.listener = None  # called as listener("kneel", level) after each send (recording)

    def update(self, direction):
        if not direction or (direction < 0 and self.level <= 0) or (direction > 0 and self.level >= 1):
            self.last = None  # released or at the end: hold, and don't bank elapsed time
            return
        now = self.clock()
        dt = 0.0 if self.last is None else min(now - self.last, MAX_ARM_STEP)
        self.last = now
        self.level = clamp(self.level + direction * dt / self.args.kneel_time, (0.0, 1.0))
        self.send()

    def set_level(self, level):
        """Jump to a kneel level (used by replay)."""
        self.level = clamp(float(level), (0.0, 1.0))
        self.last = None
        self.send()

    def stand(self):
        if self.sent is not None:
            self.level = 0.0
            self.send()

    def send(self):
        pitch = round(self.level * self.args.kneel_pitch, 2)
        height = round(self.args.stand_height
                       + self.level * (self.args.kneel_height - self.args.stand_height), 2)
        if (pitch, height) != self.sent:
            self.dog.attitude("p", pitch)  # + is front down (xgolib's floor-pickup demo)
            self.dog.translation("z", height)
            self.sent = (pitch, height)
            if self.listener:
                self.listener("kneel", round(self.level, 4))


def robot_state(driver, arm, posture):
    """Snapshot of what has been commanded, for recordings."""
    return {"arm": list(arm.sent["arm"]) if arm.sent["arm"] else None,
            "claw": arm.sent["claw"], "kneel": round(posture.level, 4),
            "walk": dict(driver.current)}


class Barker:
    """Play the bark when the press counter increases; ignore presses while one is playing."""

    def __init__(self, play=None, log=print):
        self.play = play or (lambda: subprocess.Popen(
            ["aplay", "-q", str(BARK_SOUND)], stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
        self.log = log
        self.seen = 0  # each sender run starts its own receiver, both counting from 0
        self.playing = None

    def update(self, count):
        if count <= self.seen:
            self.seen = count
            return
        self.seen = count
        if self.playing is not None and self.playing.poll() is None:
            return
        try:
            self.playing = self.play()
        except OSError as exc:
            self.log(f"Cannot play bark ({exc}).")


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


def run(driver, lines, timeout, log=print, first=None, arm=None, barker=None, ramp=None,
        posture=None):
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
                    if ramp is not None:
                        ramp.reset()
                    if arm is not None:
                        arm.idle()
                    if posture is not None:
                        posture.update(0)
                    continue
            line = latest(lines, line)
            if line is EOF:
                log("Command stream ended; stopping.")
                return
            try:
                message = parse_message(line)
                walk, arm_command, bark, kneel = (message.walk, message.arm, message.bark,
                                                  message.kneel)
            except ValueError as exc:
                log(f"Bad command ({exc}); stopping.")
                driver.stop()
                if ramp is not None:
                    ramp.reset()
                if arm is not None:
                    arm.idle()
                if posture is not None:
                    posture.update(0)
                continue
            if ramp is not None:
                walk = ramp.apply(walk)
            driver.apply(scale(walk, driver.args))
            if arm is not None:
                if arm_command is None:
                    arm.idle()
                else:
                    arm.update(arm_command)
            if barker is not None:
                barker.update(bark)
            if posture is not None:
                posture.update(kneel)
    finally:
        driver.stop()
        if posture is not None:
            posture.stand()


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
        if not BARK_SOUND.is_file():
            log(f"Bark sound missing at {BARK_SOUND}; B will do nothing.")
        run(driver, lines, args.timeout, log, first=first,
            arm=ArmController(dog, args), barker=Barker(log=log),
            ramp=Ramp(args.ramp, args.ramp_start), posture=Posture(dog, args))
    finally:
        driver.stop()
        log("Robot stopped.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
