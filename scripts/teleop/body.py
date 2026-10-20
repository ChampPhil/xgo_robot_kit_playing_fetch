"""The robot's body: legs, arm, claw and kneel, as commanded by the receiver loop.

Each controller sends only changed values to xgolib and can report what it sent to a
listener (the recorder). Limits measured on this robot are documented next to them.
"""

import math
import time

from protocol import AXES


LIMITS = {"x": 25, "y": 18, "yaw": 100}  # xgolib VX/VY/VYAW limits for the Lite
ARM_LIMITS = {"x": (-80, 155), "z": (-95, 155), "claw": (0, 255)}  # arm(x, z) mm; claw 0=open
# The arm only reaches a ring around its base; the firmware silently ignores targets outside
# it, so the arm "freezes". Measured: (140, 30) and (80, 115) followed, (150, 30), (80, 130)
# and (55, 30) ignored; xgolib's arm_polar documents radius 80-140 mm. Kept in front (x >= 0).
ARM_REACH = (80, 140)
MAX_ARM_STEP = 0.1  # seconds; a late message never makes the arm jump to catch up


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
        self.listener = None  # called as listener("walk", steps) when the legs' command changes

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
        self.notify()

    def stop(self):
        was_moving = any(self.current.values())
        self.dog.stop()
        self.current = dict.fromkeys(AXES, 0)
        if was_moving:
            self.notify()

    def notify(self):
        if self.listener:
            self.listener("walk", dict(self.current))


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

    def forget(self):
        """After a full robot reset the arm's pose and claw are unknown again."""
        self.pose = self.grip = self.last = None
        self.sent = {"arm": None, "claw": None}

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

    def forget(self):
        """After a full robot reset the body is back at neutral (standing)."""
        self.level, self.last, self.sent = 0.0, None, None

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


class LegStepper:
    """Alignment's legs: one primitive at a small fixed speed until stop() (sign = direction)."""

    AXIS = {"turn": "yaw", "forward": "x", "strafe": "y"}

    def __init__(self, driver, speeds):
        self.driver = driver
        self.speeds = speeds

    def step(self, primitive, amount):
        """Move with `primitive` at |amount| (0..1] of its speed; the sign gives the direction."""
        steps = dict.fromkeys(AXES, 0)
        speed = self.speeds[primitive] * min(1.0, abs(amount))
        steps[self.AXIS[primitive]] = int(round(math.copysign(max(1.0, speed), amount)))
        self.driver.apply(steps)

    def stop(self):
        self.driver.stop()


class PosePreparer:
    """Put the kneel level and arm where a recording's reference photo was taken."""

    def __init__(self, arm, posture):
        self.arm = arm
        self.posture = posture

    def prepare(self, kneel, arm):
        self.posture.set_level(kneel or 0.0)
        if arm:
            self.arm.set_pose(*arm)


def robot_state(driver, arm, posture):
    """Snapshot of what has been commanded, for recordings."""
    return {"arm": list(arm.sent["arm"]) if arm.sent["arm"] else None,
            "claw": arm.sent["claw"], "kneel": round(posture.level, 4),
            "walk": dict(driver.current)}
