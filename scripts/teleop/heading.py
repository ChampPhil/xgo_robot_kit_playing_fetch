"""Heading hold: keep walking straight by correcting drift with the robot's own gyro.

While the legs are commanded to walk without turning, the heading at the start is locked
and small corrective turns are added whenever the IMU's fused yaw drifts. Any requested
turn wins and re-locks afterwards. The gyro's direction (does turning left raise or lower
the reading?) is learned from the first real turn, because a wrong sign would make the
"correction" spin the robot faster and faster.

The fused yaw also drifts at a steady rate even when the robot stands still (6.9 deg/s
measured on this robot, never wrapped). That rate is measured whenever the legs have been
still for a moment and subtracted, so only real rotation is corrected.
"""

import json
import math
import struct
from collections import deque

from teach import write_json_atomic

YAW_REGISTER = 0x64
LEARN_DEGREES = 5.0
STILL_SETTLE = 0.5  # seconds after the legs stop before samples count
DRIFT_WINDOW = 5.0  # seconds of still samples kept for the drift fit
DRIFT_MIN_SPAN = 2.0  # seconds of still samples needed before trusting the fit


def fit_slope(samples):
    """Least-squares slope of (t, value) samples."""
    n = len(samples)
    mean_t = sum(t for t, _ in samples) / n
    mean_v = sum(v for _, v in samples) / n
    spread = sum((t - mean_t) ** 2 for t, _ in samples)
    return sum((t - mean_t) * (v - mean_v) for t, v in samples) / spread


def angle_diff(a, b):
    """a - b in degrees, wrapped to [-180, 180)."""
    return (a - b + 180.0) % 360.0 - 180.0


def make_yaw_reader(connection, timeout=0.05):
    """Read fused yaw with the logger's validated protocol: short timeout, raises on failure.

    (xgolib's read_yaw() waits up to 1 s and returns 0 on failure, which would look like a
    sudden turn.)
    """
    from stream_sensors import XGOReader

    reader = XGOReader(connection, timeout=timeout)

    def read():
        angle = struct.unpack("<f", reader.read_register(YAW_REGISTER, 4))[0]
        if not math.isfinite(angle):
            raise ValueError("non-finite yaw")
        return angle

    return read


class HeadingHold:
    def __init__(self, read_yaw, clock, log=print, sign=None, sign_path=None, enabled=True,
                 gain=2.0, max_turn=25, deadband=2.0, period=0.2, runaway=30.0, bias=None):
        self.read_yaw = read_yaw
        self.clock = clock
        self.log = log
        self.sign_path = sign_path
        self.enabled = enabled
        self.gain = gain
        self.max_turn = max_turn
        self.deadband = deadband
        self.period = period
        self.runaway = runaway
        self.sign = sign if sign is not None else self.load_sign()
        self.last_read = None
        self.failed = False
        self.reported_failure = False
        self.told_to_turn = False
        self.told_to_stand = False
        self.turn_start = None
        self.bias = bias  # gyro drift in deg/s; None until measured while standing still
        self.still_since = None
        self.still_samples = deque()
        self.reset()

    # --- persistence of the learned gyro direction --------------------------------------
    def load_sign(self):
        try:
            sign = json.loads(self.sign_path.read_text()).get("sign")
        except (AttributeError, OSError, ValueError):
            return None
        return sign if sign in (1, -1) else None

    def save_sign(self):
        if self.sign_path is None:
            return
        try:
            self.sign_path.parent.mkdir(parents=True, exist_ok=True)
            write_json_atomic(self.sign_path, {"version": 1, "sign": self.sign})
        except OSError as exc:
            self.log(f"Could not save the gyro direction ({exc}); it will be relearned.")

    # --- control ----------------------------------------------------------------------------
    def reset(self):
        self.target = None
        self.correction = 0.0

    def read(self):
        """The yaw if a reading is due and succeeds; None if not due or it failed."""
        now = self.clock()
        if self.last_read is not None and now - self.last_read < self.period - 1e-6:
            return None
        self.last_read = now
        try:
            value = self.read_yaw()
        except (OSError, TimeoutError, ValueError, struct.error) as exc:
            self.failed = True
            if not self.reported_failure:
                self.reported_failure = True
                self.log(f"Heading hold paused: no gyro reading ({exc}).")
            return None
        self.failed = self.reported_failure = False
        return value

    def rotation_since(self, yaw, start):
        """Real rotation since `start` = (yaw, time): measured change minus expected drift."""
        return angle_diff(yaw, start[0]) - self.bias * (self.clock() - start[1])

    def idle(self):
        """Called every tick while the legs are stopped: keep the drift estimate fresh."""
        if not self.enabled:
            return
        now = self.clock()
        if self.still_since is None:
            self.still_since = now
            self.still_samples.clear()
        if now - self.still_since < STILL_SETTLE:
            return
        yaw = self.read()
        if yaw is None:
            return
        self.still_samples.append((now, yaw))
        while now - self.still_samples[0][0] > DRIFT_WINDOW:
            self.still_samples.popleft()
        span = self.still_samples[-1][0] - self.still_samples[0][0]
        if span >= DRIFT_MIN_SPAN and len(self.still_samples) >= 8:
            first = self.bias is None
            self.bias = fit_slope(self.still_samples)
            if first:
                self.log(f"Heading hold: gyro drift measured at {self.bias:.1f} deg/s "
                         "(subtracted while walking).")

    def moving(self):
        self.still_since = None
        self.still_samples.clear()

    def learn(self, turn_command):
        if self.bias is None:
            return  # a slow turn plus unknown drift could be misread
        yaw = self.read()
        if yaw is None:
            return
        if self.turn_start is None:
            self.turn_start = (yaw, self.clock())
            return
        delta = self.rotation_since(yaw, self.turn_start)
        if abs(delta) >= LEARN_DEGREES:
            self.sign = 1 if delta * turn_command > 0 else -1
            self.save_sign()
            self.log("Heading hold ready: learned the gyro direction ({}).".format(
                "counter-clockwise positive" if self.sign == 1 else "clockwise positive"))

    def adjust(self, steps):
        """Return the walk command with a heading correction added when walking straight."""
        steps = dict(steps)
        if not self.enabled:
            return steps
        if any(steps.values()):
            self.moving()
        if steps["yaw"]:  # the operator (or a replay/alignment) is turning: theirs wins
            self.reset()
            if self.sign is None:
                self.learn(steps["yaw"])
            return steps
        self.turn_start = None
        if not (steps["x"] or steps["y"]):
            self.reset()
            return steps
        if self.bias is None:
            if not self.told_to_stand:
                self.told_to_stand = True
                self.log("Heading hold waiting: stand still for 2 s so it can measure the "
                         "gyro's drift.")
            return steps
        if self.sign is None:
            if not self.told_to_turn:
                self.told_to_turn = True
                self.log("Heading hold waiting: turn once (Q/E or the turn stick) so it can "
                         "learn the gyro direction.")
            return steps
        yaw = self.read()
        if yaw is not None:
            if self.target is None:
                self.target, self.correction = (yaw, self.clock()), 0.0
            else:
                error = self.rotation_since(yaw, self.target)
                if abs(error) > self.runaway:
                    self.enabled = False
                    self.reset()
                    self.log("Heading hold disabled: drift reached {:.0f} deg despite "
                             "corrections (wrong --heading-sign?).".format(error))
                    return steps
                if abs(error) <= self.deadband:
                    self.correction = 0.0
                else:
                    self.correction = max(-self.max_turn, min(
                        self.max_turn, -self.sign * self.gain * error))
        elif self.failed:
            self.correction = 0.0
        steps["yaw"] = int(round(self.correction))
        return steps


class HeadingDriver:
    """Wrap the leg Driver: send heading-corrected commands, but report and record what was
    asked for (a replay then asks again and is corrected live, not twice)."""

    STOPPED = {"x": 0, "y": 0, "yaw": 0}

    def __init__(self, inner, hold):
        self.inner = inner
        self.hold = hold
        self.current = dict(self.STOPPED)
        self.listener = None

    @property
    def args(self):
        return self.inner.args

    def apply(self, steps):
        steps = dict(steps)
        if not any(steps.values()):
            self.stop()
            return
        if steps != self.current:
            self.current = steps
            if self.listener:
                self.listener("walk", dict(steps))
        self.inner.apply(self.hold.adjust(steps))

    def refresh(self):
        """Called every loop tick: keeps a walk that is not re-sent (replay) corrected, and
        measures the gyro's drift while the legs are stopped."""
        if any(self.current.values()):
            self.inner.apply(self.hold.adjust(self.current))
        else:
            self.hold.idle()

    def stop(self):
        was_moving = any(self.current.values())
        self.current = dict(self.STOPPED)
        self.hold.reset()
        self.inner.stop()
        if was_moving and self.listener:
            self.listener("walk", dict(self.STOPPED))
