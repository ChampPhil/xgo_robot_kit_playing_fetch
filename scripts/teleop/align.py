"""Gyro-assisted alignment: get the robot back to where the recording's first frame was taken.

Measured on the robot (2026-10-15 pulse probe): a leg command does nothing for the first
~0.5 s (gait start-up), then turns at roughly the commanded speed in deg/s; the camera spans
~94 deg; after the program's startup reset the gyro tracks turns with no drift. So:

* turns are closed-loop on the gyro: turn until it reports the wanted angle, computed from
  where the box sits in the image (degrees per image width, calibrated once with one short
  turn toward the box);
* walking is in timed segments that include the start-up delay; since a box's apparent
  height is proportional to 1/distance, 1/height falls linearly while walking, so the time
  to the target size is planned from the measured rate of that fall;
* every decision uses a settled, sharp, two-frame observation (the camera shakes when walking).
"""

import json
import math
from datetime import datetime

from heading import angle_diff  # noqa: F401  (also used by callers as align.angle_diff)
from teach import write_json_atomic

CALIBRATION_VERSION = 2


def errors(features, reference):
    """(eu, eh, ev): horizontal offset, relative size error, vertical offset vs the reference."""
    return (features["u"] - reference["u"], features["h"] / reference["h"] - 1.0,
            features["v"] - reference["v"])


def within(error, tol):
    """True when the driven errors (eu, eh) are inside tolerance; ev is checked separately."""
    return abs(error[0]) <= tol["u"] and abs(error[1]) <= tol["h"]


def walk_seconds(h, h_target, rate):
    """Seconds of forward walking (negative: backward) to grow apparent height h to h_target,
    given rate = how fast 1/h falls per second of forward walking."""
    return (1.0 / h - 1.0 / h_target) / rate


def load_calibration(path):
    """The saved calibration, or None if missing, damaged, or from the old aligner."""
    try:
        data = json.loads(path.read_text())
    except (OSError, ValueError):
        return None
    if not isinstance(data, dict) or data.get("version") != CALIBRATION_VERSION:
        return None
    per_u, sign = data.get("deg_per_u"), data.get("gyro_sign")
    if (isinstance(per_u, bool) or not isinstance(per_u, (int, float)) or not math.isfinite(per_u)
            or per_u <= 0 or sign not in (1, -1)):
        return None
    return data


def save_calibration(path, calibration):
    path.parent.mkdir(parents=True, exist_ok=True)
    write_json_atomic(path, dict(calibration, version=CALIBRATION_VERSION,
                                 created=datetime.now().isoformat(timespec="seconds")))


def compatible(calibration, setup):
    """A calibration applies only with the same speeds and the same camera pose."""
    return all(calibration.get(key) == setup.get(key) for key in ("speeds", "kneel", "arm"))


def agree(a, b, du=0.02, dh=0.05):
    return abs(a["u"] - b["u"]) <= du and abs(a["h"] / b["h"] - 1) <= dh


class Observer:
    """Report the box only from steady frames: captured after the legs settled, sharp, and
    two in a row agreeing (the camera shakes heavily while walking)."""

    def __init__(self, camera, measure, sharpness, clock, min_sharpness=50, timeout=2.0):
        self.camera = camera
        self.measure = measure  # frame -> (features | None, reason)
        self.sharpness = sharpness
        self.clock = clock
        self.min_sharpness = min_sharpness
        self.timeout = timeout
        self.begin(clock())

    def begin(self, not_before):
        self.not_before = not_before
        self.deadline = not_before + self.timeout
        self.previous = None
        self.misses = 0
        self.last_stamp = None

    def poll(self):
        """None while waiting; ("ok", features), ("lost", reason) or ("stalled", reason)."""
        now = self.clock()
        frame, stamp = self.camera.latest()
        if frame is None or stamp is None or stamp < self.not_before or stamp == self.last_stamp:
            return ("stalled", "no fresh camera frame") if now > self.deadline else None
        self.last_stamp = stamp
        if self.sharpness(frame) < self.min_sharpness:
            return ("stalled", "camera image stays blurry") if now > self.deadline else None
        features, reason = self.measure(frame)
        if features is None:
            self.previous = None
            self.misses += 1
            return ("lost", reason) if self.misses >= 2 else None
        self.misses = 0
        if self.previous is not None and agree(self.previous, features):
            pair = (self.previous, features)
            return "ok", {key: (pair[0][key] + pair[1][key]) / 2 for key in ("u", "v", "h")}
        self.previous = features
        return None


class GyroAligner:
    """Hold-A / fetch state machine. Ticked by the receiver loop; never blocks.

    legs: step(primitive, amount) starts moving (sign = direction, |amount| = fraction of the
    primitive's speed), stop() stops. read_yaw(): gyro heading in degrees (raises on failure).
    pose: prepare(kneel, arm). reference: () -> {"features": {u, v, h}, "state": {...}} | None.
    """

    DEAD = 0.5  # gait start-up before a command moves anything (probe: 0.5-0.7 s)
    CAL_TURN = 15.0  # degrees for the one-off calibration turn
    MIN_TURN = 3.0  # smaller corrections cannot be executed reliably
    MIN_WALK, MAX_WALK = 0.3, 2.5  # seconds of actual walking per segment
    FINE_AMOUNT = 0.5  # half speed when a full-speed segment would be too short
    TURN_RATE = 0.8  # deg/s per unit of turn speed (probe), only for turn deadlines

    def __init__(self, legs, pose, observer, read_yaw, reference, calibration_path, log, clock,
                 motion=False, recalibrate=False, settle=0.7, speeds=None, tol=None,
                 max_seconds=45.0, max_moves=25, prepare_settle=1.0):
        self.legs = legs
        self.pose = pose
        self.observer = observer
        self.read_yaw = read_yaw
        self.reference = reference
        self.calibration_path = calibration_path
        self.log = log
        self.clock = clock
        self.motion = motion
        self.recalibrate = recalibrate
        self.settle = settle
        self.speeds = speeds or {"turn": 40, "forward": 12}
        self.tol = tol or {"u": 0.04, "h": 0.08, "v": 0.05}
        self.max_seconds = max_seconds
        self.max_moves = max_moves
        self.prepare_settle = prepare_settle
        self.active = False
        self.refused = False
        self.phase = None
        self.moving = None  # None, "turn" or "walk"

    # --- receiver interface -------------------------------------------------------------
    def update(self, held, manual):
        """Call once per message with A held / manual input. True while it owns the robot."""
        if not held:
            if self.active:
                self.abort("A released", quiet=self.phase == "aligned")
            self.refused = False
            return False
        if manual:
            if self.active:
                self.abort("manual input")
            self.refused = True
            return False
        if self.refused:
            return False
        if not self.active and not self.begin():
            self.refused = True
            return False
        self.tick(self.clock())
        return self.active

    def watchdog(self):
        if self.active:
            self.abort("no commands")

    def abort(self, reason, quiet=False):
        self.legs.stop()
        self.moving = None
        if self.active:
            self.log("Alignment ended." if quiet else f"Alignment stopped: {reason}.")
        self.active = False
        self.phase = None
        self.refused = True

    # --- setup ------------------------------------------------------------------------------
    def begin(self):
        reference = self.reference()
        if reference is None:
            self.log("No recording to align to yet; record one with X first.")
            return False
        now = self.clock()
        self.ref = reference["features"]
        state = reference.get("state") or {}
        self.setup = {"speeds": self.speeds, "kneel": state.get("kneel"), "arm": state.get("arm")}
        calibration = None if self.recalibrate else load_calibration(self.calibration_path)
        if calibration is not None and not compatible(calibration, self.setup):
            self.log("Saved calibration was made with other speeds or arm/kneel; recalibrating.")
            calibration = None
        self.calibration = calibration
        self.active, self.started, self.moves = True, now, 0
        self.walk_rate = None  # how fast 1/h falls per second of full-speed forward walking
        self.lead = 1.0  # degrees the robot keeps turning after a stop
        self.prev_norm, self.grow, self.in_tol, self.lost = None, 0, 0, 0
        self.last_move, self.last_display = None, None
        self.phase = "initial"
        if self.motion:
            self.pose.prepare(state.get("kneel", 0.0), state.get("arm"))
            self.log("ALIGN: moving arm/kneel to the recording's pose...")
            self.observer.begin(now + self.prepare_settle)
        else:
            self.log("ALIGN display only (no --align-motion): nothing will move.")
            self.observer.begin(now)
        return True

    # --- motion -----------------------------------------------------------------------------
    def yaw(self):
        try:
            return self.read_yaw()
        except (OSError, TimeoutError, ValueError):
            return None

    def start_turn(self, degrees_left, now):
        """Turn by `degrees_left` (negative = right), stopping on the gyro's say-so."""
        start = self.yaw()
        if start is None:
            self.abort("no gyro reading, so turns cannot be measured")
            return
        direction = 1 if degrees_left > 0 else -1
        target = abs(degrees_left)
        rate = self.TURN_RATE * self.speeds["turn"]
        self.turn = {"start": start, "direction": direction, "target": target, "fails": 0,
                     "deadline": now + self.DEAD + target / rate + 2.0}
        self.legs.step("turn", direction)
        self.moving = "turn"
        self.last_move = ("turn", direction, target)

    def turned(self, yaw):
        """Degrees turned in the commanded direction (absolute until the gyro sign is known)."""
        change = angle_diff(yaw, self.turn["start"])
        if self.calibration is None:
            return abs(change)
        return change * self.calibration["gyro_sign"] * self.turn["direction"]

    def turn_tick(self, now):
        yaw = self.yaw()
        if yaw is None:
            self.turn["fails"] += 1
            if self.turn["fails"] >= 3:
                self.abort("the gyro stopped answering during a turn")
            return
        self.turn["fails"] = 0
        progress = self.turned(yaw)
        if progress >= self.turn["target"] - self.lead or now >= self.turn["deadline"]:
            self.legs.stop()
            self.moving = None
            self.turn["at_stop"] = progress
            self.observer.begin(now + self.settle)

    def start_walk(self, direction, seconds, amount, now):
        self.legs.step("forward", direction * amount)
        self.walk = {"until": now + self.DEAD + seconds, "seconds": seconds,
                     "direction": direction, "amount": amount}
        self.moving = "walk"
        self.last_move = ("walk", direction, seconds)

    def tick(self, now):
        if self.phase == "aligned":
            return
        if self.moving == "turn":
            self.turn_tick(now)
            return
        if self.moving == "walk":
            if now < self.walk["until"]:
                return
            self.legs.stop()
            self.moving = None
            self.observer.begin(now + self.settle)
        result = self.observer.poll()
        if result is None:
            return
        status, value = result
        if status == "stalled":
            self.abort(value)
            return
        getattr(self, "on_" + self.phase)(status, value, now)

    # --- phases -----------------------------------------------------------------------------
    def on_initial(self, status, value, now):
        if not self.motion:
            self.phase = "display"
            self.on_display(status, value, now)
            return
        if status == "lost":
            self.abort(f"box not in view at the start ({value})")
            return
        self.previous = value
        if self.calibration is None:
            # turn toward the box so it stays in view; measure degrees per image width
            self.phase = "calibrate"
            self.cal_u = value["u"]
            self.log("CALIBRATE: one {:.0f} deg turn toward the box...".format(self.CAL_TURN))
            self.start_turn(self.CAL_TURN if value["u"] < 0.5 else -self.CAL_TURN, now)
            return
        self.phase, self.align_started = "align", now
        self.on_align(status, value, now)

    def on_display(self, status, value, now):
        if self.last_display is None or now - self.last_display >= 1.0:
            self.last_display = now
            if status == "lost":
                self.log(f"ALIGN (display only): {value}")
            else:
                eu, eh, ev = errors(value, self.ref)
                self.log("ALIGN (display only) eu={:+.2f} eh={:+.2f} ev={:+.2f} -> {}".format(
                    eu, eh, ev, self.advice(eu, eh, ev)))
        self.observer.begin(now)

    def advice(self, eu, eh, ev):
        if within((eu, eh), self.tol):
            return "in tolerance" + (" (view mismatch)" if abs(ev) > self.tol["v"] else "")
        if abs(eu) > self.tol["u"]:
            side = "right" if eu > 0 else "left"
            if self.calibration:
                return "would turn {} {:.0f} deg".format(
                    side, abs(eu) * self.calibration["deg_per_u"])
            return "would turn " + side
        return "would walk " + ("forward (box looks small)" if eh < 0 else "back (box looks big)")

    def on_calibrate(self, status, value, now):
        if status == "lost":
            self.abort("the box left the view during the calibration turn")
            return
        yaw = self.yaw()
        if yaw is None:
            self.abort("no gyro reading after the calibration turn")
            return
        raw = angle_diff(yaw, self.turn["start"])
        du = value["u"] - self.cal_u
        if abs(raw) < self.MIN_TURN or abs(du) < 0.02:
            self.abort("the calibration turn did not move the box (did the legs start?)")
            return
        gyro_sign = 1 if raw * self.turn["direction"] > 0 else -1
        left_degrees = raw * gyro_sign  # positive when the robot turned left
        deg_per_u = left_degrees / du  # turning left moves the box right
        if deg_per_u <= 0:
            self.abort("the calibration turn moved the box the wrong way")
            return
        self.calibration = dict(self.setup, deg_per_u=round(deg_per_u, 2), gyro_sign=gyro_sign)
        save_calibration(self.calibration_path, self.calibration)
        self.recalibrate = False
        self.log("CALIBRATE done: {:.0f} deg across the image, gyro {}.".format(
            deg_per_u, "counter-clockwise positive" if gyro_sign == 1 else "clockwise positive"))
        self.last_move = None
        self.phase, self.align_started = "align", now
        self.on_align(status, value, now)

    def learn(self, value):
        """Update the turn lead and walking rate from what the last move actually did."""
        if self.last_move is None:
            return
        if self.last_move[0] == "turn" and "at_stop" in self.turn:
            yaw = self.yaw()
            if yaw is not None:
                coast = self.turned(yaw) - self.turn["at_stop"]
                self.lead = max(0.0, min(8.0, 0.7 * self.lead + 0.3 * coast))
        elif self.last_move[0] == "walk":
            walked = self.walk["seconds"] * self.walk["amount"]
            change = (1.0 / self.previous["h"] - 1.0 / value["h"]) * self.walk["direction"]
            rate = change / walked
            if rate > 0:
                self.walk_rate = rate if self.walk_rate is None else 0.5 * (self.walk_rate + rate)

    def on_align(self, status, value, now):
        if now - self.align_started > self.max_seconds:
            self.abort(f"not aligned after {self.max_seconds:.0f} s")
            return
        if status == "lost":
            self.lost += 1
            if self.lost >= 3:
                self.abort(f"lost the box ({value})")
            else:
                self.observer.begin(now)
            return
        self.lost = 0
        self.learn(value)
        self.last_move = None
        self.previous = value
        eu, eh, ev = errors(value, self.ref)
        if within((eu, eh), self.tol):
            if abs(ev) > self.tol["v"]:
                self.abort("view mismatch: the box is the right size but at a different height "
                           "in the image (check kneel, arm, floor and box)")
                return
            self.in_tol += 1
            if self.in_tol >= 2:
                self.legs.stop()
                self.phase = "aligned"
                self.log("ALIGNED (eu={:+.2f} eh={:+.2f}) - release A, then press Y".format(
                    eu, eh))
                return
            self.observer.begin(now)
            return
        self.in_tol = 0
        norm = math.hypot(eu, eh)
        growing = self.prev_norm is not None and norm > self.prev_norm + 0.01
        self.grow = self.grow + 1 if growing else 0
        self.prev_norm = norm
        if self.grow >= 3:
            self.abort("error grew 3 moves in a row; something moves the wrong way "
                       "(try --recalibrate)")
            return
        self.moves += 1
        if self.moves > self.max_moves:
            self.abort(f"not aligned after {self.max_moves} moves")
            return
        if abs(eu) > self.tol["u"]:
            degrees = -eu * self.calibration["deg_per_u"]  # box right of target: turn right
            if abs(degrees) < self.MIN_TURN:
                degrees = math.copysign(self.MIN_TURN, degrees)
            self.log("ALIGN {}: eu={:+.2f} eh={:+.2f} -> turn {} {:.0f} deg".format(
                self.moves, eu, eh, "left" if degrees > 0 else "right", abs(degrees)))
            self.start_turn(degrees, now)
            return
        direction, seconds, amount = self.plan_walk(value["h"], eh)
        self.log("ALIGN {}: eu={:+.2f} eh={:+.2f} -> walk {} {:.1f} s{}".format(
            self.moves, eu, eh, "forward" if direction > 0 else "back", seconds,
            " (half speed)" if amount < 1 else ""))
        self.start_walk(direction, seconds, amount, now)

    def plan_walk(self, h, eh):
        """(direction, seconds of actual walking, speed fraction) for the next segment."""
        if self.walk_rate is None:  # first segment: a modest probe
            return (1 if eh < 0 else -1), 1.0, 1.0
        needed = walk_seconds(h, self.ref["h"], self.walk_rate)
        direction = 1 if needed > 0 else -1
        seconds = abs(needed) * 0.9  # undershoot slightly; the next look corrects
        amount = 1.0
        if seconds < self.MIN_WALK * 2:  # too short at full speed: walk slower instead
            amount = self.FINE_AMOUNT
            seconds /= amount
        return direction, max(self.MIN_WALK, min(self.MAX_WALK, seconds)), amount


def alignment_legs_driver(driver):
    """Alignment steers by the camera; keep heading-hold corrections out of its steps."""
    return getattr(driver, "inner", driver)


# Far away under room lighting the purple box measures S~62, V~40 and hue up to ~175, outside
# the recording's purple range; alignment uses a wider range plus a 40 px minimum, and takes a
# clearly dominant box over small background specks.
ALIGN_PURPLE = ((115, 40, 25), (178, 255, 255))
ALIGN_MIN_AREA = 40
ALIGN_DOMINANT = 2.0


def alignment_measure(color):
    """frame -> (features, reason) with the alignment detector."""
    import teach as teach_module  # puts box_detection on sys.path
    from detect_boxes import HSV_RANGES, detect_boxes

    ranges = dict(HSV_RANGES, purple=ALIGN_PURPLE)

    def measure(frame):
        found = detect_boxes(frame, min_area=ALIGN_MIN_AREA, ranges=ranges)
        return teach_module.box_features(found, color, frame.shape, dominant=ALIGN_DOMINANT)

    return measure


def alignment_reference(load, measure):
    """The latest reference, re-measured from its photo with the alignment detector so sizes
    are compared like for like (falls back to the stored features)."""
    def reference():
        found = load()
        if found is None:
            return None
        try:
            import cv2  # type: ignore[import-not-found]

            image = cv2.imread(str(found.get("image")))
            features, _ = measure(image) if image is not None else (None, None)
        except Exception:
            features = None
        return dict(found, features=features or found["features"])

    return reference
