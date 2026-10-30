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
# Used in the successful 2026-10-26 pickup; repeatability across positions is unmeasured.
DEFAULT_TOL = {"u": 0.02, "h": 0.04, "v": 0.025}


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
        if now > self.deadline:  # e.g. the box flickers in and out and never settles
            return "stalled", "no steady view of the box"
        if features is None:
            self.previous = None
            self.misses += 1
            return ("lost", reason) if self.misses >= 2 else None
        self.misses = 0
        if self.previous is not None and agree(self.previous, features):
            pair = (self.previous, features)
            averaged = {key: (pair[0][key] + pair[1][key]) / 2 for key in ("u", "v", "h")}
            averaged["samples"] = pair  # final acceptance checks each frame, not just its mean
            return "ok", averaged
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
    MIN_TURN = 3.0  # minimum useful motion for validating calibration
    MIN_CORRECTION = 1.0  # quarter-speed request; not an independently calibrated turn resolution
    MAX_CORRECTION = 12.0  # re-observe rather than risk a large turn from a poor visual scale
    MIN_WALK, MAX_WALK = 0.15, 2.5  # actual walking, in addition to learned gait start-up
    FINE_AMOUNT = 0.5  # half speed when a full-speed segment would be too short
    # Small turns at half speed: at full speed a turn keeps going ~5 deg after the stop, so
    # 5-6 deg corrections turned 10-11 deg and swung back and forth (2026-10-23).
    FINE_TURN = 12.0
    PRECISE_TURN = 6.0
    PRECISE_AMOUNT = 0.25
    PRECISE_WALK_ERROR = 0.15
    # Close to the box, centre with quarter-speed side steps, not turns: measured 2026-10-30,
    # small turns shuffle the robot back ~5 mm (right turns barely turn) while a side step
    # moves the box ~0.02 of the image width per 0.2 s of stepping with little backing up.
    SIDE_ZONE = {"u": 0.08, "h": 0.15}
    SIDE_RATE = 0.1  # eu per second of quarter-speed side stepping (start; then learned)
    MIN_SIDE, MAX_SIDE = 0.15, 0.6
    CONFIRM_LOOKS = 3
    CONFIRM_GAP = 0.25  # fresh, stopped looks spread out before handing over to replay
    TURN_RATE = 0.8  # deg/s per unit of turn speed (probe), only for turn deadlines
    DRIFT_WAIT = 6.0  # seconds to wait for heading hold's drift estimate before giving up
    TOO_BIG = 2.5  # a "box" this many times the reference height is something else
    MAX_WALK_DEAD = 2.5  # seconds; a walk that has not started by then is a fault
    NO_MOTION = 0.03  # a walk that changed the box height by less than this did not start
    MAX_BACKUPS = 3  # times the box may be lost to an overshoot before giving up
    KNEEL_LEVELS = (0.0, 0.5, 1.0)  # postures tried when the box is not in view
    MAX_SEARCH_MOVES = 8
    SCAN_TURNS, SCAN_DEGREES = 7, 45.0  # then look around: 8 directions in all
    SEARCH_WALK = 1.5  # seconds toward a box only another posture can see
    SEARCH_CENTER = 0.1  # turn toward it first when it is this far off the target column

    def __init__(self, legs, pose, observer, read_yaw, reference, calibration_path, log, clock,
                 motion=False, recalibrate=False, settle=0.7, speeds=None, tol=None,
                 max_seconds=45.0, max_moves=25, prepare_settle=1.0, drift=None,
                 gyro_sign_learned=None, target="box"):
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
        self.tol = dict(DEFAULT_TOL if tol is None else tol)
        self.max_seconds = max_seconds
        self.max_moves = max_moves
        self.prepare_settle = prepare_settle
        self.drift = drift or (lambda: 0.0)  # gyro drift in deg/s (None until measured)
        self.gyro_sign_learned = gyro_sign_learned  # tell heading hold the checked direction
        self.target = target  # what is aligned to, for messages ("box", "marker")
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
        self.lead = {1.0: 1.0, self.FINE_AMOUNT: 1.0, self.PRECISE_AMOUNT: 0.5}
        self.turn_gain = 1.0  # reduced when a settled correction crosses the target column
        self.precision_reversals = 0
        self.prev_norm, self.grow, self.in_tol, self.lost = None, 0, 0, 0
        self.last_move, self.last_display = None, None
        self.drift_wait = None
        # Walking from a kneel starts later than turning (a 1.0 s walk did not move at all,
        # 2026-10-21): learned upwards when a walk leaves the box's size unchanged.
        self.walk_dead = self.DEAD
        self.side_rate = self.SIDE_RATE
        self.backups, self.max_forward = 0, None
        self.search_levels, self.search_moves, self.scans = None, 0, 0
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
        amount = self.FINE_AMOUNT if target < self.FINE_TURN and self.calibration else 1.0
        if target < self.PRECISE_TURN and self.calibration:
            amount = self.PRECISE_AMOUNT
        rate = self.TURN_RATE * self.speeds["turn"] * amount
        self.turn = {"start": start, "direction": direction, "target": target, "fails": 0,
                     "began": now, "bias": self.drift() or 0.0, "amount": amount,
                     "deadline": now + self.DEAD + target / rate + 2.0}
        self.legs.step("turn", direction * amount)
        self.moving = "turn"
        self.last_move = ("turn", direction, target)

    def turned(self, yaw):
        """Degrees turned in the commanded direction (absolute until the gyro sign is known),
        with the gyro's own drift since the turn began subtracted."""
        change = self.raw_turn(yaw)
        if self.calibration is None:
            return abs(change)
        return change * self.calibration["gyro_sign"] * self.turn["direction"]

    def raw_turn(self, yaw):
        """Gyro change since the turn began, minus drift, in the gyro's own direction."""
        elapsed = self.clock() - self.turn["began"]
        return angle_diff(yaw, self.turn["start"]) - self.turn["bias"] * elapsed

    def turn_tick(self, now):
        yaw = self.yaw()
        if yaw is None:
            self.turn["fails"] += 1
            if self.turn["fails"] >= 3:
                self.abort("the gyro stopped answering during a turn")
            return
        self.turn["fails"] = 0
        if now - self.turn["began"] < self.DEAD and now < self.turn["deadline"]:
            return  # the legs have not started yet: never stop a turn before it begins
        progress = self.turned(yaw)
        lead = min(self.lead[self.turn["amount"]], 0.8 * self.turn["target"])
        if progress >= self.turn["target"] - lead or now >= self.turn["deadline"]:
            self.legs.stop()
            self.moving = None
            self.turn["at_stop"] = progress
            self.observer.begin(now + self.settle)

    def start_walk(self, direction, seconds, amount, now, primitive="forward"):
        """primitive "strafe": sideways, direction +1 = left."""
        self.legs.step(primitive, direction * amount)
        self.walk = {"until": now + self.walk_dead + seconds, "seconds": seconds,
                     "direction": direction, "amount": amount}
        self.moving = "walk"
        self.last_move = ("walk" if primitive == "forward" else primitive, direction, seconds)

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
        if self.phase == "search_moved":
            self.return_to_pose(now)
            return
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
            if value == TOO_CLOSE:
                self.back_up(1.0, 1.0, now)
            else:
                self.search(now)
            return
        reason = self.implausible(value)
        if reason:
            self.abort(reason)
            return
        if self.drift() is None:  # turns are measured with the gyro, minus its drift
            if self.drift_wait is None:
                self.drift_wait = now
                self.log("ALIGN: measuring the gyro drift first; keep the robot still...")
            if now - self.drift_wait > self.DRIFT_WAIT:
                self.abort("gyro drift not measured (heading hold needs ~2.5 s standing still)")
                return
            self.observer.begin(now + 0.5)
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

    # --- looking from other postures ----------------------------------------------------------
    def search(self, now):
        """Not in view in the recording's posture: kneeling the camera looks down at the near
        floor, standing it sees farther. Try the other kneel levels, nearest first."""
        kneel = self.setup["kneel"] or 0.0
        if self.search_levels is None:
            self.search_levels = sorted((level for level in self.KNEEL_LEVELS
                                         if abs(level - kneel) > 1e-3),
                                        key=lambda level: abs(level - kneel))
        if not self.search_levels:
            if self.scans >= self.SCAN_TURNS:
                self.abort("{} not in view in any posture, looking all around".format(
                    self.target))
                return
            self.scans += 1
            self.log("SEARCH: no {} in any posture; looking around: turn left {:.0f} deg "
                     "({}/{})".format(self.target, self.SCAN_DEGREES, self.scans,
                                       self.SCAN_TURNS))
            self.pose.prepare(self.setup["kneel"] or 0.0, self.setup["arm"])
            self.phase = "search_moved"  # back to the recording's posture after the turn
            self.start_turn(self.SCAN_DEGREES, now)
            self.last_move = None
            return
        level = self.search_levels.pop(0)
        self.log("SEARCH: no {} in view; looking from kneel {:.1f}...".format(self.target,
                                                                              level))
        self.pose.prepare(level, self.setup["arm"])
        self.phase = "search"
        self.observer.begin(now + self.prepare_settle)

    def on_search(self, status, value, now):
        if status == "lost" and value == TOO_CLOSE:
            self.return_to_pose(now)  # close: the recording's posture handles it
            return
        if status == "lost" or self.implausible(value):
            self.search(now)
            return
        self.search_moves += 1
        if self.search_moves > self.MAX_SEARCH_MOVES:
            self.abort("box only seen from another posture after {} moves".format(
                self.MAX_SEARCH_MOVES))
            return
        eu = value["u"] - self.ref["u"]
        self.phase = "search_moved"
        if abs(eu) > self.SEARCH_CENTER:
            deg_per_u = self.calibration["deg_per_u"] if self.calibration else 95.0
            degrees = -eu * deg_per_u
            self.log("SEARCH: box seen (u={:.2f}); turn {} {:.0f} deg".format(
                value["u"], "left" if degrees > 0 else "right", abs(degrees)))
            self.start_turn(degrees, now)
        else:
            self.log("SEARCH: box seen ahead; walk forward {:.1f} s".format(self.SEARCH_WALK))
            self.start_walk(1, self.SEARCH_WALK, 1.0, now)
        self.last_move = None  # not comparable with the recording's posture: nothing to learn

    def return_to_pose(self, now):
        self.pose.prepare(self.setup["kneel"] or 0.0, self.setup["arm"])
        self.search_levels = None
        self.phase = "initial"
        self.observer.begin(now + self.prepare_settle)

    def implausible(self, value):
        if value["h"] > self.TOO_BIG * self.ref["h"]:
            return ("the 'box' looks far too big ({:.0%} of the image height): another purple "
                    "object?".format(value["h"]))
        return None

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
        raw = self.raw_turn(yaw)
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
        try:
            save_calibration(self.calibration_path, self.calibration)
        except OSError as exc:  # e.g. a full disk: keep going with it for this session
            self.log(f"Could not save the calibration ({exc}); using it for this session only.")
        if self.gyro_sign_learned:
            self.gyro_sign_learned(gyro_sign)
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
        if self.last_move[0] != "turn":
            self.precision_reversals = 0
        if self.last_move[0] == "turn":
            before = self.previous["u"] - self.ref["u"]
            after = value["u"] - self.ref["u"]
            if before * after < 0 and abs(after) > self.tol["u"]:
                self.turn_gain = max(0.2, self.turn_gain * 0.5)
                self.precision_reversals = (self.precision_reversals + 1
                                            if self.last_move[2] <= self.MIN_CORRECTION else 0)
                if self.precision_reversals >= 2:
                    self.abort("minimum turn still overshoots the alignment tolerance; "
                               "finer motion needs calibration")
                    return False
                self.log("ALIGN: turn crossed the target; correction gain {:.2f}.".format(
                    self.turn_gain))
            else:
                self.precision_reversals = 0
        if self.last_move[0] == "strafe":  # stepping left moves the box right in the image
            moved = (value["u"] - self.previous["u"]) * self.walk["direction"]
            if moved > 0.003:
                rate = moved / self.walk["seconds"]
                self.side_rate = 0.5 * (self.side_rate + rate)
        if self.last_move[0] == "turn" and self.turn.get("at_stop", 0.0) >= 1.0:
            yaw = self.yaw()  # learn the coast only from turns that really moved
            if yaw is not None:
                coast = self.turned(yaw) - self.turn["at_stop"]
                amount = self.turn["amount"]
                self.lead[amount] = max(0.0, min(8.0, 0.5 * self.lead[amount] + 0.5 * coast))
        elif self.last_move[0] == "walk":
            walked = self.walk["seconds"] * self.walk["amount"]
            change = (1.0 / self.previous["h"] - 1.0 / value["h"]) * self.walk["direction"]
            expected = (self.walk_rate or 0.0) * walked
            relative = abs(value["h"] - self.previous["h"]) / self.previous["h"]
            # quarter-speed final steps change the size ~4-5%, too close to camera noise to
            # tell a step that did not start
            started = self.walk["amount"] < self.FINE_AMOUNT or (
                relative >= self.NO_MOTION and change >= 0.25 * expected)
            if not started:
                return self.walk_did_not_start()
            rate = change / walked
            if rate > 0 and self.walk["amount"] >= self.FINE_AMOUNT:
                self.walk_rate = rate if self.walk_rate is None else 0.5 * (self.walk_rate + rate)
        return True

    def walk_did_not_start(self):
        """The legs were stopped before the gait got going: wait longer from now on. Too long
        a wait only undershoots (the next look corrects); too short overshoots."""
        if self.walk_dead >= self.MAX_WALK_DEAD:
            self.abort("walking did not move the box (do the legs walk?)")
            return False
        commanded = self.walk_dead + self.walk["seconds"]  # the start-up is at least this
        self.walk_dead = min(self.MAX_WALK_DEAD, commanded + 0.25)
        self.log("ALIGN: that walk did not start; allowing {:.1f} s of start-up now.".format(
            self.walk_dead))
        return True

    def overshot(self, reason, now):
        """Cut off at the bottom, or lost right after walking forward: the box is under the
        camera. Back up (as far as that walk went)."""
        forward = self.last_move is not None and self.last_move[:2] == ("walk", 1)
        if not forward and reason != TOO_CLOSE:
            return False
        if self.backups >= self.MAX_BACKUPS:
            return False
        seconds, amount = (self.walk["seconds"], self.walk["amount"]) if forward else (1.0, 1.0)
        self.max_forward = 0.5 * seconds * amount  # creep in from here
        self.back_up(seconds, amount, now)
        return True

    def back_up(self, seconds, amount, now):
        if self.backups >= self.MAX_BACKUPS:
            self.abort("still too close after backing up {} times".format(self.backups))
            return
        self.backups += 1
        self.log("ALIGN: box too close (under the camera) - backing up {:.1f} s.".format(seconds))
        self.lost = 0
        self.start_walk(-1, seconds, amount, now)
        self.last_move = ("backup", -1, seconds)

    def on_align(self, status, value, now):
        if now - self.align_started > self.max_seconds:
            self.abort(f"not aligned after {self.max_seconds:.0f} s")
            return
        if status == "lost":
            self.in_tol = 0
            if self.overshot(value, now):
                return
            self.lost += 1
            if self.lost >= 3:
                self.abort(f"lost the box ({value})")
            else:
                self.observer.begin(now)
            return
        reason = self.implausible(value)
        if reason:
            self.in_tol = 0
            status, value = "lost", reason
            self.lost += 1
            if self.lost >= 3:
                self.abort(reason)
            else:
                self.observer.begin(now)
            return
        self.lost = 0
        backed_up = self.last_move is not None and self.last_move[0] == "backup"
        if not backed_up and self.learn(value) is False:  # a back-up undoes: nothing to learn
            return
        self.last_move = None
        self.previous = value
        eu, eh, ev = errors(value, self.ref)
        if within((eu, eh), self.tol):
            if abs(ev) > self.tol["v"]:
                self.abort("view mismatch: the box is the right size but at a different height "
                           "in the image (check kneel, arm, floor and box)")
                return
            samples = value.get("samples", (value,))
            precise = all(within(errors(sample, self.ref), self.tol)
                          and abs(sample["v"] - self.ref["v"]) <= self.tol["v"]
                          for sample in samples)
            self.in_tol = self.in_tol + 1 if precise else 0
            if self.in_tol >= self.CONFIRM_LOOKS:
                self.legs.stop()
                self.phase = "aligned"
                self.log("ALIGNED (eu={:+.3f} eh={:+.3f} ev={:+.3f}; {} settled looks) "
                         "- release A, then press Y".format(eu, eh, ev, self.in_tol))
                return
            self.observer.begin(now + self.CONFIRM_GAP)
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
        if (abs(eu) > self.tol["u"] and abs(eu) <= self.SIDE_ZONE["u"]
                and abs(eh) < self.SIDE_ZONE["h"] and "strafe" in self.speeds):
            direction = -1 if eu > 0 else 1  # box right of target: step right
            seconds = max(self.MIN_SIDE, min(self.MAX_SIDE, 0.8 * abs(eu) / self.side_rate))
            self.log("ALIGN {}: eu={:+.3f} eh={:+.3f} -> step {} {:.2f} s (quarter speed)"
                     .format(self.moves, eu, eh, "left" if direction > 0 else "right",
                             seconds))
            self.start_walk(direction, seconds, self.PRECISE_AMOUNT, now, primitive="strafe")
            return
        if abs(eu) > self.tol["u"]:
            degrees = -eu * self.calibration["deg_per_u"] * self.turn_gain
            degrees = max(-self.MAX_CORRECTION, min(self.MAX_CORRECTION, degrees))
            if abs(degrees) < self.MIN_CORRECTION:
                degrees = math.copysign(self.MIN_CORRECTION, degrees)
            self.log("ALIGN {}: eu={:+.3f} eh={:+.3f} -> turn {} {:.1f} deg".format(
                self.moves, eu, eh, "left" if degrees > 0 else "right", abs(degrees)))
            self.start_turn(degrees, now)
            return
        direction, seconds, amount = self.plan_walk(value["h"], eh)
        speed_label = {1.0: "", self.FINE_AMOUNT: " (half speed)",
                       self.PRECISE_AMOUNT: " (quarter speed)"}[amount]
        self.log("ALIGN {}: eu={:+.3f} eh={:+.3f} -> walk {} {:.2f} s{}".format(
            self.moves, eu, eh, "forward" if direction > 0 else "back", seconds,
            speed_label))
        self.start_walk(direction, seconds, amount, now)

    def plan_walk(self, h, eh):
        """(direction, seconds of actual walking, speed fraction) for the next segment."""
        if self.walk_rate is None and abs(eh) < self.PRECISE_WALK_ERROR:
            # already close: a full-speed probe would overshoot (eh -0.12 -> +1.3 in the sim)
            return (1 if eh < 0 else -1), 0.4, self.PRECISE_AMOUNT
        if self.walk_rate is None:  # first segment: a modest probe
            seconds = 1.0 if self.max_forward is None or eh > 0 else min(1.0, self.max_forward)
            amount = 1.0 if seconds >= self.MIN_WALK * 2 else self.FINE_AMOUNT
            return (1 if eh < 0 else -1), max(self.MIN_WALK, seconds / amount), amount
        needed = walk_seconds(h, self.ref["h"], self.walk_rate)
        direction = 1 if needed > 0 else -1
        seconds = abs(needed) * 0.9  # undershoot slightly; the next look corrects
        if direction > 0 and self.max_forward is not None:
            seconds = min(seconds, self.max_forward)
        amount = (self.PRECISE_AMOUNT if abs(eh) < self.PRECISE_WALK_ERROR else
                  self.FINE_AMOUNT if seconds < self.MIN_WALK * 2 else 1.0)
        seconds /= amount
        return direction, max(self.MIN_WALK, min(self.MAX_WALK, seconds)), amount


def alignment_legs_driver(driver):
    """Alignment steers by the camera; keep heading-hold corrections out of its steps."""
    return getattr(driver, "inner", driver)


# Far away under room lighting the purple box measures S~62, V~40 and hue up to ~175, outside
# the recording's purple range; alignment uses a wider range plus a 40 px minimum, and takes a
# clearly dominant box over small background specks.
# Hue of the purple box varies with the light: 103-118 in brighter room light (2026-10-21),
# >= 115 in the dim recording, up to ~175 far away. 105 overlaps light-blue (85-110), fine for
# a single-box fetch; 170 stops short of red (170-179 and 0-10).
# Bright floor reads H106 S48 V135 (2026-10-23) and passed as a "too close" box; the far box
# in dim light reads S55-62 V~40. So pale purple counts only when it is also dark.
ALIGN_PURPLE = [((105, 60, 25), (170, 255, 255)), ((105, 40, 25), (170, 255, 90))]
# Kneeling just too close, the box is a wide strip cut off by the bottom edge, which the
# detector's shape filter drops (125x45 px measured 2026-10-21): report it as too close.
TOO_CLOSE = "box cut off at the bottom of the view (too close)"
ALIGN_CLIPPED_AREA = 800
ALIGN_MIN_AREA = 40
ALIGN_DOMINANT = 2.0
# The box sits on the floor, so it is seen below the horizon (mid-image or lower with the
# camera level or tilted down by the kneel); blobs ending above this fraction of the height
# are furniture (a blue chair part read as purple, 2026-10-21).
ALIGN_FLOOR_FROM = 0.4


def alignment_measure(color):
    """frame -> (features, reason) with the alignment detector."""
    import teach as teach_module  # puts box_detection on sys.path
    from detect_boxes import HSV_RANGES, detect_boxes

    ranges = dict(HSV_RANGES, purple=ALIGN_PURPLE)

    def measure(frame):
        found = detect_boxes(frame, min_area=ALIGN_MIN_AREA, ranges=ranges)
        floor = ALIGN_FLOOR_FROM * frame.shape[0]
        found[color] = [c for c in found.get(color, []) if c["bbox"][1] + c["bbox"][3] > floor]
        features, reason = teach_module.box_features(found, color, frame.shape,
                                                     dominant=ALIGN_DOMINANT)
        if features is None and cut_off_at_bottom(frame, ranges[color]):
            return None, TOO_CLOSE
        return features, reason

    return measure


def cut_off_at_bottom(frame, hsv_range):
    """A large blob of the colour touching the bottom edge of the image."""
    import cv2  # type: ignore[import-not-found]
    from detect_boxes import color_mask

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = color_mask(hsv, hsv_range)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    height = frame.shape[0]
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        if y + h >= height - 2 and cv2.contourArea(contour) >= ALIGN_CLIPPED_AREA:
            return True
    return False


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
