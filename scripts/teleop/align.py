"""Self-calibrating alignment: walk until the box looks like the recording's reference photo.

The robot measures how each step primitive (turn, forward, strafe) moves the box in the
image, uses that to pick the next step, and keeps refining it as it goes. See
docs/superpowers/specs/2026-10-07-self-calibrating-alignment-design.md.
"""

import json
import math
from datetime import datetime

from teach import write_json_atomic

PRIMITIVES = ("turn", "forward", "strafe")
NOISE = 0.005  # per-step image change below this is treated as "this primitive does nothing"
LEARN_RATE = 0.3
LEARN_MIN_AMOUNT = 0.25


def errors(features, reference):
    """(eu, eh, ev): horizontal offset, relative size error, vertical offset vs the reference."""
    return (features["u"] - reference["u"], features["h"] / reference["h"] - 1.0,
            features["v"] - reference["v"])


def within(error, tol):
    """True when the driven errors (eu, eh) are inside tolerance; ev is checked separately."""
    return abs(error[0]) <= tol["u"] and abs(error[1]) <= tol["h"]


def usable(column):
    return column is not None and math.hypot(column[0], column[1]) >= NOISE


def choose_step(error, columns, min_amount=0.0):
    """Pick (primitive, amount in [-1, 1]) predicted to shrink (eu, eh) the most.

    Steps shorter than `min_amount` of a full step cannot be executed reliably, so they are
    rounded up to it before predicting. Returns (None, 0.0) when no usable primitive helps.
    """
    eu, eh = error[0], error[1]
    best, best_amount, best_residual = None, 0.0, math.hypot(eu, eh)
    for name in PRIMITIVES:
        column = columns.get(name)
        if not usable(column):
            continue
        cu, ch = column
        amount = -(cu * eu + ch * eh) / (cu * cu + ch * ch)
        amount = max(-1.0, min(1.0, amount))
        if 0 < abs(amount) < min_amount:
            amount = math.copysign(min_amount, amount)
        residual = math.hypot(eu + amount * cu, eh + amount * ch)
        if residual < best_residual - 1e-9:
            best, best_amount, best_residual = name, amount, residual
    return best, best_amount


def broyden_update(column, observed, amount, rate=LEARN_RATE):
    """Move a column toward the change actually observed for `amount` of a step."""
    if abs(amount) < LEARN_MIN_AMOUNT:
        return list(column)
    return [c + rate * (o / amount - c) for c, o in zip(column, observed)]


def finite_pair(value):
    return (isinstance(value, list) and len(value) == 2
            and all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)
                    for x in value))


def load_calibration(path):
    """The saved calibration, or None if missing, damaged or the wrong shape."""
    try:
        data = json.loads(path.read_text())
    except (OSError, ValueError):
        return None
    if not isinstance(data, dict) or data.get("version") != 1:
        return None
    columns = data.get("columns")
    if not isinstance(columns, dict):
        return None
    for name in PRIMITIVES:
        if columns.get(name) is not None and not finite_pair(columns[name]):
            return None
    return data


def save_calibration(path, calibration):
    path.parent.mkdir(parents=True, exist_ok=True)
    write_json_atomic(path, dict(calibration, version=1,
                                 created=datetime.now().isoformat(timespec="seconds")))


def compatible(calibration, setup):
    """A saved calibration only applies with the same step speeds/time and camera pose."""
    return all(calibration.get(key) == setup.get(key)
               for key in ("speeds", "step_seconds", "kneel", "arm"))


def agree(a, b, du=0.02, dh=0.05):
    return abs(a["u"] - b["u"]) <= du and abs(a["h"] / b["h"] - 1) <= dh


def change(before, after):
    """Image change between two observations, in model units (du, relative dh)."""
    return [after["u"] - before["u"], after["h"] / before["h"] - 1.0]


def mean(rows):
    return [sum(values) / len(values) for values in zip(*rows)]


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


def describe(primitive, amount):
    words = {"turn": ("turn left", "turn right"), "forward": ("walk forward", "walk back"),
             "strafe": ("step left", "step right")}[primitive]
    return "{} {:.2f}".format(words[0] if amount > 0 else words[1], abs(amount))


class Aligner:
    """Hold-A state machine: prepare the camera pose, calibrate once (360° turn + probes),
    then step until the box matches the reference. Ticked by the receiver loop; never blocks.

    legs: step(primitive, amount) starts moving (sign = direction), stop() stops.
    pose: prepare(kneel, arm) puts the kneel level and arm where the reference was taken.
    reference: () -> {"features": {u, v, h}, "state": {"kneel", "arm"}} or None.
    """

    MIN_STEP_SECONDS = 0.12
    TURN_STEP_LIMIT = 90
    CALIBRATION_SECONDS = 120.0

    def __init__(self, legs, pose, observer, reference, calibration_path, log, clock,
                 motion=False, recalibrate=False, settle=0.7, step_seconds=0.3, speeds=None,
                 tol=None, max_steps=40, max_seconds=60.0, prepare_settle=1.0):
        self.legs = legs
        self.pose = pose
        self.observer = observer
        self.reference = reference
        self.calibration_path = calibration_path
        self.log = log
        self.clock = clock
        self.motion = motion
        self.recalibrate = recalibrate
        self.settle = settle
        self.step_seconds = step_seconds
        self.speeds = speeds or {"turn": 20, "forward": 8, "strafe": 6}
        self.tol = tol or {"u": 0.04, "h": 0.08, "v": 0.05}
        self.max_steps = max_steps
        self.max_seconds = max_seconds
        self.prepare_settle = prepare_settle
        self.min_amount = self.MIN_STEP_SECONDS / step_seconds
        self.active = False
        self.refused = False  # stay idle until A is released after a stop/refusal
        self.phase = None
        self.moving_until = None

    # --- receiver interface -------------------------------------------------------------
    def update(self, held, manual):
        """Call once per message with A held / manual input. True while aligning owns the
        legs, arm and kneel."""
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
        self.moving_until = None
        if self.active:
            self.log("Alignment ended." if quiet else f"Alignment stopped: {reason}.")
        self.active = False
        self.phase = None
        self.refused = True

    # --- internals ------------------------------------------------------------------------
    def begin(self):
        reference = self.reference()
        if reference is None:
            self.log("No recording to align to yet; record one with X first.")
            return False
        now = self.clock()
        self.ref = reference["features"]
        state = reference.get("state") or {}
        self.setup = {"speeds": self.speeds, "step_seconds": self.step_seconds,
                      "kneel": state.get("kneel"), "arm": state.get("arm")}
        calibration = None if self.recalibrate else load_calibration(self.calibration_path)
        if calibration is not None and not compatible(calibration, self.setup):
            self.log("Saved calibration was made with other speeds or arm/kneel; recalibrating.")
            calibration = None
        self.columns = dict(calibration["columns"]) if calibration else None
        self.active, self.steps, self.started = True, 0, now
        self.prev_e = self.prev_norm = self.last_step = None
        self.grow = self.in_tol = self.lost = 0
        self.last_display = None
        self.phase = "initial"
        if self.motion:
            self.pose.prepare(state.get("kneel", 0.0), state.get("arm"))
            self.log("ALIGN: moving arm/kneel to the recording's pose...")
            self.observer.begin(now + self.prepare_settle)
        else:
            self.log("ALIGN display only (no --align-motion): nothing will move.")
            self.observer.begin(now)
        return True

    def move(self, primitive, amount, now):
        self.legs.step(primitive, amount)
        self.moving_until = now + max(self.MIN_STEP_SECONDS, abs(amount) * self.step_seconds)
        self.move_started = now
        self.last_step = (primitive, amount)

    def tick(self, now):
        if self.phase == "aligned":
            return
        if self.moving_until is not None:
            if now < self.moving_until:
                return
            self.legs.stop()
            primitive, amount = self.last_step
            actual = math.copysign((now - self.move_started) / self.step_seconds, amount)
            self.last_step = (primitive, actual)  # learn from what was actually executed
            self.moving_until = None
            self.observer.begin(now + self.settle)
        result = self.observer.poll()
        if result is None:
            return
        status, value = result
        if status == "stalled":
            self.abort(value)
            return
        getattr(self, "on_" + self.phase)(status, value, now)

    def on_initial(self, status, value, now):
        if not self.motion:
            self.phase = "display"
            self.on_display(status, value, now)
            return
        if status == "lost":
            self.abort(f"box not in view at the start ({value})")
            return
        if self.columns is None:
            self.cal = {"u0": value["u"], "prev": value, "samples": [], "steps": 0,
                        "lost": False, "found": False, "started": now}
            self.phase = "turn360"
            self.log("CALIBRATE: full 360° turn in small steps; keep the area around the "
                     "robot clear.")
            self.move("turn", 1.0, now)
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
                if within((eu, eh), self.tol):
                    advice = "in tolerance" + (" (view mismatch)" if abs(ev) > self.tol["v"] else "")
                elif self.columns:
                    primitive, amount = choose_step((eu, eh), self.columns, self.min_amount)
                    advice = "would " + describe(primitive, amount) if primitive else "no step helps"
                elif abs(eu) > self.tol["u"]:
                    advice = "box is " + ("right" if eu > 0 else "left") + " of the target"
                else:
                    advice = "box looks " + ("small (too far)" if eh < 0 else "large (too close)")
                self.log(f"ALIGN (display only) eu={eu:+.2f} eh={eh:+.2f} ev={ev:+.2f} -> {advice}")
        self.observer.begin(now)

    def on_turn360(self, status, value, now):
        cal = self.cal
        cal["steps"] += 1
        if cal["steps"] % 5 == 0:
            self.log(f"CALIBRATE turn {cal['steps']}/~30")
        if status == "ok":
            if not cal["lost"]:
                cal["samples"].append(change(cal["prev"], value))
                cal["prev"] = value
            else:
                cal["found"] = True
                if not cal["samples"]:
                    self.abort("the box left the view on the first turn step; lower --align-turn")
                    return
                column = mean(cal["samples"])
                if (value["u"] - cal["u0"]) * column[0] >= 0:  # back where it started
                    steps = cal["steps"] - (value["u"] - cal["u0"]) / column[0]
                    cal["deg"] = 360.0 / steps
                    cal["turn"] = column
                    self.phase = "probe"
                    self.probe = {"plan": [("forward", 1.0), ("forward", -1.0), ("strafe", 1.0),
                                           ("strafe", -1.0)], "seen": [value]}
                    self.log("CALIBRATE: 360° done ({:.1f}° per turn step); probing walk and "
                             "side-step...".format(cal["deg"]))
                    self.move(*self.probe["plan"].pop(0), now)
                    return
        else:
            if cal["found"]:
                self.abort("the box was lost again after coming back (another object in view?)")
                return
            cal["lost"] = True
        if cal["steps"] >= self.TURN_STEP_LIMIT or now - cal["started"] > self.CALIBRATION_SECONDS:
            self.abort("box not seen again during the 360° turn")
            return
        self.move("turn", 1.0, now)

    def on_probe(self, status, value, now):
        if status == "lost":
            self.abort(f"lost the box while probing ({value})")
            return
        seen = self.probe["seen"]
        seen.append(value)
        if self.probe["plan"]:
            self.move(*self.probe["plan"].pop(0), now)
            return
        forward = mean([change(seen[0], seen[1]), [-x for x in change(seen[1], seen[2])]])
        strafe = mean([change(seen[2], seen[3]), [-x for x in change(seen[3], seen[4])]])
        columns = {"turn": self.cal["turn"], "forward": forward, "strafe": strafe}
        columns = {name: (column if usable(column) else None) for name, column in columns.items()}
        save_calibration(self.calibration_path, dict(self.setup, columns=columns,
                                                     deg_per_turn_step=self.cal["deg"]))
        self.columns = columns
        self.recalibrate = False
        self.log("CALIBRATE done: {:.1f}°/turn step, turn du={}, forward dh={}, strafe du={}".format(
            self.cal["deg"], *(("{:+.3f}".format(columns[n][i]) if columns[n] else "unusable")
                               for n, i in (("turn", 0), ("forward", 1), ("strafe", 0)))))
        self.phase, self.align_started = "align", now
        self.last_step = None
        self.on_align(status, value, now)

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
        eu, eh, ev = errors(value, self.ref)
        if self.last_step is not None and self.prev_e is not None:
            primitive, amount = self.last_step
            observed = (eu - self.prev_e[0], (1 + eh) / (1 + self.prev_e[1]) - 1)
            if self.columns.get(primitive) is not None:
                self.columns[primitive] = broyden_update(self.columns[primitive], observed, amount)
        self.last_step = None
        self.prev_e = (eu, eh)
        if within((eu, eh), self.tol):
            if abs(ev) > self.tol["v"]:
                self.abort("view mismatch: the box is the right size but at a different height "
                           "in the image (check kneel, arm, floor and box)")
                return
            self.in_tol += 1
            if self.in_tol >= 3:
                self.legs.stop()
                self.phase = "aligned"
                self.log("ALIGNED (eu={:+.2f} eh={:+.2f}) - release A, then press Y".format(eu, eh))
                return
            self.observer.begin(now)
            return
        self.in_tol = 0
        norm = math.hypot(eu, eh)
        self.grow = self.grow + 1 if self.prev_norm is not None and norm > self.prev_norm + 1e-3 else 0
        self.prev_norm = norm
        if self.grow >= 3:
            self.abort("error grew 3 steps in a row; the motion model is wrong (try --recalibrate)")
            return
        primitive, amount = choose_step((eu, eh), self.columns, self.min_amount)
        if primitive is None:
            self.abort("no step is predicted to help (try --recalibrate)")
            return
        self.steps += 1
        if self.steps > self.max_steps:
            self.abort(f"not aligned after {self.max_steps} steps")
            return
        self.log("ALIGN {}: eu={:+.2f} eh={:+.2f} -> {}".format(
            self.steps, eu, eh, describe(primitive, amount)))
        self.move(primitive, amount, now)
