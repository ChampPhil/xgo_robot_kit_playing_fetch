"""Offline checks for self-calibrating alignment; uses a simulated robot, no hardware."""

import json
import tempfile
import unittest
from pathlib import Path

import align

REF = {"u": 0.5, "v": 0.6, "h": 0.2}


def feat(u=0.5, v=0.6, h=0.2):
    return {"u": u, "v": v, "h": h}


class MathTests(unittest.TestCase):
    def test_errors_relative_to_reference(self):
        eu, eh, ev = align.errors(feat(u=0.56, v=0.58, h=0.18), REF)
        self.assertAlmostEqual(eu, 0.06)
        self.assertAlmostEqual(eh, -0.1)
        self.assertAlmostEqual(ev, -0.02)

    def test_choose_step_picks_the_best_lever_and_clips(self):
        columns = {"turn": [-0.08, 0.0], "forward": [0.0, 0.06], "strafe": [0.03, 0.0]}
        # box right of target (eu>0): turning right (negative turn) fixes it best
        primitive, amount = align.choose_step((0.04, 0.0), columns)
        self.assertEqual(primitive, "turn")
        self.assertAlmostEqual(amount, 0.5)
        # box too small (far): walk forward, clipped to one full step
        primitive, amount = align.choose_step((0.0, -0.2), columns)
        self.assertEqual((primitive, amount), ("forward", 1.0))

    def test_minimum_step_makes_fine_corrections_use_the_finer_lever(self):
        columns = {"turn": [0.2, 0.0], "strafe": [0.03, 0.0], "forward": [0.0, 0.06]}
        # a quarter turn step would be perfect, but the shortest step is 0.4: turning would
        # overshoot to eu=-0.03, a full strafe lands at eu=+0.02
        self.assertEqual(align.choose_step((0.05, 0.0), columns, min_amount=0.4),
                         ("strafe", -1.0))
        self.assertEqual(align.choose_step((0.05, 0.0), columns)[0], "turn")

    def test_unusable_columns_are_never_chosen(self):
        columns = {"turn": [-0.08, 0.0], "strafe": [0.001, 0.0], "forward": None}
        self.assertEqual(align.choose_step((0.0, -0.2), columns), (None, 0.0))
        self.assertEqual(align.choose_step((0.04, 0.0), columns)[0], "turn")

    def test_broyden_update_moves_the_column_toward_what_happened(self):
        column = align.broyden_update([-0.08, 0.0], observed=(-0.05, 0.01), amount=0.5)
        # per full step it actually did (-0.1, 0.02); move 30% of the way
        self.assertAlmostEqual(column[0], -0.08 + 0.3 * (-0.1 + 0.08))
        self.assertAlmostEqual(column[1], 0.3 * 0.02)
        self.assertEqual(align.broyden_update([-0.08, 0.0], (-0.01, 0.0), 0.1), [-0.08, 0.0])

    def test_broyden_learns_a_wrong_model(self):
        truth, model = [-0.1, 0.0], [-0.03, 0.02]
        for _ in range(15):
            model = align.broyden_update(model, (truth[0] * 0.5, truth[1] * 0.5), 0.5)
        self.assertAlmostEqual(model[0], -0.1, places=2)
        self.assertAlmostEqual(model[1], 0.0, places=2)

    def test_within_tolerance(self):
        tol = {"u": 0.04, "h": 0.08, "v": 0.05}
        self.assertTrue(align.within((0.03, -0.05, 0.01), tol))
        self.assertFalse(align.within((0.05, 0.0, 0.0), tol))
        self.assertFalse(align.within((0.0, 0.09, 0.0), tol))


class CalibrationFileTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = Path(self.tmp.name) / "calibration.json"
        self.setup = {"speeds": {"turn": 20, "forward": 8, "strafe": 6}, "step_seconds": 0.3,
                      "kneel": 0.0, "arm": [80, 30]}

    def tearDown(self):
        self.tmp.cleanup()

    def test_save_load_and_compatibility(self):
        calibration = dict(self.setup, deg_per_turn_step=12.0,
                           columns={"turn": [-0.07, 0.0], "forward": [0.0, 0.06], "strafe": None})
        align.save_calibration(self.path, calibration)
        loaded = align.load_calibration(self.path)
        self.assertEqual(loaded["columns"]["turn"], [-0.07, 0.0])
        self.assertTrue(align.compatible(loaded, self.setup))
        self.assertFalse(align.compatible(loaded, dict(self.setup, step_seconds=0.4)))
        self.assertFalse(align.compatible(loaded, dict(self.setup, kneel=0.5)))

    def test_missing_or_damaged_calibration_loads_as_none(self):
        self.assertIsNone(align.load_calibration(self.path))
        self.path.write_text('{"version": 1, "colu')
        self.assertIsNone(align.load_calibration(self.path))
        self.path.write_text(json.dumps({"version": 1, "columns": {"turn": ["x", 0]}}))
        self.assertIsNone(align.load_calibration(self.path))


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


class SimRobot:
    """Hidden truth: box bearing `theta` (deg, + = right of centre), apparent size `h`.

    Turning left (+) swings the box to the right in the image. The camera sees +-30 deg.
    Frames are blurry while the legs move and for `shake` seconds after they stop.
    """

    def __init__(self, clock, theta=8.0, h=0.12, deg_per_step=12.0, fwd_gain=0.06,
                 strafe_deg=1.8, step_seconds=0.3, shake=0.5, box_returns=True, sign=1):
        self.clock, self.theta, self.h = clock, theta, h
        self.deg_per_step, self.fwd_gain, self.strafe_deg = deg_per_step, fwd_gain, strafe_deg
        self.step_seconds, self.shake, self.box_returns, self.sign = (
            step_seconds, shake, box_returns, sign)
        self.moving, self.started, self.stopped_at = None, None, -10.0
        self.steps = []
        self.turned = 0.0

    # legs interface
    def step(self, primitive, amount):
        self.moving, self.started = (primitive, 1 if amount > 0 else -1), self.clock()
        self.steps.append((primitive, round(amount, 3)))

    def stop(self):
        if self.moving is None:
            return
        primitive, direction = self.moving
        amount = direction * (self.clock() - self.started) / self.step_seconds * self.sign
        if primitive == "turn":
            self.theta += self.deg_per_step * amount
            self.turned += self.deg_per_step * amount
        elif primitive == "forward":
            self.h *= 1 + self.fwd_gain * amount
        else:
            self.theta += self.strafe_deg * amount
        self.theta = (self.theta + 180) % 360 - 180
        self.moving, self.stopped_at = None, self.clock()

    # camera interface
    def latest(self):
        visible = abs(self.theta) < 30 and (self.box_returns or abs(self.turned) < 90)
        shaky = self.moving is not None or self.clock() - self.stopped_at < self.shake
        frame = {"features": ({"u": 0.5 + self.theta / 60, "v": 0.4 + self.h, "h": self.h}
                              if visible else None),
                 "sharp": 20.0 if shaky else 400.0}
        return frame, self.clock()


def measure(frame):
    return (frame["features"], None) if frame["features"] else (None, "no purple box in view")


class Pose:
    def __init__(self):
        self.calls = []

    def prepare(self, kneel, arm):
        self.calls.append((kneel, arm))


REFERENCE = {"features": {"u": 0.5, "v": 0.6, "h": 0.2}, "state": {"kneel": 0.0, "arm": [80, 30]}}


class AlignerHarness(unittest.TestCase):
    def make(self, motion=True, reference=REFERENCE, recalibrate=False, **sim):
        self.clock = Clock()
        self.robot = SimRobot(self.clock, **sim)
        self.pose = Pose()
        self.log = []
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.cal_path = Path(self.tmp.name) / "calibration.json"
        observer = align.Observer(self.robot, measure, lambda f: f["sharp"], self.clock,
                                  min_sharpness=50)
        self.aligner = align.Aligner(
            legs=self.robot, pose=self.pose, observer=observer, reference=lambda: reference,
            calibration_path=self.cal_path, log=self.log.append, clock=self.clock,
            motion=motion, recalibrate=recalibrate, settle=0.7, step_seconds=0.3,
            speeds={"turn": 20, "forward": 8, "strafe": 6})
        return self.aligner

    def run_for(self, seconds, held=True, manual=False):
        owned = False
        for _ in range(int(seconds / 0.05)):
            self.clock.now += 0.05
            owned = self.aligner.update(held, manual)
        return owned

    def said(self, text):
        return any(text in line for line in self.log)


class ObserverTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.robot = SimRobot(self.clock)
        self.observer = align.Observer(self.robot, measure, lambda f: f["sharp"], self.clock,
                                       min_sharpness=50, timeout=2.0)

    def poll_for(self, seconds):
        result = None
        for _ in range(int(seconds / 0.05)):
            self.clock.now += 0.05
            result = self.observer.poll()
            if result is not None:
                return result
        return result

    def test_waits_for_frames_after_settle_and_needs_two_agreeing(self):
        self.observer.begin(self.clock.now + 0.5)
        self.clock.now += 0.4
        self.assertIsNone(self.observer.poll())  # before the settle time
        status, features = self.poll_for(1.0)
        self.assertEqual(status, "ok")
        self.assertAlmostEqual(features["u"], 0.5 + 8 / 60)
        self.assertGreaterEqual(self.clock.now, 100.55)  # two frames after settle

    def test_blurry_frames_are_skipped(self):
        self.robot.stopped_at = self.clock.now  # just stopped: shaky for 0.5 s
        self.observer.begin(self.clock.now)
        status, _ = self.poll_for(1.5)
        self.assertEqual(status, "ok")
        self.assertGreaterEqual(self.clock.now, 100.55)

    def test_disagreeing_frames_wait_for_a_stable_pair(self):
        self.observer.begin(self.clock.now)
        self.clock.now += 0.05
        self.assertIsNone(self.observer.poll())
        self.robot.theta += 3  # the view jumps between the two frames
        self.clock.now += 0.05
        self.assertIsNone(self.observer.poll())
        self.clock.now += 0.05
        self.assertEqual(self.observer.poll()[0], "ok")

    def test_missing_box_and_stalled_camera(self):
        self.robot.theta = 90
        self.observer.begin(self.clock.now)
        status, reason = self.poll_for(1.0)
        self.assertEqual((status, reason), ("lost", "no purple box in view"))
        self.robot.latest = lambda: ({"features": None, "sharp": 400.0}, 50.0)  # frozen camera
        self.observer.begin(self.clock.now)
        self.assertEqual(self.poll_for(3.0)[0], "stalled")


class AlignerTests(AlignerHarness):
    def test_calibration_measures_the_360_turn_and_probes(self):
        self.make()
        self.run_for(60)
        calibration = align.load_calibration(self.cal_path)
        self.assertIsNotNone(calibration, self.log)
        # each simulated step is 0.30-0.35 s of a 0.3 s nominal step at 12 deg per 0.3 s
        self.assertTrue(11.0 <= calibration["deg_per_turn_step"] <= 15.0,
                        calibration["deg_per_turn_step"])
        turn_u = calibration["columns"]["turn"][0]
        self.assertGreater(turn_u, 0.15)  # turning left moves the box right in the image
        self.assertGreater(calibration["columns"]["forward"][1], 0.03)
        self.assertGreater(calibration["columns"]["strafe"][0], 0.01)
        turns = [s for s in self.robot.steps if s[0] == "turn"]
        self.assertGreaterEqual(len(turns), 25)  # a full revolution, step by step
        self.assertTrue(self.said("CALIBRATE done"))
        self.assertEqual(self.pose.calls, [(0.0, [80, 30])])  # reference arm/kneel first

    def test_aligns_after_calibrating_and_reports_aligned(self):
        self.make()
        self.run_for(150)
        self.assertTrue(self.said("ALIGNED"), self.log[-5:])
        eu, eh, _ = align.errors({"u": 0.5 + self.robot.theta / 60, "v": 0.4 + self.robot.h,
                                  "h": self.robot.h}, REFERENCE["features"])
        self.assertLessEqual(abs(eu), 0.04)
        self.assertLessEqual(abs(eh), 0.08)
        self.assertIsNone(self.robot.moving)  # legs stopped once aligned
        moves = len(self.robot.steps)
        self.run_for(5)
        self.assertEqual(len(self.robot.steps), moves)  # holds still while A is still held

    def test_saved_calibration_is_reused(self):
        self.make()
        self.run_for(150)
        first = len([s for s in self.robot.steps if s[0] == "turn"])
        path = self.cal_path
        self.make(theta=6.0, h=0.15)
        self.cal_path.parent.mkdir(parents=True, exist_ok=True)
        self.cal_path.write_text(path.read_text() if path.exists() else "")
        self.aligner.calibration_path = self.cal_path
        self.run_for(60)
        self.assertTrue(self.said("ALIGNED"), self.log[-5:])
        self.assertLess(len([s for s in self.robot.steps if s[0] == "turn"]), first)
        self.assertFalse(self.said("CALIBRATE turn"))

    def test_calibration_aborts_if_the_box_never_comes_back(self):
        self.make(box_returns=False)
        self.run_for(120)
        self.assertTrue(self.said("not seen again"))
        self.assertIsNone(align.load_calibration(self.cal_path))
        self.assertIsNone(self.robot.moving)

    def test_release_watchdog_and_manual_input_stop_the_legs(self):
        for stop in ("release", "watchdog", "manual"):
            self.make()
            self.run_for(3)
            self.assertTrue(self.robot.steps)
            if stop == "release":
                self.assertFalse(self.run_for(0.05, held=False))
            elif stop == "watchdog":
                self.aligner.watchdog()
            else:
                self.assertFalse(self.run_for(0.05, manual=True))
            self.assertIsNone(self.robot.moving, stop)
            self.assertFalse(self.aligner.active, stop)

    def test_display_only_never_moves_anything(self):
        self.make(motion=False, theta=10.0)
        self.run_for(5)
        self.assertEqual(self.robot.steps, [])
        self.assertEqual(self.pose.calls, [])
        self.assertTrue(self.said("eu=+0.17"))
        self.assertTrue(self.said("display only"))

    def test_no_recording_refuses_once(self):
        self.make(reference=None)
        self.assertFalse(self.run_for(2))
        self.assertEqual(sum("No recording" in line for line in self.log), 1)
        self.assertEqual(self.robot.steps, [])

    def test_box_not_in_view_at_start_aborts(self):
        self.make(theta=90.0)
        self.run_for(5)
        self.assertTrue(self.said("box not in view"))
        self.assertEqual(self.robot.steps, [])

    def test_growing_error_aborts(self):
        self.make()
        self.run_for(60)  # calibrate with the true directions
        self.make(sign=-1)  # legs now move opposite to the calibrated model
        self.cal_path.parent.mkdir(parents=True, exist_ok=True)
        self.aligner.calibration_path.write_text(json.dumps(dict(
            json.loads(json.dumps(align.load_calibration(self.cal_path) or {})),
            version=1, speeds={"turn": 20, "forward": 8, "strafe": 6}, step_seconds=0.3,
            kneel=0.0, arm=[80, 30], deg_per_turn_step=14.0,
            columns={"turn": [0.23, 0.0], "forward": [0.0, 0.07], "strafe": [0.03, 0.0]})))
        self.run_for(60)
        # wrong-way steps either trip the growth check or unlearn the model until no step helps
        self.assertTrue(self.said("error grew") or self.said("no step"), self.log[-5:])
        self.assertIsNone(self.robot.moving)

    def test_view_mismatch_aborts_when_size_fits_but_height_does_not(self):
        reference = {"features": {"u": 0.5 + 8 / 60, "v": 0.45, "h": 0.12},
                     "state": {"kneel": 0.0, "arm": [80, 30]}}
        self.make(reference=reference)
        self.cal_path.write_text(json.dumps(dict(
            version=1, speeds={"turn": 20, "forward": 8, "strafe": 6}, step_seconds=0.3,
            kneel=0.0, arm=[80, 30], deg_per_turn_step=14.0,
            columns={"turn": [0.23, 0.0], "forward": [0.0, 0.07], "strafe": [0.03, 0.0]})))
        self.run_for(10)
        self.assertTrue(self.said("view mismatch"), self.log[-5:])


if __name__ == "__main__":
    unittest.main()
