"""Offline checks for gyro-assisted alignment, against a simulated robot built from the
2026-10-15 pulse probe: commands do nothing for the first ~0.5 s (gait start-up), then the
robot turns at ~0.8 deg/s per unit of turn speed; the camera spans ~94 deg; frames are
blurry while moving and for 0.5 s after stopping."""

import json
import math
import tempfile
import unittest
import unittest.mock
from pathlib import Path

import align

FOV = 94.0  # degrees across the image
DEAD = 0.5  # seconds of gait start-up before a command moves anything
K = 0.018  # box apparent height * distance (m): 0.24 at 7.5 cm, 0.03 at 60 cm
REFERENCE = {"features": {"u": 0.5, "v": 0.8, "h": 0.24}, "state": {"kneel": 1.0, "arm": None}}
SPEEDS = {"turn": 40, "forward": 12, "strafe": 10}


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


class GyroSim:
    """A robot on a floor with one box. Legs, camera and gyro interfaces for the aligner."""

    def __init__(self, clock, bearing=20.0, distance=0.6, gyro_sign=1, walk_per_speed=0.0042,
                 legs_sign=1, gyro_drift=0.0, coast=0.0, walk_dead=DEAD, hidden_above_h=None):
        self.clock = clock
        self.x = self.y = 0.0
        self.psi = 0.0  # heading, degrees, counter-clockwise positive
        # place the box `bearing` degrees to the right at `distance`
        self.bx = distance * math.cos(math.radians(-bearing))
        self.by = distance * math.sin(math.radians(-bearing))
        self.gyro_sign, self.walk_per_speed, self.legs_sign = gyro_sign, walk_per_speed, legs_sign
        self.command, self.started, self.stopped_at = None, None, -10.0
        self.steps, self.gyro_fail = [], False
        self.gyro_drift = gyro_drift  # deg/s the gyro reading creeps by itself
        self.coast = coast  # degrees a turn keeps going after stop()
        self.turn_durations = []
        self.walk_dead = walk_dead  # walking from a kneel starts slower than turning
        self.hidden_above_h = hidden_above_h  # closer than this the box is under the head
        self.closest = 1.0

    # --- legs -----------------------------------------------------------------------------
    def step(self, primitive, amount):
        self.stop()
        self.command = (primitive, 1 if amount > 0 else -1)
        self.fraction = min(1.0, abs(amount))  # |amount| = fraction of the primitive's speed
        self.started = self.clock()
        self.steps.append(self.command)

    def stop(self):
        if self.command is not None:
            self.x, self.y, self.psi = self.pose(self.clock())
            if self.command[0] == "turn":
                self.turn_durations.append(self.clock() - self.started)
                if self.clock() - self.started > DEAD:
                    self.psi += self.coast * self.command[1] * self.legs_sign
            self.stopped_at = self.clock()
        self.command = None

    def pose(self, now):
        if self.command is None:
            return self.x, self.y, self.psi
        primitive, sign = self.command
        dead = DEAD if primitive == "turn" else self.walk_dead
        moving = max(0.0, now - self.started - dead) * sign * self.legs_sign * self.fraction
        if primitive == "turn":
            return self.x, self.y, self.psi + 0.8 * SPEEDS["turn"] * moving
        travel = self.walk_per_speed * SPEEDS[primitive] * moving
        angle = math.radians(self.psi + (0 if primitive == "forward" else 90))
        return self.x + travel * math.cos(angle), self.y + travel * math.sin(angle), self.psi

    # --- gyro -----------------------------------------------------------------------------
    def read_yaw(self):
        if self.gyro_fail:
            raise TimeoutError("no valid reply from register 0x64")
        return self.gyro_sign * self.pose(self.clock())[2] + self.gyro_drift * (self.clock() - 100)

    # --- camera ---------------------------------------------------------------------------
    def view(self):
        x, y, psi = self.pose(self.clock())
        dx, dy = self.bx - x, self.by - y
        distance = math.hypot(dx, dy)
        left = math.degrees(math.atan2(dy, dx)) - psi
        right = -((left + 180) % 360 - 180)
        self.closest = min(self.closest, distance)
        if abs(right) >= FOV / 2 or distance < 0.01:
            return None
        h = K / distance
        if self.hidden_above_h is not None and h > self.hidden_above_h:
            return None
        return {"u": 0.5 + right / FOV, "v": 0.56 + h, "h": h}

    def latest(self):
        now = self.clock()
        shaky = self.command is not None or now - self.stopped_at < 0.5
        return {"features": self.view(), "sharp": 20.0 if shaky else 400.0}, now

    def errors(self):
        return align.errors(self.view(), REFERENCE["features"])


def measure(frame):
    return (frame["features"], None) if frame["features"] else (None, "no purple box in view")


class Pose:
    def __init__(self):
        self.calls = []

    def prepare(self, kneel, arm):
        self.calls.append((kneel, arm))


CALIBRATION = {"version": 2, "deg_per_u": 94.0, "gyro_sign": 1, "speeds": SPEEDS,
               "kneel": 1.0, "arm": None}


class Harness(unittest.TestCase):
    def make(self, motion=True, reference=REFERENCE, recalibrate=False, cal_dir=None,
             drift=0.0, **sim):
        self.clock = Clock()
        self.robot = GyroSim(self.clock, **sim)
        self.pose = Pose()
        self.log = []
        if cal_dir is None:
            tmp = tempfile.TemporaryDirectory()
            self.addCleanup(tmp.cleanup)
            cal_dir = tmp.name
        self.cal_path = Path(cal_dir) / "calibration.json"
        self.learned = []  # gyro directions reported to heading hold
        observer = align.Observer(self.robot, measure, lambda f: f["sharp"], self.clock,
                                  min_sharpness=50)
        self.aligner = align.GyroAligner(
            legs=self.robot, pose=self.pose, observer=observer, read_yaw=self.robot.read_yaw,
            reference=lambda: reference, calibration_path=self.cal_path, log=self.log.append,
            clock=self.clock, motion=motion, recalibrate=recalibrate, speeds=SPEEDS,
            drift=(drift if callable(drift) else (lambda: drift)),
            gyro_sign_learned=self.learned.append)
        return self.aligner

    def calibrated(self):
        self.cal_path.write_text(json.dumps(CALIBRATION))

    def run_until(self, seconds, held=True, manual=False, stop_when_aligned=True):
        start = self.clock.now
        while self.clock.now - start < seconds:
            self.clock.now += 0.05
            self.aligner.update(held, manual)
            if stop_when_aligned and self.aligner.phase == "aligned":
                break
        return self.clock.now - start

    def said(self, text):
        return any(text in line for line in self.log)


class MathTests(unittest.TestCase):
    def test_errors_relative_to_reference(self):
        eu, eh, ev = align.errors({"u": 0.56, "v": 0.58, "h": 0.18}, {"u": 0.5, "v": 0.6, "h": 0.2})
        self.assertAlmostEqual(eu, 0.06)
        self.assertAlmostEqual(eh, -0.1)
        self.assertAlmostEqual(ev, -0.02)

    def test_within_tolerance(self):
        tol = {"u": 0.04, "h": 0.08, "v": 0.05}
        self.assertTrue(align.within((0.03, -0.05, 0.01), tol))
        self.assertFalse(align.within((0.05, 0.0, 0.0), tol))
        self.assertFalse(align.within((0.0, 0.09, 0.0), tol))

    def test_walk_time_from_inverse_size(self):
        # 1/h falls linearly while walking: from h=0.03 (1/h=33.3) to 0.24 (4.17) at 10 per s
        self.assertAlmostEqual(align.walk_seconds(0.03, 0.24, 10.0), 2.916, places=2)
        self.assertLess(align.walk_seconds(0.3, 0.24, 10.0), 0)  # too close: walk back


class CalibrationFileTests(unittest.TestCase):
    def test_save_load_and_compatibility(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "calibration.json"
            setup = {"speeds": SPEEDS, "kneel": 1.0, "arm": None}
            align.save_calibration(path, dict(setup, deg_per_u=94.0, gyro_sign=1))
            loaded = align.load_calibration(path)
            self.assertEqual((loaded["deg_per_u"], loaded["gyro_sign"]), (94.0, 1))
            self.assertTrue(align.compatible(loaded, setup))
            self.assertFalse(align.compatible(loaded, dict(setup, kneel=0.0)))
            path.write_text('{"version": 2, "deg_')
            self.assertIsNone(align.load_calibration(path))
            path.write_text(json.dumps({"version": 1, "columns": {}}))  # the old format
            self.assertIsNone(align.load_calibration(path))


class ObserverTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.robot = GyroSim(self.clock, bearing=5.0)
        self.observer = align.Observer(self.robot, measure, lambda f: f["sharp"], self.clock,
                                       min_sharpness=50, timeout=2.0)

    def poll_for(self, seconds):
        for _ in range(int(seconds / 0.05)):
            self.clock.now += 0.05
            result = self.observer.poll()
            if result is not None:
                return result
        return None

    def test_waits_for_settled_sharp_agreeing_frames(self):
        self.robot.stopped_at = self.clock.now  # just stopped: blurry for 0.5 s
        self.observer.begin(self.clock.now + 0.2)
        status, features = self.poll_for(1.5)
        self.assertEqual(status, "ok")
        self.assertAlmostEqual(features["u"], 0.5 + 5 / FOV, places=3)
        self.assertGreaterEqual(self.clock.now, 100.5)

    def test_a_view_that_never_settles_times_out(self):
        flicker = [True]

        def latest():
            flicker[0] = not flicker[0]
            features = {"u": 0.5, "v": 0.6, "h": 0.2} if flicker[0] else None
            return {"features": features, "sharp": 400.0}, self.clock.now

        self.robot.latest = latest  # found, missing, found, ... never two agreeing frames
        self.observer.begin(self.clock.now)
        self.assertEqual(self.poll_for(3.0)[0], "stalled")

    def test_missing_box_and_stalled_camera(self):
        self.robot.bx = -1.0  # behind the robot
        self.observer.begin(self.clock.now)
        self.assertEqual(self.poll_for(1.0), ("lost", "no purple box in view"))
        self.robot.latest = lambda: ({"features": None, "sharp": 400.0}, 50.0)
        self.observer.begin(self.clock.now)
        self.assertEqual(self.poll_for(3.0)[0], "stalled")


class GyroAlignerTests(Harness):
    def test_quick_calibration_measures_the_camera_with_the_gyro(self):
        self.make()
        self.run_until(15, stop_when_aligned=False)
        calibration = align.load_calibration(self.cal_path)
        self.assertIsNotNone(calibration, self.log)
        self.assertAlmostEqual(calibration["deg_per_u"], FOV, delta=FOV * 0.1)
        self.assertEqual(calibration["gyro_sign"], 1)
        self.assertEqual(self.pose.calls, [(1.0, None)])  # the reference's kneel/arm first
        self.assertTrue(self.said("CALIBRATE done"))

    def test_aligns_from_two_feet_and_twenty_degrees_in_under_45_seconds(self):
        self.make()
        took = self.run_until(90)
        self.assertEqual(self.aligner.phase, "aligned", self.log[-6:])
        eu, eh, _ = self.robot.errors()
        self.assertLessEqual(abs(eu), 0.04)
        self.assertLessEqual(abs(eh), 0.08)
        self.assertLess(took, 45, self.log)
        self.assertIsNone(self.robot.command)  # legs stopped
        self.assertTrue(self.said("ALIGNED"))

    def test_saved_calibration_is_reused_and_alignment_is_faster(self):
        with tempfile.TemporaryDirectory() as tmp:
            self.make(cal_dir=tmp)
            first = self.run_until(90)
            self.make(cal_dir=tmp)
            second = self.run_until(90)
            self.assertEqual(self.aligner.phase, "aligned", self.log[-6:])
            self.assertFalse(self.said("CALIBRATE"))
            self.assertLess(second, first)

    def test_other_starting_positions(self):
        for bearing, distance in ((-15.0, 0.5), (0.0, 0.3), (30.0, 0.45)):
            self.make()
            self.calibrated()
            took = self.run_until(90)
            self.assertEqual(self.aligner.phase, "aligned", (bearing, distance, self.log[-4:]))
            self.assertLess(took, 45, (bearing, distance))

    def test_gyro_counting_the_other_way_is_handled(self):
        self.make(gyro_sign=-1)
        self.run_until(90)
        self.assertEqual(self.aligner.phase, "aligned", self.log[-6:])

    def test_too_close_walks_back(self):
        self.make(bearing=0.0, distance=0.05)
        self.calibrated()
        self.run_until(60)
        self.assertEqual(self.aligner.phase, "aligned", self.log[-6:])
        self.assertIn(("forward", -1), self.robot.steps)

    def test_gyro_failure_stops_the_turn(self):
        self.make()
        self.run_until(2.0, stop_when_aligned=False)  # calibration turn under way
        self.assertEqual(self.robot.command, ("turn", -1))  # toward the box, which is right
        self.robot.gyro_fail = True
        self.run_until(3.0, stop_when_aligned=False)
        self.assertIsNone(self.robot.command)
        self.assertFalse(self.aligner.active)
        self.assertTrue(self.said("gyro"))

    def test_box_not_in_view_at_the_start_aborts(self):
        self.make(bearing=80.0)
        self.run_until(10)
        self.assertTrue(self.said("box not in view"))
        self.assertEqual(self.robot.steps, [])

    def test_release_manual_input_and_watchdog_stop_the_legs(self):
        for stop in ("release", "manual", "watchdog"):
            self.make()
            self.run_until(2.0, stop_when_aligned=False)
            self.assertIsNotNone(self.robot.command, stop)  # mid-turn
            if stop == "release":
                self.aligner.update(False, False)
            elif stop == "manual":
                self.aligner.update(True, True)
            else:
                self.aligner.watchdog()
            self.assertIsNone(self.robot.command, stop)
            self.assertFalse(self.aligner.active, stop)

    def test_display_only_never_moves(self):
        self.make(motion=False)
        self.run_until(5)
        self.assertEqual(self.robot.steps, [])
        self.assertEqual(self.pose.calls, [])
        self.assertTrue(self.said("display only"))
        self.assertTrue(self.said("would turn right"))

    def test_no_recording_refuses_once(self):
        self.make(reference=None)
        self.run_until(2)
        self.assertEqual(sum("No recording" in line for line in self.log), 1)
        self.assertEqual(self.robot.steps, [])

    def test_view_mismatch_aborts(self):
        reference = {"features": {"u": 0.5, "v": 0.70, "h": 0.24},  # v should be ~0.8 at h 0.24
                     "state": {"kneel": 1.0, "arm": None}}
        self.make(reference=reference, bearing=0.0, distance=K / 0.24)
        self.calibrated()
        self.run_until(10)
        self.assertTrue(self.said("view mismatch"), self.log[-3:])

    def test_gyro_drift_is_subtracted_in_calibration_and_turns(self):
        self.make(gyro_drift=19.0, drift=19.0)  # the worst drift seen on the robot
        took = self.run_until(90)
        self.assertEqual(self.aligner.phase, "aligned", self.log[-6:])
        calibration = align.load_calibration(self.cal_path)
        self.assertEqual(calibration["gyro_sign"], 1)
        self.assertAlmostEqual(calibration["deg_per_u"], FOV, delta=FOV * 0.15)
        self.assertLess(took, 45)
        self.assertEqual(self.learned, [1])  # heading hold is told the calibrated direction

    def test_waits_for_a_drift_estimate_then_gives_up(self):
        self.make(drift=None)
        self.run_until(10)
        self.assertTrue(self.said("drift"), self.log)
        self.assertEqual(self.robot.steps, [])
        self.assertFalse(self.aligner.active)

    def test_turns_never_stop_before_the_legs_start_and_coast_is_handled(self):
        self.make(coast=3.0)
        self.run_until(90)
        self.assertEqual(self.aligner.phase, "aligned", self.log[-6:])
        self.assertTrue(all(d > DEAD for d in self.robot.turn_durations), self.robot.turn_durations)

    def test_a_far_too_big_blob_is_not_taken_for_the_box(self):
        self.make(bearing=0.0, distance=K / 0.7)  # 2.9x the reference size: another object
        self.calibrated()
        self.run_until(10)
        self.assertTrue(self.said("too big"), self.log[-3:])
        self.assertEqual([s for s in self.robot.steps if s[0] == "forward"], [])

    def test_a_calibration_that_cannot_be_saved_still_aligns(self):
        self.make()
        with unittest.mock.patch.object(align, "save_calibration",
                                        side_effect=OSError(28, "No space left on device")):
            self.run_until(90)
        self.assertEqual(self.aligner.phase, "aligned", self.log[-6:])
        self.assertTrue(self.said("Could not save"))

    def test_slow_walking_start_up_is_learned_without_overshooting(self):
        # measured 2026-10-21: a 1.5 s walk command from the kneel did not move the robot
        self.make(walk_dead=1.5)
        self.calibrated()
        took = self.run_until(90)
        self.assertEqual(self.aligner.phase, "aligned", self.log[-6:])
        self.assertGreater(self.robot.closest, K / (REFERENCE["features"]["h"] * 1.3))
        self.assertLess(took, 60, self.log)

    def test_box_lost_right_after_walking_forward_means_too_close_so_back_up(self):
        self.make(bearing=0.0, distance=K / 0.2, hidden_above_h=0.4)  # start nearly there
        self.calibrated()
        self.run_until(60)
        self.assertEqual(self.aligner.phase, "aligned", self.log[-6:])
        self.assertTrue(self.said("backing up"), self.log)

    def test_legs_going_the_wrong_way_are_stopped(self):
        self.make(legs_sign=-1)
        self.calibrated()
        self.run_until(60)
        self.assertNotEqual(self.aligner.phase, "aligned")
        self.assertFalse(self.aligner.active)
        self.assertIsNone(self.robot.command)


if __name__ == "__main__":
    unittest.main()
