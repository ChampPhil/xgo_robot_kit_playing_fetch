"""Offline checks for teach-and-repeat; no robot, camera or GPU required."""

import json
import tempfile
import unittest
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

import teach

BOX = {"bbox": [140, 100, 40, 40], "center": [160, 120], "area_px": 1600}


def detections(*boxes, color="purple"):
    found = {"purple": [], "orange": [], "light-blue": []}
    found[color] = list(boxes)
    return found


class FakeClock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


class FakeCamera:
    def __init__(self, clock):
        self.clock = clock
        self.frame = np.zeros((240, 320, 3), dtype=np.uint8)
        self.stamp = clock.now

    def latest(self):
        return self.frame, self.stamp

    def new_frame(self):
        self.stamp = self.clock.now


def purple_frame():
    hsv = np.zeros((240, 320, 3), dtype=np.uint8)
    cv2.rectangle(hsv, (140, 100), (180, 140), (140, 200, 210), -1)
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


STATE = {"arm": [80, 30], "claw": 128, "kneel": 0.0, "walk": {"x": 0, "y": 0, "yaw": 0}}


class FeatureTests(unittest.TestCase):
    def test_features_of_one_box_from_the_real_detector(self):
        import detect_boxes

        frame = purple_frame()
        features, reason = teach.box_features(detect_boxes.detect_boxes(frame), "purple",
                                              frame.shape)
        self.assertIsNone(reason)
        self.assertAlmostEqual(features["u"], 0.5, places=1)
        self.assertAlmostEqual(features["h"], 41 / 240, places=2)
        self.assertEqual(features["frame_size"], [320, 240])

    def test_missing_ambiguous_and_clipped_boxes_are_rejected(self):
        shape = (240, 320, 3)
        self.assertIn("no purple", teach.box_features(detections(), "purple", shape)[1])
        self.assertIn("ambiguous", teach.box_features(detections(BOX, BOX), "purple", shape)[1])
        edge = {"bbox": [0, 100, 40, 40], "center": [20, 120], "area_px": 1600}
        self.assertIn("edge", teach.box_features(detections(edge), "purple", shape)[1])


class RecorderTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name) / "xgo_teach"
        self.clock = FakeClock()
        self.camera = FakeCamera(self.clock)
        self.free = [1000]
        self.seen = detections(BOX)
        self.recorder = teach.Recorder(
            self.root, self.camera, lambda frame: self.seen, "purple", fps=10, min_free_mb=200,
            clock=self.clock, free_mb=lambda path: self.free[0],
            now=lambda: datetime(2026, 10, 5, 12, 0, 0), encode=lambda frame: b"jpeg")

    def tearDown(self):
        self.tmp.cleanup()

    def session(self):
        return self.root / "20261005-120000"

    def test_start_writes_reference_and_refuses_without_a_unique_box(self):
        self.seen = detections()
        self.assertIn("no purple", self.recorder.start(STATE))
        self.assertFalse(self.recorder.active)
        self.seen = detections(BOX)
        self.assertIsNone(self.recorder.start(STATE))
        reference = json.loads((self.session() / "reference.json").read_text())
        self.assertEqual(reference["color"], "purple")
        self.assertEqual(reference["state"], STATE)
        self.assertEqual((self.session() / "reference.jpg").read_bytes(), b"jpeg")

    def test_start_refuses_stale_frame_and_low_disk(self):
        self.clock.now += 5
        self.assertIn("stale", self.recorder.start(STATE))
        self.camera.new_frame()
        self.free[0] = 100
        self.assertIn("MB free", self.recorder.start(STATE))
        self.assertFalse(self.root.exists() and any(self.root.iterdir()))

    def test_motion_events_and_atomic_motion_file(self):
        self.recorder.start(STATE)
        self.clock.now += 0.5
        self.recorder.on_send("arm", [90, 30])
        self.clock.now += 0.25
        self.recorder.on_send("claw", 200)
        self.recorder.on_send("kneel", 0.5)
        self.clock.now += 1
        summary = self.recorder.stop()
        motion = json.loads((self.session() / "motion.json").read_text())
        self.assertEqual(motion["start"], {"arm": [80, 30], "claw": 128, "kneel": 0.0})
        self.assertEqual(motion["events"], [{"t": 0.5, "arm": [90, 30]},
                                            {"t": 0.75, "claw": 200},
                                            {"t": 0.75, "kneel": 0.5}])
        self.assertEqual(motion["duration"], 1.75)
        self.assertEqual(summary["events"], 3)
        self.assertFalse(self.recorder.active)
        self.assertEqual(list(self.session().glob("*.tmp")), [])

    def test_events_are_on_disk_before_stop(self):
        self.recorder.start(STATE)
        self.recorder.on_send("arm", [90, 30])
        lines = (self.session() / "motion_events.jsonl").read_text().splitlines()
        self.assertEqual(json.loads(lines[0]), {"t": 0.0, "arm": [90, 30]})
        self.assertFalse((self.session() / "motion.json").exists())

    def test_events_ignored_when_not_recording(self):
        self.recorder.on_send("arm", [90, 30])
        self.assertFalse(self.root.exists())

    def test_dataset_frames_at_fps_with_command_state_and_detection(self):
        self.recorder.start(STATE)
        for _ in range(10):  # 20 Hz ticks for 0.5 s at 10 fps -> 5 frames
            self.recorder.tick({"x": 1}, STATE)
            self.clock.now += 0.05
            self.camera.new_frame()
        lines = [json.loads(line)
                 for line in (self.session() / "dataset.jsonl").read_text().splitlines()]
        self.assertEqual(len(lines), 5)
        self.assertEqual(lines[0]["frame"], "frames/000000.jpg")
        self.assertEqual(lines[0]["command"], {"x": 1})
        self.assertEqual(lines[0]["state"], STATE)
        self.assertEqual(lines[0]["detection"]["bbox"], BOX["bbox"])
        self.assertTrue((self.session() / "frames" / "000004.jpg").exists())
        self.assertEqual(self.recorder.stop()["frames"], 5)

    def test_same_frame_is_not_written_twice(self):
        self.recorder.start(STATE)
        self.recorder.tick({}, STATE)
        self.clock.now += 0.2  # time passes but the camera delivered nothing new
        self.recorder.tick({}, STATE)
        self.assertEqual(len((self.session() / "dataset.jsonl").read_text().splitlines()), 1)

    def test_lost_box_is_recorded_as_null_detection(self):
        self.recorder.start(STATE)
        self.seen = detections()
        self.recorder.tick({}, STATE)
        line = json.loads((self.session() / "dataset.jsonl").read_text())
        self.assertIsNone(line["detection"])

    def test_disk_guard_stops_recording(self):
        self.recorder.start(STATE)
        self.free[0] = 150
        self.assertIn("MB free", self.recorder.tick({}, STATE))
        self.assertFalse(self.recorder.active)
        self.assertTrue((self.session() / "motion.json").exists())

    def test_two_sessions_in_one_second_get_distinct_folders(self):
        self.recorder.start(STATE)
        self.recorder.stop()
        self.recorder.start(STATE)
        self.recorder.stop()
        self.assertEqual(sorted(p.name for p in self.root.iterdir()),
                         ["20261005-120000", "20261005-120000-2"])


class FakeArm:
    def __init__(self):
        self.calls = []

    def set_pose(self, x, z):
        self.calls.append(("arm", x, z))

    def set_claw(self, value):
        self.calls.append(("claw", value))


class FakePosture:
    def __init__(self, calls):
        self.calls = calls

    def set_level(self, level):
        self.calls.append(("kneel", level))


MOTION = {"version": 1, "color": "purple", "duration": 2.0,
          "start": {"arm": [80, 30], "claw": 0, "kneel": 0.0},
          "events": [{"t": 0.5, "arm": [90, 20]}, {"t": 1.0, "claw": 200},
                     {"t": 1.5, "kneel": 0.5}]}


class ReplayTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.clock = FakeClock()
        self.arm = FakeArm()
        self.posture = FakePosture(self.arm.calls)
        self.replayer = teach.Replayer(self.arm, self.posture, clock=self.clock, settle=1.0)

    def tearDown(self):
        self.tmp.cleanup()

    def save(self, name, motion):
        (self.root / name).mkdir()
        (self.root / name / "motion.json").write_text(json.dumps(motion))

    def test_latest_session_and_load(self):
        self.assertIsNone(teach.latest_session(self.root))
        self.save("20261005-110000", MOTION)
        self.save("20261005-120000", MOTION)
        (self.root / "20261005-130000").mkdir()  # unfinished: no motion.json
        self.assertEqual(teach.latest_session(self.root).name, "20261005-120000")
        self.assertEqual(teach.load_motion(teach.latest_session(self.root)), MOTION)
        self.assertIsNone(teach.latest_session(self.root / "missing"))

    def test_load_rejects_bad_motion(self):
        for bad in ({"version": 2}, dict(MOTION, events=[{"t": -1, "arm": [1, 2]}]),
                    dict(MOTION, events=[{"t": 1, "leg": 3}]),
                    dict(MOTION, events=[{"t": 1, "arm": [1, 2], "claw": 3}])):
            self.save("bad", bad)
            with self.assertRaises(ValueError):
                teach.load_motion(self.root / "bad")
            (self.root / "bad" / "motion.json").unlink()
            (self.root / "bad").rmdir()
        self.save("cut", MOTION)
        (self.root / "cut" / "motion.json").write_text('{"version": 1, "ev')  # power cut
        with self.assertRaises(ValueError):
            teach.load_motion(self.root / "cut")

    def test_replay_sends_start_then_events_on_time(self):
        self.replayer.start(MOTION)
        self.assertEqual(self.arm.calls, [("kneel", 0.0), ("arm", 80, 30), ("claw", 0)])
        self.clock.now += 1.4  # 0.4 s after the 1 s settle
        self.assertIsNone(self.replayer.tick())
        self.assertEqual(len(self.arm.calls), 3)
        self.clock.now += 0.2
        self.replayer.tick()
        self.assertEqual(self.arm.calls[-1], ("arm", 90, 20))
        self.clock.now += 1.0
        self.assertEqual(self.replayer.tick(), "done")
        self.assertEqual(self.arm.calls[-2:], [("claw", 200), ("kneel", 0.5)])
        self.assertFalse(self.replayer.active)

    def test_replay_skips_unknown_start_pose_and_abort_stops(self):
        self.replayer.start(dict(MOTION, start={"arm": None, "claw": None, "kneel": 0.0}))
        self.assertEqual(self.arm.calls, [("kneel", 0.0)])
        self.replayer.abort()
        self.clock.now += 10
        self.assertIsNone(self.replayer.tick())
        self.assertEqual(len(self.arm.calls), 1)


class FakeMessage:
    def __init__(self, record=0, replay=0, raw=None):
        self.record, self.replay, self.raw = record, replay, raw or {}


class FakeRecorder:
    def __init__(self, reason=None):
        self.active, self.reason, self.ticks, self.stopped = False, reason, [], 0

    def start(self, state):
        if self.reason:
            return self.reason
        self.active = True
        self.session = "/tmp/session"
        return None

    def tick(self, command, state):
        self.ticks.append(command)
        return None

    def stop(self):
        if not self.active:
            return None
        self.active = False
        self.stopped += 1
        return {"session": "/tmp/session", "frames": 3, "events": 2, "duration": 1.5}


class FakeCam:
    def __init__(self, error=None):
        self.error, self.started, self.closed = error, 0, False

    def start(self):
        if self.error:
            raise RuntimeError(self.error)
        self.started += 1

    def close(self):
        self.closed = True


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.clock = FakeClock()
        self.arm = FakeArm()
        self.replayer = teach.Replayer(self.arm, FakePosture(self.arm.calls), clock=self.clock,
                                       settle=0)
        self.recorder = FakeRecorder()
        self.camera = FakeCam()
        self.log = []
        self.teach = teach.TeachController(self.camera, self.recorder, self.replayer, self.root,
                                           lambda: STATE, self.log.append)

    def tearDown(self):
        self.tmp.cleanup()

    def save_motion(self):
        (self.root / "20261005-120000").mkdir()
        (self.root / "20261005-120000" / "motion.json").write_text(json.dumps(MOTION))

    def test_x_toggles_recording_and_ticks_while_recording(self):
        self.teach.update(FakeMessage(record=1), manual=False)
        self.assertTrue(self.recorder.active)
        self.assertEqual(self.camera.started, 1)
        self.teach.update(FakeMessage(record=1, raw={"x": 1}), manual=True)
        self.assertEqual(self.recorder.ticks[-1], {"x": 1})
        self.teach.update(FakeMessage(record=2), manual=False)
        self.assertFalse(self.recorder.active)
        self.assertTrue(any("3 frames" in line for line in self.log))

    def test_record_refusal_is_reported(self):
        self.recorder.reason = "no purple box in view"
        self.teach.update(FakeMessage(record=1), manual=False)
        self.assertFalse(self.recorder.active)
        self.assertTrue(any("no purple box" in line for line in self.log))

    def test_record_reports_camera_failure(self):
        self.camera.error = "cannot open camera 0"
        self.teach.update(FakeMessage(record=1), manual=False)
        self.assertFalse(self.recorder.active)
        self.assertTrue(any("cannot open camera" in line for line in self.log))

    def test_replay_without_recordings(self):
        self.assertFalse(self.teach.update(FakeMessage(replay=1), manual=False))
        self.assertEqual(self.arm.calls, [])
        self.assertTrue(any("No recording" in line for line in self.log))

    def test_replay_with_corrupt_motion_file(self):
        (self.root / "20261005-120000").mkdir()
        (self.root / "20261005-120000" / "motion.json").write_text("{not json")
        self.assertFalse(self.teach.update(FakeMessage(replay=1), manual=False))
        self.assertEqual(self.arm.calls, [])
        self.assertTrue(any("Cannot replay" in line for line in self.log))

    def test_replay_runs_and_manual_input_aborts(self):
        self.save_motion()
        self.assertTrue(self.teach.update(FakeMessage(replay=1), manual=False))
        self.assertFalse(self.teach.update(FakeMessage(replay=1), manual=True))
        self.assertFalse(self.replayer.active)
        self.assertTrue(any("aborted" in line for line in self.log))

    def test_replay_and_record_exclude_each_other(self):
        self.save_motion()
        self.teach.update(FakeMessage(record=1), manual=False)
        self.teach.update(FakeMessage(record=1, replay=1), manual=False)
        self.assertFalse(self.replayer.active)
        self.teach.update(FakeMessage(record=2, replay=1), manual=False)
        self.teach.update(FakeMessage(record=2, replay=2), manual=False)
        self.assertTrue(self.replayer.active)
        self.teach.update(FakeMessage(record=3, replay=2), manual=False)
        self.assertFalse(self.recorder.active)
        self.assertTrue(any("ignored" in line for line in self.log))

    def test_watchdog_aborts_replay_and_close_finalises(self):
        self.save_motion()
        self.teach.update(FakeMessage(replay=1), manual=False)
        self.teach.watchdog()
        self.assertFalse(self.replayer.active)
        self.teach.update(FakeMessage(replay=1, record=1), manual=False)
        self.teach.close()
        self.assertEqual(self.recorder.stopped, 1)
        self.assertTrue(self.camera.closed)

if __name__ == "__main__":
    unittest.main()
