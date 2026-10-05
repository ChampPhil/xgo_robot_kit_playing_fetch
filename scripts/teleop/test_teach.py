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


if __name__ == "__main__":
    unittest.main()
