"""Offline checks for the background camera; no real camera required."""

import time
import unittest

import numpy as np

import camera


class FakeCapture:
    def __init__(self, opened=True, frames=True):
        self.opened, self.frames, self.released, self.count = opened, frames, False, 0

    def isOpened(self):
        return self.opened

    def read(self):
        time.sleep(0.005)
        if not self.frames:
            return False, None
        self.count += 1
        return True, np.full((4, 4, 3), self.count % 255, dtype=np.uint8)

    def release(self):
        self.released = True


class CameraTests(unittest.TestCase):
    def test_latest_returns_newest_frame_with_timestamp(self):
        capture = FakeCapture()
        cam = camera.Camera(open_capture=lambda source: capture)
        cam.start()
        try:
            frame, stamp = cam.latest()
            self.assertEqual(frame.shape, (4, 4, 3))
            self.assertLessEqual(stamp, time.monotonic())
            time.sleep(0.05)
            self.assertGreater(cam.latest()[1], stamp)
        finally:
            cam.close()
        self.assertTrue(capture.released)
        self.assertFalse(cam.running)

    def test_unopenable_camera_raises_readable_error(self):
        cam = camera.Camera(open_capture=lambda source: FakeCapture(opened=False))
        with self.assertRaisesRegex(RuntimeError, "cannot open camera"):
            cam.start()
        self.assertFalse(cam.running)

    def test_camera_without_frames_raises_and_releases(self):
        capture = FakeCapture(frames=False)
        cam = camera.Camera(open_capture=lambda source: capture)
        with self.assertRaisesRegex(RuntimeError, "no frames"):
            cam.start(timeout=0.2)
        self.assertTrue(capture.released)

    def test_start_twice_is_harmless(self):
        opened = []
        cam = camera.Camera(open_capture=lambda source: opened.append(1) or FakeCapture())
        cam.start()
        cam.start()
        cam.close()
        self.assertEqual(len(opened), 1)

    def test_start_async_never_blocks_and_reports_errors(self):
        def slow_open(source):
            time.sleep(0.3)
            return FakeCapture()

        cam = camera.Camera(open_capture=slow_open)
        began = time.monotonic()
        cam.start_async()
        cam.start_async()  # repeated presses while opening are harmless
        self.assertLess(time.monotonic() - began, 0.1)
        self.assertEqual(cam.latest(), (None, None))
        time.sleep(0.45)
        self.assertIsNotNone(cam.latest()[0])
        cam.close()

        broken = camera.Camera(open_capture=lambda source: FakeCapture(opened=False))
        broken.start_async()
        time.sleep(0.05)
        self.assertIn("cannot open camera", broken.error)
        self.assertFalse(broken.running)

    def test_latest_before_start(self):
        self.assertEqual(camera.Camera().latest(), (None, None))


if __name__ == "__main__":
    unittest.main()
