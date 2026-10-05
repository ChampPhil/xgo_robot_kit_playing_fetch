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

    def test_latest_before_start(self):
        self.assertEqual(camera.Camera().latest(), (None, None))


if __name__ == "__main__":
    unittest.main()
