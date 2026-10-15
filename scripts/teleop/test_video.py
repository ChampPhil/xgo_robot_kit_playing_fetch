"""Offline checks for the live MJPEG stream; uses a local socket, no robot camera."""

import time
import unittest
import urllib.request
from types import SimpleNamespace

import numpy as np

import video


class VideoTests(unittest.TestCase):
    def test_stream_serves_jpeg_parts_and_index(self):
        frame = np.zeros((4, 4, 3), dtype=np.uint8)
        camera = SimpleNamespace(start=lambda: None, latest=lambda: (frame, time.monotonic()))
        server = video.VideoServer(camera, "127.0.0.1", 0, encode=lambda f: b"JPEGDATA", fps=50)
        server.start()
        try:
            page = urllib.request.urlopen(server.url, timeout=3).read().decode()
            self.assertIn('src="/stream"', page)
            with urllib.request.urlopen(server.url + "stream", timeout=3) as response:
                self.assertIn("multipart/x-mixed-replace", response.headers["Content-Type"])
                chunk = response.read(200)
            self.assertIn(b"--frame", chunk)
            self.assertIn(b"JPEGDATA", chunk)
        finally:
            server.close()

    def test_overlay_is_drawn_on_streamed_frames_not_the_camera_frame(self):
        frame = np.zeros((4, 4, 3), dtype=np.uint8)
        seen = []

        def overlay(image):
            marked = image.copy()
            marked[0, 0] = 255
            return marked

        camera = SimpleNamespace(start=lambda: None, latest=lambda: (frame, time.monotonic()))
        server = video.VideoServer(camera, "127.0.0.1", 0, fps=50, overlay=overlay,
                                   encode=lambda image: seen.append(image.copy()) or b"J")
        server.start()
        try:
            with urllib.request.urlopen(server.url + "stream", timeout=3) as response:
                response.read(100)
        finally:
            server.close()
        self.assertEqual(seen[0][0, 0].tolist(), [255, 255, 255])  # streamed frame marked
        self.assertEqual(frame[0, 0].tolist(), [0, 0, 0])  # camera/recording frame untouched

    def test_reference_overlay_draws_the_target_box_scaled_to_the_frame(self):
        reference = {"features": {"bbox": [100, 200, 40, 60], "frame_size": [640, 480]},
                     "state": {}}
        clock = [0.0]
        loads = []

        def load():
            loads.append(1)
            return reference

        draw = video.reference_overlay(load, clock=lambda: clock[0], refresh=2.0)
        frame = np.zeros((240, 320, 3), dtype=np.uint8)  # half size: box at (50,100) 20x30
        marked = draw(frame)
        self.assertEqual(frame.sum(), 0)  # original untouched
        self.assertEqual(marked[100, 50].tolist(), [0, 0, 255])  # red (BGR) corner
        self.assertEqual(marked[115, 60].tolist(), [0, 0, 0])  # inside stays clear
        draw(frame)
        clock[0] = 2.5
        draw(frame)
        self.assertEqual(len(loads), 2)  # reloaded at most every 2 s (a new recording)

    def test_reference_overlay_without_a_recording_leaves_frames_alone(self):
        draw = video.reference_overlay(lambda: None)
        frame = np.zeros((4, 4, 3), dtype=np.uint8)
        self.assertEqual(draw(frame).sum(), 0)

    def test_video_start_failure_is_reported(self):
        def broken():
            raise RuntimeError("cannot open camera 0")

        server = video.VideoServer(SimpleNamespace(start=broken), "127.0.0.1", 0)
        with self.assertRaisesRegex(RuntimeError, "cannot open camera"):
            server.start()


if __name__ == "__main__":
    unittest.main()
