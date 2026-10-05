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

    def test_video_start_failure_is_reported(self):
        def broken():
            raise RuntimeError("cannot open camera 0")

        server = video.VideoServer(SimpleNamespace(start=broken), "127.0.0.1", 0)
        with self.assertRaisesRegex(RuntimeError, "cannot open camera"):
            server.start()


if __name__ == "__main__":
    unittest.main()
