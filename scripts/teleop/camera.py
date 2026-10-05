"""Background camera reader that always hands out the newest frame and its capture time.

Reading continuously in a thread avoids OpenCV's buffered (stale) frames; callers compare
the timestamp with time.monotonic() to reject old images. Opening also happens in that
thread (start_async), so the 20 Hz control loop never waits for the camera.
"""

import threading
import time


def open_cv2_capture(source):
    import cv2  # type: ignore[import-not-found]

    return cv2.VideoCapture(source)


class Camera:
    def __init__(self, source=0, open_capture=None):
        self.source = source
        self.open_capture = open_capture or open_cv2_capture
        self.lock = threading.Lock()
        self.frame = None
        self.stamp = None
        self.capture = None
        self.thread = None
        self.running = False
        self.opening = False
        self.stopping = False
        self.error = None  # why the last open failed, for messages

    def start_async(self):
        """Begin opening the camera in the background; returns at once. Safe to repeat."""
        with self.lock:
            if self.running or self.opening:
                return
            self.opening, self.stopping, self.error = True, False, None
        self.thread = threading.Thread(target=self.open_and_read, daemon=True)
        self.thread.start()

    def start(self, timeout=3.0):
        """Blocking start for callers outside the control loop; raises RuntimeError."""
        self.start_async()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.latest()[0] is not None:
                return
            if self.error:
                raise RuntimeError(self.error)
            time.sleep(0.02)
        self.close()
        raise RuntimeError(f"camera {self.source} opened but produced no frames")

    def open_and_read(self):
        capture = self.open_capture(self.source)
        with self.lock:
            self.opening = False
            if capture is None or not capture.isOpened() or self.stopping:
                if capture is not None and not self.stopping:
                    self.error = (f"cannot open camera {self.source} "
                                  "(is the camera stream or another program using it?)")
                elif capture is None:
                    self.error = f"cannot open camera {self.source}"
                failed = capture
                capture = None
            else:
                self.capture, self.running, failed = capture, True, None
        if failed is not None:
            failed.release()
        if capture is not None:
            self.loop()

    def loop(self):
        while self.running:
            ok, frame = self.capture.read()
            if ok and frame is not None:
                with self.lock:
                    self.frame, self.stamp = frame, time.monotonic()
            else:
                time.sleep(0.05)

    def latest(self):
        with self.lock:
            return self.frame, self.stamp

    def close(self):
        with self.lock:
            self.stopping = True
            self.running = False
        if self.thread is not None:
            self.thread.join(timeout=1.0)
            self.thread = None
        if self.capture is not None:
            self.capture.release()
            self.capture = None
