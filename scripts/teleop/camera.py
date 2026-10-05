"""Background camera reader that always hands out the newest frame and its capture time.

Reading continuously in a thread avoids OpenCV's buffered (stale) frames; callers compare
the timestamp with time.monotonic() to reject old images.
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

    def start(self, timeout=3.0):
        if self.running:
            return
        self.capture = self.open_capture(self.source)
        if self.capture is None or not self.capture.isOpened():
            self.capture = None
            raise RuntimeError(f"cannot open camera {self.source} "
                               "(is the camera stream or another program using it?)")
        self.running = True
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.latest()[0] is not None:
                return
            time.sleep(0.02)
        self.close()
        raise RuntimeError(f"camera {self.source} opened but produced no frames")

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
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=1.0)
            self.thread = None
        if self.capture is not None:
            self.capture.release()
            self.capture = None
