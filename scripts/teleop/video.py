"""Live MJPEG view of the robot camera, served only on the robot's loopback address.

The tailnet lets the Mac reach the robot over SSH but not other ports, so
gamepad_sender.py --video forwards a local port through its SSH session to here.
"""

import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

PAGE = (b"<!doctype html><title>XGO camera</title>"
        b"<body style='margin:0;background:#111'>"
        b"<img src=\"/stream\" style='width:100%;height:100vh;object-fit:contain'></body>")


LOCAL_HOST = "127.0.0.1"
TARGET_BGR = (0, 0, 255)  # red


def reference_overlay(load_reference, clock=time.monotonic, refresh=2.0):
    """Return frame -> copy with a red box where the box was in the latest recording's first
    frame (its reference). Reloads every `refresh` seconds so a new recording shows up."""
    cache = {"at": None, "reference": None}

    def draw(frame):
        now = clock()
        if cache["at"] is None or now - cache["at"] >= refresh:
            cache["at"] = now
            try:
                cache["reference"] = load_reference()
            except Exception:  # a broken reference must never stop the video
                cache["reference"] = None
        features = (cache["reference"] or {}).get("features") or {}
        bbox, size = features.get("bbox"), features.get("frame_size")
        if not bbox or not size:
            return frame
        import cv2  # type: ignore[import-not-found]

        height, width = frame.shape[:2]
        sx, sy = width / size[0], height / size[1]
        x, y, w, h = bbox
        top_left = (int(round(x * sx)), int(round(y * sy)))
        bottom_right = (int(round((x + w) * sx)), int(round((y + h) * sy)))
        marked = frame.copy()  # never draw on the camera's frame (recordings use it)
        cv2.rectangle(marked, top_left, bottom_right, TARGET_BGR, 2)
        cv2.putText(marked, "target", (top_left[0], max(12, top_left[1] - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, TARGET_BGR, 1)
        return marked

    return draw


class VideoServer:
    def __init__(self, camera, host, port, encode=None, fps=10, overlay=None):
        self.camera = camera
        self.overlay = overlay  # frame -> annotated copy, for viewers only
        self.host = host
        self.port = port
        self.encode = encode
        self.period = 1.0 / fps
        self.server = None

    def start(self):
        self.camera.start()
        if self.encode is None:
            from teach import encode_jpeg
            self.encode = encode_jpeg
        outer = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):  # keep the driving terminal quiet
                pass

            def do_GET(self):
                if self.path == "/":
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html")
                    self.end_headers()
                    self.wfile.write(PAGE)
                elif self.path == "/stream":
                    self.send_response(200)
                    self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                    self.end_headers()
                    outer.stream_to(self.wfile)
                else:
                    self.send_error(404)

        self.server = ThreadingHTTPServer((self.host, self.port), Handler)
        self.server.daemon_threads = True
        self.port = self.server.server_address[1]
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    @property
    def url(self):
        return f"http://{self.host}:{self.port}/"

    def stream_to(self, out):
        last = None
        try:
            while self.server is not None:
                frame, stamp = self.camera.latest()
                if frame is not None and stamp != last:
                    data = self.encode(self.overlay(frame) if self.overlay else frame)
                    out.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                              + str(len(data)).encode() + b"\r\n\r\n" + data + b"\r\n")
                    out.flush()
                    last = stamp
                time.sleep(self.period)
        except (BrokenPipeError, ConnectionResetError):
            pass  # viewer closed the page

    def close(self):
        if self.server is not None:
            server, self.server = self.server, None
            server.shutdown()
            server.server_close()
