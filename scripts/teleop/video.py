"""Live MJPEG view of the robot camera, served only on the robot's Tailscale address."""

import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

PAGE = (b"<!doctype html><title>XGO camera</title>"
        b"<body style='margin:0;background:#111'>"
        b"<img src=\"/stream\" style='width:100%;height:100vh;object-fit:contain'></body>")


def tailscale_ipv4(run=subprocess.run):
    """The robot's Tailscale IPv4 address, or None if Tailscale is unavailable."""
    try:
        result = run(["tailscale", "ip", "-4"], capture_output=True, text=True, timeout=3)
    except (OSError, subprocess.TimeoutExpired):
        return None
    lines = result.stdout.split() if result.returncode == 0 else []
    return lines[0] if lines else None


class VideoServer:
    def __init__(self, camera, host, port, encode=None, fps=10):
        self.camera = camera
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
                    data = self.encode(frame)
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
