"""
MJPEG HTTP server and the newest-JPEG mailbox.

Purpose:
    The Pi has no display over SSH. This module serves ``/`` (HTML + stats
    panel), ``/stream`` (multipart JPEGs), and ``/stats`` (JSON telemetry).
    ``LiveHub`` holds only the latest JPEG so a slow browser skips frames
    instead of stalling the camera.

Key terms:
    - LiveHub: One-slot mailbox. publish() from the camera thread; wait_next()
      from each HTTP client thread.
    - MJPEG: Motion JPEG. A still JPEG after another, never a finished page.
    - multipart/x-mixed-replace: The HTTP content type browsers use for MJPEG.

Exports:
    - LiveHub, start_http_server, CameraHTTPServer.
"""

from __future__ import annotations

import json
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

from .models import JPEG_QUALITY, MJPEG_BOUNDARY, PAGE_HTML


class LiveHub:
    """Mailbox that stores only the newest camera JPEG.

    :param: none
    """

    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._jpeg: bytes | None = None
        self._sequence = 0

    def publish(self, frame: object) -> None:
        """Replace the stored JPEG with a compressed copy of ``frame``.

        :param frame: BGR image from ``VideoCapture.read``.
        :returns: None.
        """

        encoded, buffer = cv2.imencode(
            ".jpg",
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY],
        )
        if not encoded:
            return
        with self._condition:
            self._jpeg = buffer.tobytes()
            self._sequence += 1
            self._condition.notify_all()

    def wake(self) -> None:
        """Unblock viewers waiting for the next frame so the process can exit.

        :returns: None.
        """

        with self._condition:
            self._condition.notify_all()

    def wait_next(self, last_sequence: int, timeout: float = 1.0) -> tuple[int, bytes | None]:
        """Block until a JPEG newer than ``last_sequence`` arrives, or we time out.

        :param last_sequence: Counter the caller already sent. Zero means none yet.
        :param timeout: Seconds to wait before returning the current frame.
        :returns: ``(sequence, jpeg_bytes)``. jpeg is None until the first publish.
        """

        with self._condition:
            if self._sequence == last_sequence:
                self._condition.wait(timeout=timeout)
            return self._sequence, self._jpeg


class StreamHandler(BaseHTTPRequestHandler):
    """Serve HTML, MJPEG, and JSON stats.

    :param: filled by ``BaseHTTPRequestHandler``.
    """

    hub: LiveHub | None = None
    stop: threading.Event | None = None
    telemetry: object | None = None

    def do_GET(self) -> None:
        """Answer ``/``, ``/stream``, and ``/stats``.

        :returns: None.
        """

        path = self.path.split("?", 1)[0]
        if path == "/stream":
            self._write_stream()
            return
        if path == "/stats":
            self._write_stats()
            return
        body = PAGE_HTML.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _write_stats(self) -> None:
        """Return the latest robot telemetry as JSON.

        :returns: None.
        """

        if self.telemetry is None:
            payload = {"ok": False, "error": "Telemetry disabled (--no-telemetry)."}
        else:
            payload = self.telemetry.snapshot()
        body = json.dumps(payload).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _write_stream(self) -> None:
        """Push each new JPEG until the browser disconnects or the robot stops.

        :returns: None.
        """

        if self.hub is None or self.stop is None:
            self.send_error(503, "Camera hub is not ready")
            return
        self.send_response(200)
        self.send_header(
            "Content-Type",
            f"multipart/x-mixed-replace; boundary={MJPEG_BOUNDARY.decode('ascii')}",
        )
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        sequence = 0
        while not self.stop.is_set():
            sequence, jpeg = self._next_jpeg(sequence)
            if jpeg is None:
                continue
            try:
                self.wfile.write(b"--" + MJPEG_BOUNDARY + b"\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii"))
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, TimeoutError, OSError):
                break

    def _next_jpeg(self, sequence: int) -> tuple[int, bytes | None]:
        """Wait for a JPEG this client has not already sent.

        :param sequence: Last sequence number written to this client.
        :returns: New sequence and JPEG, or the same sequence and None on timeout.
        """

        assert self.hub is not None
        new_sequence, jpeg = self.hub.wait_next(sequence)
        if new_sequence == sequence:
            return sequence, None
        return new_sequence, jpeg

    def log_message(self, format: str, *args: object) -> None:
        """Log one line per connection, not per JPEG.

        :param format: ``BaseHTTPRequestHandler`` format string.
        :param args: Values for that format string.
        :returns: None.
        """

        sys.stderr.write("[xgo-camera] " + (format % args) + "\n")


class CameraHTTPServer(ThreadingHTTPServer):
    """HTTP server that allows a quick restart after Ctrl+C."""

    allow_reuse_address = True
    daemon_threads = True


def start_http_server(
    bind: str,
    port: int,
    hub: LiveHub,
    stop: threading.Event,
    telemetry: object | None = None,
) -> CameraHTTPServer:
    """Listen for browsers in a background thread.

    :param bind: Interface address, usually ``0.0.0.0``.
    :param port: TCP port, usually 8080.
    :param hub: Mailbox the camera publishes JPEGs into.
    :param stop: Set this when the camera loop ends so stream handlers exit.
    :param telemetry: Optional poller with a ``snapshot()`` method for ``GET /stats``.
    :returns: The running server.
    :raises RuntimeError: If the port is already taken.
    """

    StreamHandler.hub = hub
    StreamHandler.stop = stop
    StreamHandler.telemetry = telemetry
    try:
        server = CameraHTTPServer((bind, port), StreamHandler)
    except OSError as exc:
        raise RuntimeError(
            f"Could not listen on {bind}:{port} ({exc}). "
            "Another copy of this script may still be running."
        ) from exc
    thread = threading.Thread(target=server.serve_forever, name="xgo-http", daemon=True)
    thread.start()
    return server
