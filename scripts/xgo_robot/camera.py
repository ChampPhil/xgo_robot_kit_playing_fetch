"""
Camera capture loop: annotate, record AVI, publish JPEGs.

Purpose:
    ``run_recording`` is the one while-True that ``stream``, ``chase-person``,
    and ``chase-ball`` share. Chase passes ``on_frame`` to steer without
    copying this loop. The detector may be YOLO or HSV; this file only
    calls ``submit`` / ``snapshot``.
    ``run_viewer`` is the laptop-side OpenCV window for ``/stream``.

Key terms:
    - on_frame: Optional callback ``fn(display, detections)`` after boxes
      are drawn. Chase uses it to walk.
    - upscale: If the driver ignores 1280x720, we resize so the page is large.

Exports:
    - run_recording, run_viewer.
"""

from __future__ import annotations

from pathlib import Path
from urllib.error import URLError
from urllib.request import urlopen

import cv2
import numpy as np

# Any object with submit/snapshot/close: YoloAnnotator or ColorBallAnnotator.
from .helpers import draw_detections, open_writer, scale_detections
from .http_stream import LiveHub
from .models import QUIT_KEY


def run_recording(
    capture: cv2.VideoCapture,
    output: Path | None,
    fps: float,
    preview: bool,
    hub: LiveHub | None,
    annotator: object | None,
    target_width: int,
    target_height: int,
    on_frame: object | None = None,
) -> None:
    """Read frames until Ctrl+C (or q in preview), annotate, save, and publish.

    :param capture: An already-opened OpenCV VideoCapture.
    :param output: Adjacent AVI path to create, or None to skip the file.
    :param fps: Playback FPS for the writer.
    :param preview: If True, also call ``imshow`` (needs a display).
    :param hub: Live JPEG mailbox, or None when ``--no-serve`` is set.
    :param annotator: Detector with submit/snapshot/close, or None.
    :param target_width: Minimum width of the picture sent to the browser.
    :param target_height: Minimum height of that picture.
    :param on_frame: Optional ``fn(display, detections)`` after boxes are drawn.
    :returns: None.
    """

    writer: cv2.VideoWriter | None = None
    window_name = "XGO camera (press q to quit)"
    frame_count = 0
    told_upscale = False

    try:
        while True:
            ok, frame = capture.read()
            if not ok or frame is None:
                print("Lost the camera stream; stopping.")
                break

            if annotator is not None:
                annotator.submit(frame)
                detections = annotator.snapshot()
            else:
                detections = []

            src_height, src_width = frame.shape[:2]
            display = frame
            if src_width < target_width or src_height < target_height:
                display = cv2.resize(
                    frame,
                    (target_width, target_height),
                    interpolation=cv2.INTER_LINEAR,
                )
                detections = scale_detections(
                    detections, src_width, src_height, target_width, target_height
                )
                if not told_upscale:
                    print(
                        f"Camera delivered {src_width}x{src_height}; "
                        f"scaling the feed to {target_width}x{target_height}."
                    )
                    told_upscale = True

            if detections:
                draw_detections(display, detections)
            if on_frame is not None:
                on_frame(display, detections)

            if output is not None and writer is None:
                writer = open_writer(output, display, fps)
                print(
                    f"Recording {display.shape[1]}x{display.shape[0]} "
                    f"@ {fps:g} fps -> {output}"
                )
            if writer is not None:
                writer.write(display)
            frame_count += 1
            if hub is not None:
                hub.publish(display)
            if preview:
                cv2.imshow(window_name, display)
                if cv2.waitKey(1) & 0xFF == ord(QUIT_KEY):
                    break
    except KeyboardInterrupt:
        print("\nStopped by Ctrl+C.")
    finally:
        if output is not None:
            print(f"Wrote {frame_count} frames to {output}")
        else:
            print(f"Showed {frame_count} frames (no AVI written).")
        if writer is not None:
            writer.release()
        capture.release()
        if preview:
            cv2.destroyAllWindows()


def run_viewer(url: str) -> None:
    """Draw the Pi's MJPEG page in an OpenCV window on this computer.

    :param url: Stream URL, for example ``http://100.74.30.90:8080/stream``.
    :returns: None.
    :raises RuntimeError: If the robot is not serving, or the connection drops.
    """

    window_name = "XGO camera (press q to quit)"
    jpeg_start = b"\xff\xd8"
    jpeg_end = b"\xff\xd9"
    try:
        response = urlopen(url, timeout=10)
    except URLError as exc:
        raise RuntimeError(
            f"Could not open {url} ({exc}). "
            "Start python3 main.py stream on the robot first, and use its Tailscale IP."
        ) from exc

    buffer = b""
    try:
        while True:
            chunk = response.read(8192)
            if not chunk:
                raise RuntimeError("The robot closed the live stream.")
            buffer += chunk
            while True:
                start = buffer.find(jpeg_start)
                if start == -1:
                    buffer = buffer[-1:]
                    break
                end = buffer.find(jpeg_end, start + 2)
                if end == -1:
                    buffer = buffer[start:]
                    break
                payload = buffer[start : end + 2]
                buffer = buffer[end + 2 :]
                frame = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is None:
                    continue
                cv2.imshow(window_name, frame)
                if cv2.waitKey(1) & 0xFF == ord(QUIT_KEY):
                    return
    except KeyboardInterrupt:
        print("\nClosed the viewer.")
    finally:
        response.close()
        cv2.destroyAllWindows()
