"""
Small vision and I/O helpers shared by detect, camera, and chase.

Purpose:
    Box geometry (IoU, NMS, scaling, drawing), AVI paths, FPS fallbacks,
    Tailscale URL printing, opening xgolib, and applying ``--small-objects``.
    Nothing in here opens a camera or starts HTTP.

Key terms:
    - IoU: Intersection over union. How much two boxes overlap, 0 to 1.
    - NMS: Non-maximum suppression. When tiled YOLO draws the same ball
      twice, keep the higher-confidence copy.
    - FourCC: Codec id written into the AVI (MJPG on this Pi).
    - xgolib: The vendor Python library that talks to the motion board
      on ``/dev/ttyAMA0``.

Exports:
    - class_color, class_name_from_label, nms_detections, scale_detections,
      draw_detections, detect_colored_ball, default_output_path, resolve_fps,
      open_capture, open_writer, print_live_urls, apply_small_object_flags,
      open_xgo.
"""

from __future__ import annotations

import subprocess
import sys
from argparse import Namespace
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from .models import (
    BALL_HSV_RANGES,
    DEFAULT_FPS,
    DEFAULT_YOLO_CONF,
    Detection,
    RECORD_FOURCC,
    SCRIPT_DIR,
)


def class_color(class_id: int) -> tuple[int, int, int]:
    """Pick a stable BGR color for a COCO class id.

    :param class_id: YOLO class index (0 is person, 1 is bicycle, and so on).
    :returns: A ``(blue, green, red)`` tuple in the 0–255 range.
    """

    seed = (37 * (class_id + 1) + 17) % 255
    return (seed, (seed * 3) % 255, (seed * 7) % 255)


def class_name_from_label(label: str) -> str:
    """Pull the COCO name out of a caption like ``person 0.91``.

    :param label: Detection caption drawn on the frame.
    :returns: Lowercased class name, or the whole label if it has no score.
    """

    parts = label.rsplit(" ", 1)
    if len(parts) == 2:
        return parts[0].strip().lower()
    return label.strip().lower()


def _iou(a: Detection, b: Detection) -> float:
    """Intersection-over-union of two axis-aligned boxes.

    :param a: First detection.
    :param b: Second detection.
    :returns: Overlap in ``[0, 1]``.
    """

    ax1, ay1, ax2, ay2 = a[:4]
    bx1, by1, bx2, by2 = b[:4]
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    inter = max(0, ix2 - ix1) * max(0, iy2 - iy1)
    if inter <= 0:
        return 0.0
    area_a = max(0, ax2 - ax1) * max(0, ay2 - ay1)
    area_b = max(0, bx2 - bx1) * max(0, by2 - by1)
    union = area_a + area_b - inter
    return inter / union if union > 0 else 0.0


def _score_from_label(label: str) -> float:
    """Read the trailing confidence in a caption like ``sports ball 0.41``.

    :param label: Detection caption.
    :returns: Score, or 0 if the caption has no number.
    """

    try:
        return float(label.rsplit(" ", 1)[-1])
    except ValueError:
        return 0.0


def nms_detections(detections: list[Detection], iou_threshold: float = 0.45) -> list[Detection]:
    """Drop overlapping boxes, keeping the higher-confidence copy.

    :param detections: Boxes in one frame, possibly duplicated by tiling.
    :param iou_threshold: Merge when overlap is at least this high.
    :returns: A shorter list, highest scores first.
    """

    ordered = sorted(detections, key=lambda item: _score_from_label(item[4]), reverse=True)
    kept: list[Detection] = []
    for candidate in ordered:
        if all(_iou(candidate, existing) < iou_threshold for existing in kept):
            kept.append(candidate)
    return kept


def detect_colored_ball(
    frame: object,
    color_name: str,
    class_label: str,
    min_area: int,
) -> list[Detection]:
    """Find round-ish blobs of a solid color and return them as detections.

    :param frame: BGR camera image.
    :param color_name: Key in ``BALL_HSV_RANGES``.
    :param class_label: Caption class so chase ``--class`` can match.
    :param min_area: Ignore speckles smaller than this many pixels.
    :returns: Zero or more detections in this frame's coordinates.
    """

    ranges = BALL_HSV_RANGES.get(color_name.lower())
    if not ranges:
        return []

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = None
    for lower, upper in ranges:
        piece = cv2.inRange(hsv, np.array(lower), np.array(upper))
        mask = piece if mask is None else cv2.bitwise_or(mask, piece)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    found: list[Detection] = []
    color_bgr = class_color(32)
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < min_area:
            continue
        perimeter = cv2.arcLength(contour, True)
        if perimeter <= 0:
            continue
        circularity = 4.0 * 3.14159265 * area / (perimeter * perimeter)
        if circularity < 0.45:
            continue
        x, y, width, height = cv2.boundingRect(contour)
        score = min(0.99, 0.55 + 0.45 * min(1.0, circularity))
        label = f"{class_label} {score:.2f}"
        found.append((x, y, x + width, y + height, label, color_bgr))
    return found


def scale_detections(
    detections: list[Detection],
    src_width: int,
    src_height: int,
    dst_width: int,
    dst_height: int,
) -> list[Detection]:
    """Move boxes from the camera frame onto a resized display frame.

    :param detections: Boxes in the coordinate system YOLO inferred on.
    :param src_width: Width of the frame submitted to YOLO.
    :param src_height: Height of that frame.
    :param dst_width: Width of the frame about to be shown.
    :param dst_height: Height of that frame.
    :returns: The same labels, with corners scaled and clamped.
    """

    if src_width <= 0 or src_height <= 0:
        return []
    if src_width == dst_width and src_height == dst_height:
        return detections

    scale_x = dst_width / src_width
    scale_y = dst_height / src_height
    scaled: list[Detection] = []
    for x1, y1, x2, y2, label, color in detections:
        left = min(dst_width - 1, max(0, int(x1 * scale_x)))
        top = min(dst_height - 1, max(0, int(y1 * scale_y)))
        right = min(dst_width - 1, max(0, int(x2 * scale_x)))
        bottom = min(dst_height - 1, max(0, int(y2 * scale_y)))
        scaled.append((left, top, right, bottom, label, color))
    return scaled


def draw_detections(frame: object, detections: list[Detection]) -> None:
    """Paint rectangles and class labels onto ``frame`` in place.

    :param frame: BGR image that will be streamed and recorded.
    :param detections: Boxes already in this frame's pixel coordinates.
    :returns: None.
    """

    height = int(frame.shape[0])
    font_scale = max(0.6, height / 720.0 * 0.8)
    thickness = 2 if height >= 480 else 1
    for x1, y1, x2, y2, label, color in detections:
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)
        (text_width, text_height), baseline = cv2.getTextSize(
            label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness
        )
        text_top = y1 - text_height - baseline - 4
        if text_top < 0:
            text_top = y1
        cv2.rectangle(
            frame,
            (x1, text_top),
            (x1 + text_width + 6, text_top + text_height + baseline + 4),
            color,
            -1,
        )
        cv2.putText(
            frame,
            label,
            (x1 + 3, text_top + text_height + 1),
            cv2.FONT_HERSHEY_SIMPLEX,
            font_scale,
            (255, 255, 255),
            thickness,
            cv2.LINE_AA,
        )


def default_output_path() -> Path:
    """Build a timestamped AVI path in this package folder.

    :returns: Absolute path like ``.../xgo_robot/xgo_camera_20261005_120000.avi``.
    """

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return SCRIPT_DIR / f"xgo_camera_{stamp}.avi"


def resolve_output_path(output: str | None, no_record: bool) -> Path | None:
    """Resolve ``--output`` the same way both commands used to.

    :param output: CLI path, or None for a timestamped default.
    :param no_record: If True, return None (no AVI).
    :returns: Absolute path or None.
    """

    if no_record:
        return None
    path = Path(output).expanduser() if output else default_output_path()
    if not path.is_absolute():
        path = SCRIPT_DIR / path
    return path


def resolve_fps(capture: cv2.VideoCapture, override: float | None) -> float:
    """Pick a playback FPS that will not produce a zero-rate AVI.

    :param capture: An already-opened camera.
    :param override: CLI ``--fps`` value, or None to read the driver.
    :returns: A positive FPS used by VideoWriter.
    """

    if override is not None and override > 0:
        return float(override)
    reported = float(capture.get(cv2.CAP_PROP_FPS) or 0.0)
    if reported > 1.0:
        return reported
    return DEFAULT_FPS


def open_capture(source: str, width: int, height: int) -> cv2.VideoCapture:
    """Open the XGO camera (or a remote stream) and request preview size.

    :param source: Camera index as a string ("0") or a stream URL.
    :param width: Requested frame width in pixels.
    :param height: Requested frame height in pixels.
    :returns: An opened ``VideoCapture``.
    :raises RuntimeError: If the device or URL will not open.
    """

    if source.isdigit():
        camera_index = int(source)
        if sys.platform.startswith("linux"):
            capture = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
        else:
            capture = cv2.VideoCapture(camera_index)
    else:
        capture = cv2.VideoCapture(source)

    if not capture.isOpened():
        raise RuntimeError(
            f"Could not open camera source {source!r}. "
            "Run this on the XGO over SSH, or pass a working stream URL."
        )

    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    actual_width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
    actual_height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
    print(f"Camera reports {actual_width}x{actual_height} (asked for {width}x{height}).")
    return capture


def open_writer(path: Path, frame: object, fps: float) -> cv2.VideoWriter:
    """Create an MJPG AVI writer whose size matches the first real frame.

    :param path: Destination file, normally next to this package.
    :param frame: First BGR image. Used only for height/width.
    :param fps: Playback rate stored in the AVI header.
    :returns: An opened ``VideoWriter``.
    :raises RuntimeError: If OpenCV cannot encode on this Pi.
    """

    height, width = frame.shape[:2]
    fourcc = cv2.VideoWriter_fourcc(*RECORD_FOURCC)
    writer = cv2.VideoWriter(str(path), fourcc, fps, (width, height))
    if not writer.isOpened():
        raise RuntimeError(
            f"Could not open video writer for {path}. "
            "MJPG/AVI is missing or the path is not writable."
        )
    return writer


def tailscale_ipv4_addresses() -> list[str]:
    """Ask the local Tailscale client which 100.x address this machine has.

    :returns: IPv4 Tailscale addresses, or an empty list if the CLI is missing.
    """

    try:
        completed = subprocess.run(
            ["tailscale", "ip", "-4"],
            check=False,
            capture_output=True,
            text=True,
            timeout=3,
        )
    except (OSError, subprocess.TimeoutExpired):
        return []
    return [line.strip() for line in completed.stdout.splitlines() if line.strip()]


def print_live_urls(port: int) -> None:
    """Print the browser URL your laptop should open.

    :param port: TCP port the MJPEG server is bound to.
    :returns: None.
    """

    addresses = tailscale_ipv4_addresses()
    print("Live view — open this in a browser on your computer:")
    if not addresses:
        print(f"  http://<pi-tailscale-ip>:{port}/")
        return
    for address in addresses:
        print(f"  http://{address}:{port}/")
    print("Use the URL above (ends with /), not /stream alone — the HTML stats panel is only on /.")
    print(
        f"OpenCV window on your computer: python3 main.py view --url http://{addresses[0]}:{port}/stream"
    )


def apply_small_object_flags(args: Namespace) -> None:
    """If ``--small-objects`` is set, turn on 2x2 tiles and a lower conf.

    Leaves an explicit ``--slices`` / ``--conf`` alone when the user already
    changed them from the defaults.

    :param args: Mutated in place.
    :returns: None.
    """

    if not getattr(args, "small_objects", False):
        return
    if getattr(args, "slices", 1) <= 1:
        args.slices = 2
    if getattr(args, "conf", DEFAULT_YOLO_CONF) >= DEFAULT_YOLO_CONF:
        args.conf = 0.12


def open_xgo(port: str) -> object:
    """Open a dedicated xgolib connection when telemetry is disabled.

    :param port: Serial device, usually ``/dev/ttyAMA0``.
    :returns: An ``XGO`` instance.
    :raises RuntimeError: If xgolib is missing or the port will not open.
    """

    try:
        from xgolib import XGO
    except ImportError as exc:
        raise RuntimeError(
            "xgolib is not installed. On the Pi run: pip3 install xgolib"
        ) from exc

    try:
        return XGO("xgolite", port=port)
    except TypeError:
        return XGO(port=port, version="xgolite")
    except OSError as exc:
        raise RuntimeError(f"Could not open {port}: {exc}") from exc
