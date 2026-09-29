"""Camera-only HSV detector for three colored boxes; never commands the robot."""

import argparse
import json
from pathlib import Path

import cv2  # type: ignore[import-not-found]
import numpy as np  # type: ignore[import-not-found]

# OpenCV H is 0–179; S and V are 0–255. Tune these on actual robot-camera frames.
HSV_RANGES = {
    "purple": ((125, 65, 60), (160, 255, 255)),
    "orange": ((5, 90, 90), (25, 255, 255)),
    "light-blue": ((85, 55, 80), (110, 255, 255)),
}
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
KERNEL = np.ones((5, 5), dtype=np.uint8)


def positive_int(value):
    try:
        number = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("must be an integer") from exc
    if number < 1:
        raise argparse.ArgumentTypeError("must be at least 1")
    return number


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--order", nargs=3, required=True, choices=tuple(HSV_RANGES), metavar="COLOR",
        help="three different colors, bottom to top: purple orange light-blue (any order)",
    )
    parser.add_argument(
        "--source", default="0", metavar="CAMERA_OR_PATH",
        help="camera index (default: 0), image path, or video path",
    )
    parser.add_argument("--frames", type=positive_int, default=60, help="max frames for camera/video (default: 60)")
    parser.add_argument(
        "--stable-frames", type=positive_int, default=3,
        help="consecutive frames with exactly one candidate per color (default: 3)",
    )
    parser.add_argument("--min-area", type=positive_int, default=150, help="minimum color area in pixels")
    parser.add_argument("--output", type=Path, metavar="IMAGE", help="save an annotated image of the final frame")
    return parser


def detect_boxes(frame, min_area=150):
    """Return all plausible colored rectangles; do not silently choose between duplicates."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    found = {}
    for color, (lower, upper) in HSV_RANGES.items():
        mask = cv2.inRange(hsv, np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area:
                continue
            x, y, width, height = cv2.boundingRect(contour)
            if not 0.5 <= width / height <= 2.0 or area / (width * height) < 0.5:
                continue
            candidates.append({
                "bbox": [x, y, width, height],
                "center": [x + width // 2, y + height // 2],
                "area_px": round(area),
            })
        found[color] = sorted(candidates, key=lambda item: item["area_px"], reverse=True)
    return found


def evaluate(detections, order):
    missing = [color for color in order if not detections[color]]
    ambiguous = [color for color in order if len(detections[color]) > 1]
    status = "missing" if missing else "ambiguous" if ambiguous else "ready"
    return {
        "status": status,
        "order_bottom_to_top": order,
        "missing": missing,
        "ambiguous": ambiguous,
        "detections": detections,
        # This is an observation order, NOT coordinates or instructions for a gripper.
        "observed_order": [
            {"color": color, **detections[color][0]} for color in order
        ] if status == "ready" else [],
    }


def annotate(frame, detections):
    annotated = frame.copy()
    for color, candidates in detections.items():
        for candidate in candidates:
            x, y, width, height = candidate["bbox"]
            cv2.rectangle(annotated, (x, y), (x + width, y + height), (0, 255, 0), 2)
            cv2.putText(
                annotated, color, (x, max(y - 6, 12)), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (255, 255, 255), 2,
            )
    return annotated


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    if len(set(args.order)) != 3:
        parser.error("--order must list each color exactly once")
    if args.source.isdecimal():
        try:
            source = int(args.source)
        except ValueError:
            parser.error("camera index is too large")
        image = None
    else:
        source = Path(args.source)
        if not source.is_file():
            parser.error(f"source does not exist: {source}")
        image = cv2.imread(str(source)) if source.suffix.lower() in IMAGE_SUFFIXES else None
        if source.suffix.lower() in IMAGE_SUFFIXES and image is None:
            parser.error(f"cannot read image: {source}")

    cap = None
    if image is None:
        cap = cv2.VideoCapture(str(source) if isinstance(source, Path) else source)
        if not cap.isOpened():
            parser.error(f"cannot open camera/video source: {source}")

    result = None
    last_frame = None
    stable = 0
    required = 1 if image is not None else args.stable_frames
    try:
        for _ in range(1 if image is not None else args.frames):
            if image is not None:
                frame = image
            else:
                if cap is None:
                    raise RuntimeError("video source was not initialized")
                ok, frame = cap.read()
                if not ok:
                    break
            last_frame = frame
            detections = detect_boxes(frame, args.min_area)
            result = evaluate(detections, args.order)
            stable = stable + 1 if result["status"] == "ready" else 0
            if stable >= required:
                break
    finally:
        if cap is not None:
            cap.release()

    if result is None:
        parser.exit(2, "no frames read; check the camera or video source\n")
    result["stable_frames"] = stable
    if stable < required and result["status"] == "ready":
        result["status"] = "unstable"
        result["observed_order"] = []
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(args.output), annotate(last_frame, result["detections"])):
            parser.exit(2, f"could not save image: {args.output}\n")
    print(json.dumps(result, indent=2))
    return 0 if stable >= required else 2


if __name__ == "__main__":
    raise SystemExit(main())
