"""Multi-box observations and conservative verification; no UART or motor imports."""

import math
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "box_detection"))
from detect_boxes import HSV_RANGES, detect_boxes  # noqa: E402

COLORS = tuple(HSV_RANGES)


def roi(value):
    if (not isinstance(value, (list, tuple)) or len(value) != 4
            or any(isinstance(v, bool) or not isinstance(v, (float, int))
                   or not math.isfinite(v) for v in value)):
        raise ValueError("ROI must be [left, top, right, bottom] in normalized image coordinates")
    x0, y0, x1, y1 = value
    if not (0 <= x0 < x1 <= 1 and 0 <= y0 < y1 <= 1):
        raise ValueError("ROI must have positive area inside the image")
    return list(value)


def overlaps(a, b):
    return min(a[2], b[2]) > max(a[0], b[0]) and min(a[3], b[3]) > max(a[1], b[1])


def contains(region, box):
    x, y, w, h = box["bbox"]
    return region[0] <= x and region[1] <= y and x + w <= region[2] and y + h <= region[3]


def features(box):
    x, y, w, h = box["bbox"]
    return {"u": x + w / 2, "v": y + h / 2, "w": w, "h": h}


class Perception:
    def __init__(self, marker_id=0, min_area=60):
        if not isinstance(marker_id, int) or isinstance(marker_id, bool) or not 0 <= marker_id < 50:
            raise ValueError("marker ID must be 0..49 (DICT_4X4_50)")
        if not hasattr(cv2, "aruco"):
            raise RuntimeError("This OpenCV lacks ArUco; use an ArUco-capable build before motion")
        self.marker_id = marker_id
        self.min_area = min_area
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters()
        # Pi OpenCV 4.7 groups away some valid markers at the default distance (80px ID 0
        # reproduces it). Keep candidates, then merge nested footprints of the SAME ID.
        # Error-correction/border acceptance is unchanged; separate prints stay ambiguous.
        parameters.minMarkerDistanceRate = 0.01
        self.detector = cv2.aruco.ArucoDetector(dictionary, parameters)

    def observe(self, frame):
        height, width = frame.shape[:2]
        found = detect_boxes(frame, min_area=self.min_area)  # disjoint multi-color ranges
        boxes = {}
        for color, candidates in found.items():
            boxes[color] = []
            for candidate in candidates:
                x, y, w, h = candidate["bbox"]
                if x <= 2 or y <= 2 or x + w >= width - 2 or y + h >= height - 2:
                    continue  # clipped colors cannot verify a grasp or a tower
                boxes[color].append({"bbox": [x / width, y / height, w / width, h / height],
                                     "area_px": candidate["area_px"]})
        corners, ids, _ = self.detector.detectMarkers(frame)
        markers = []
        for index, marker_id in enumerate([] if ids is None else ids.flatten()):
            if int(marker_id) != self.marker_id:
                continue
            points = corners[index].reshape(4, 2) / np.array([width, height])
            if np.any(points <= 0.005) or np.any(points >= 0.995):
                continue
            x0, y0 = points.min(axis=0)
            x1, y1 = points.max(axis=0)
            edge = points[1] - points[0]
            markers.append({"u": float((x0 + x1) / 2), "v": float((y0 + y1) / 2),
                            "w": float(x1 - x0), "h": float(y1 - y0),
                            "angle": math.degrees(math.atan2(edge[1], edge[0])),
                            "corners": points.tolist()})
        distinct = []
        for marker in sorted(markers, key=lambda m: m["w"] * m["h"], reverse=True):
            if not any(np.max(np.abs(np.array(marker["corners"]) - np.array(old["corners"])))
                       <= 0.12 * min(old["w"], old["h"]) for old in distinct):
                distinct.append(marker)
        return {"boxes": boxes, "marker": distinct[0] if len(distinct) == 1 else None,
                "marker_count": len(distinct), "frame_size": [width, height]}


def candidates(scene, color, region, excluded=()):
    return [box for box in scene["boxes"].get(color, [])
            if contains(region, box) and not any(contains(r, box) for r in excluded)]


def inventory(scene, colors, floor, excluded=()):
    found = {color: candidates(scene, color, floor, excluded) for color in colors}
    return {"ready": all(len(items) == 1 for items in found.values()),
            "missing": [c for c, items in found.items() if not items],
            "ambiguous": [c for c, items in found.items() if len(items) > 1], "targets": found}


def pickup_verdict(scene, color, floor, held, grip=None, grip_profile=None):
    """held=None (the floor rule): the carried color anywhere in the floor region means it is
    not held; otherwise it counts as held (the lifted box is above the camera's view, and a
    later tower check confirms it arrived). With a held region, a visible held box + no floor
    box succeeds and disappearance alone is unknown."""
    on_floor = candidates(scene, color, floor)
    if held is None:
        return "floor" if on_floor else "held"
    in_hand = candidates(scene, color, held)
    if len(on_floor) > 1 or len(in_hand) > 1 or (on_floor and in_hand):
        return "unknown"
    result = "held" if in_hand and not on_floor else "floor" if on_floor and not in_hand else "unknown"
    if grip_profile is not None:
        band = grip_profile["occupied" if result == "held" else "empty"]
        if grip is None or not band[0] <= grip <= band[1]:
            return "unknown"
    return result


def marker_matches(marker, reference, tolerance=0.025):
    if marker is None or "corners" not in reference:
        return False
    # Corner agreement checks marker rotation/perspective as well as center/size.
    a, b = np.array(marker["corners"]), np.array(reference["corners"])
    return a.shape == b.shape == (4, 2) and bool(np.max(np.abs(a - b)) <= tolerance)


def tower_verdict(scene, expected, stack, held, reference):
    """Visible bottom-to-top colors, adjacent faces and empty held ROI; not a 3-D force check."""
    if not marker_matches(scene["marker"], reference):
        return "unknown"
    if any(candidates(scene, color, held) for color in COLORS):
        return "unknown"
    if any(candidates(scene, color, stack) for color in COLORS if color not in expected):
        return "unknown"
    rows = [candidates(scene, color, stack) for color in expected]
    if not all(len(row) == 1 for row in rows):
        return "unknown"
    boxes = [row[0]["bbox"] for row in rows]
    for lower, upper in zip(boxes, boxes[1:]):
        lx, ly, lw, lh = lower
        ux, uy, uw, uh = upper
        if not uy + uh / 2 < ly + lh / 2:
            return "unknown"
        if abs((ux + uw / 2) - (lx + lw / 2)) > 0.25 * min(lw, uw):
            return "unknown"
        if abs(uy + uh - ly) > 0.25 * min(lh, uh):
            return "unknown"  # same colors side by side or widely separated are not a tower
    return "placed"


class InventoryTracker:
    """One identity per unique color; missing, duplicates or jumps break continuity."""
    def __init__(self, required, floor, excluded=(), count=3):
        self.required, self.floor, self.excluded, self.count = required, floor, excluded, count
        self.tracks, self.generation, self.stamp = {}, 0, None

    def update(self, scene, stamp):
        report = inventory(scene, self.required, self.floor, self.excluded)
        if self.stamp is not None and stamp <= self.stamp:
            return dict(report, ready=False, tracks={}, reason="stale frame")
        self.stamp = stamp
        for color, boxes in report["targets"].items():
            if len(boxes) != 1:
                self.tracks.pop(color, None)
                continue
            value = features(boxes[0])
            old = self.tracks.get(color)
            continuous = (old is not None and abs(value["u"] - old["u"]) <= 0.03
                          and abs(value["v"] - old["v"]) <= 0.03
                          and abs(value["h"] / old["h"] - 1) <= 0.15)
            if not continuous:
                self.generation += 1
            self.tracks[color] = dict(value, id=old["id"] if continuous else
                                      color + ":" + str(self.generation),
                                      hits=old["hits"] + 1 if continuous else 1)
        report["tracks"] = dict(self.tracks)
        report["ready"] = report["ready"] and all(
            self.tracks[color]["hits"] >= self.count for color in self.required)
        return report


class Stable:
    """Require consecutive *fresh* consistent evidence; never reuse an old frame."""
    def __init__(self, count=3):
        self.count = count
        self.reset()

    def reset(self):
        self.previous = None
        self.streak = 0
        self.stamp = None

    def update(self, value, stamp):
        if self.stamp is not None and stamp <= self.stamp:
            return False
        self.stamp = stamp
        if value is None or value == "unknown":
            self.previous, self.streak = None, 0
            return False
        self.streak = self.streak + 1 if value == self.previous else 1
        self.previous = value
        return self.streak >= self.count


def make_marker(path, marker_id=0):
    Perception(marker_id)  # validate ID and API before writing
    image = cv2.aruco.generateImageMarker(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), marker_id, 600)
    image = cv2.copyMakeBorder(image, 100, 100, 100, 100, cv2.BORDER_CONSTANT, value=255)
    path = Path(path)
    if path.exists():
        raise ValueError("marker output already exists")
    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), image):
        raise RuntimeError("could not save marker")
