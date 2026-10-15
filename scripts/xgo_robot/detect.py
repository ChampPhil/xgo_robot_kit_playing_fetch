"""
Person detector (YOLOv8n) and ball detector (HSV blobs), off the camera loop.

Purpose:
    Chase is no longer "run YOLO on every COCO class." There are two
    detectors, because they fail in opposite ways:

    - A person is a COCO object. YOLOv8n is the right tool. ``chase_person``
      asks the network for class id 0 only so chairs and TVs never become
      boxes.
    - A ping-pong ball is *not* a COCO ``sports ball``. That class is a
      basketball/soccer ball in the training set. At 3 m a ping-pong ball
      is a few pixels after the 640 resize and YOLO misses it. The right
      tool is an HSV color blob plus a roundness check, run every camera
      frame with no neural net.

    ``YoloAnnotator`` still runs predict in a daemon thread so the camera
    loop never waits on the network. ``ColorBallAnnotator`` is cheap enough
    to finish inside ``submit`` on the same frame.

Key terms:
    - YOLOv8n: The small COCO detector. Used only for people (and for the
      optional ``stream`` overlay). Not used when chasing a colored ball.
    - imgsz: Square size YOLO resizes to (default 640). Small objects die here.
    - slices: 2 means a 2x2 overlapping crop grid plus a full-frame pass.
    - snapshot: Latest finished boxes. For YOLO this is stale by one
      inference vs the camera. For the color ball it is the current frame.
    - HSV blob: Pixels whose hue/saturation/value sit in a named range
      (orange, red, …). Contours that are too small or not round are dropped.
    - ColorBallAnnotator: Same ``submit`` / ``snapshot`` / ``close`` shape
      as YOLO so the camera loop does not care which detector is plugged in.

Exports:
    - YoloAnnotator, ColorBallAnnotator, load_yolo, load_person_detector,
      load_ball_detector.
"""

from __future__ import annotations

import sys
import threading

from .helpers import class_color, detect_colored_ball, nms_detections
from .models import BALL_CLASS_NAME, COCO_PERSON_CLASS_ID, Detection


class YoloAnnotator:
    """Run YOLOv8n on the newest frame without blocking the camera.

    :param model: A loaded ``ultralytics.YOLO`` model.
    :param confidence: Minimum score to keep a box.
    :param image_size: Square size YOLO resizes to before the network runs.
    :param slices: 1 = one full-frame pass. 2 = 2x2 overlapping tiles.
    :param ball_color: Optional HSV preset (``orange``, ``yellow``, …).
      Only the ``stream`` overlay uses this. ``chase_colored_ball`` uses
      ``ColorBallAnnotator`` instead of mixing YOLO with blobs.
    :param ball_class: Label written on color blobs.
    :param min_ball_area: Smallest color blob, in pixels, to keep.
    :param classes: If set, keep only these COCO ids (``[0]`` is person).
    """

    def __init__(
        self,
        model: object,
        confidence: float,
        image_size: int,
        slices: int = 1,
        ball_color: str | None = None,
        ball_class: str = "sports ball",
        min_ball_area: int = 80,
        classes: list[int] | None = None,
    ) -> None:
        self._model = model
        self._confidence = confidence
        self._image_size = image_size
        self._slices = max(1, int(slices))
        self._ball_color = ball_color.lower() if ball_color else None
        self._ball_class = ball_class
        self._min_ball_area = min_ball_area
        self._classes = list(classes) if classes else None
        self._lock = threading.Lock()
        self._pending: object | None = None
        self._boxes: list[Detection] = []
        self._stop = threading.Event()
        self._new_frame = threading.Event()
        self._thread = threading.Thread(target=self._loop, name="xgo-yolo", daemon=True)
        self._thread.start()

    def submit(self, frame: object) -> None:
        """Hand the detector a private copy of the newest camera frame.

        :param frame: BGR image from ``VideoCapture.read``.
        :returns: None.
        """

        with self._lock:
            self._pending = frame.copy()
            self._new_frame.set()

    def snapshot(self) -> list[Detection]:
        """Return a copy of the latest finished boxes.

        :returns: Detections in the coordinate system of the last submitted frame.
        """

        with self._lock:
            return list(self._boxes)

    def close(self) -> None:
        """Stop the detector thread.

        :returns: None.
        """

        self._stop.set()
        self._new_frame.set()
        self._thread.join(timeout=2.0)

    def _loop(self) -> None:
        """Keep predicting until ``close`` is called.

        :returns: None.
        """

        while not self._stop.is_set():
            with self._lock:
                frame = self._pending
                self._pending = None
                if frame is None:
                    self._new_frame.clear()
            if frame is None:
                self._new_frame.wait(timeout=0.3)
                continue

            try:
                parsed = self._predict_frame(frame)
                if self._ball_color:
                    parsed.extend(
                        detect_colored_ball(
                            frame,
                            self._ball_color,
                            self._ball_class,
                            self._min_ball_area,
                        )
                    )
                    parsed = nms_detections(parsed)
            except Exception as exc:
                print(f"YOLO inference failed: {exc}", file=sys.stderr)
                continue

            with self._lock:
                self._boxes = parsed

    def _predict_frame(self, frame: object) -> list[Detection]:
        """Run YOLO on the full frame, and optionally on overlapping tiles.

        :param frame: BGR image copied from the camera thread.
        :returns: Detections in full-frame coordinates, NMS-merged.
        """

        parsed = self._parse(self._predict_crop(frame))
        if self._slices <= 1:
            return parsed

        height, width = frame.shape[:2]
        overlap = 0.25
        tile_w = min(width, int(width / self._slices * (1.0 + overlap)))
        tile_h = min(height, int(height / self._slices * (1.0 + overlap)))
        for row in range(self._slices):
            for col in range(self._slices):
                x0 = int(col * (width - tile_w) / (self._slices - 1))
                y0 = int(row * (height - tile_h) / (self._slices - 1))
                crop = frame[y0 : y0 + tile_h, x0 : x0 + tile_w]
                if crop.size == 0:
                    continue
                for x1, y1, x2, y2, label, color in self._parse(self._predict_crop(crop)):
                    parsed.append((x1 + x0, y1 + y0, x2 + x0, y2 + y0, label, color))
        return nms_detections(parsed)

    def _predict_crop(self, image: object) -> object:
        """One Ultralytics forward pass on ``image``.

        :param image: Full frame or a tile, BGR.
        :returns: The first result object from ``model.predict``.
        """

        # classes= is a YOLO predict filter. chase_person passes [0] so the
        # network still runs once but chair/tv/suitcase boxes never appear.
        predict_kw = {
            "imgsz": self._image_size,
            "conf": self._confidence,
            "verbose": False,
        }
        if self._classes is not None:
            predict_kw["classes"] = self._classes
        results = self._model.predict(image, **predict_kw)
        return results[0]

    def _parse(self, result: object) -> list[Detection]:
        """Turn one Ultralytics result into plain boxes and captions.

        :param result: The first element of ``model.predict``.
        :returns: Detections with corners in the submitted frame's pixels.
        """

        boxes = getattr(result, "boxes", None)
        if boxes is None or len(boxes) == 0:
            return []

        names = getattr(result, "names", {})
        parsed: list[Detection] = []
        for box in boxes:
            x1, y1, x2, y2 = (int(value) for value in box.xyxy[0].tolist())
            class_id = int(box.cls[0])
            if self._classes is not None and class_id not in self._classes:
                continue
            score = float(box.conf[0])
            try:
                class_name = names[class_id]
            except (KeyError, IndexError, TypeError):
                class_name = str(class_id)
            label = f"{class_name} {score:.2f}"
            parsed.append((x1, y1, x2, y2, label, class_color(class_id)))
        return parsed


class ColorBallAnnotator:
    """Find one solid-colored round blob per frame. No neural net.

    HSV + circularity is the right detector for a ping-pong ball on this
    Pi. YOLOv8n's ``sports ball`` class was trained on basketball-sized
    objects and will not see a 40 mm ball at hallway distance. This class
    matches the YOLO worker's ``submit`` / ``snapshot`` / ``close`` shape
    so ``camera.py`` does not branch.

    :param color_name: Key in ``BALL_HSV_RANGES`` (``orange``, ``red``, …).
    :param min_area: Ignore speckles smaller than this many pixels.
    :param class_label: Caption class the chaser locks onto (``ball``).
    """

    def __init__(
        self,
        color_name: str,
        min_area: int = 80,
        class_label: str = BALL_CLASS_NAME,
    ) -> None:
        self._color_name = color_name.strip().lower()
        self._min_area = min_area
        self._class_label = class_label
        self._lock = threading.Lock()
        self._boxes: list[Detection] = []

    def submit(self, frame: object) -> None:
        """Run HSV blob detection on this camera frame immediately.

        Unlike YOLO, this finishes before ``submit`` returns, so
        ``snapshot`` on the same loop iteration is the current frame.

        :param frame: BGR image from ``VideoCapture.read``.
        :returns: None.
        """

        try:
            found = detect_colored_ball(
                frame, self._color_name, self._class_label, self._min_area
            )
        except Exception as exc:
            print(f"Color-ball detection failed: {exc}", file=sys.stderr)
            return
        with self._lock:
            self._boxes = found

    def snapshot(self) -> list[Detection]:
        """Return a copy of the blobs from the last ``submit``.

        :returns: Detections in the submitted frame's coordinates.
        """

        with self._lock:
            return list(self._boxes)

    def close(self) -> None:
        """Nothing to stop. Present so the pipeline can treat detectors alike.

        :returns: None.
        """

        return None


def load_yolo(
    model_name: str,
    confidence: float,
    image_size: int,
    slices: int = 1,
    ball_color: str | None = None,
    ball_class: str = "sports ball",
    min_ball_area: int = 80,
    classes: list[int] | None = None,
) -> YoloAnnotator:
    """Load YOLOv8n and start its background thread.

    :param model_name: Weights file name or path, usually ``yolov8n.pt``.
    :param confidence: Minimum detection score.
    :param image_size: Inference size passed to ``predict``.
    :param slices: Tile grid edge length. 2 means four overlapping crops.
    :param ball_color: Optional HSV color name for the ``stream`` overlay.
    :param ball_class: Class string stored on color detections.
    :param min_ball_area: Minimum color-blob area in pixels.
    :param classes: Optional COCO id allow-list. ``[0]`` is person-only.
    :returns: A running annotator.
    :raises RuntimeError: If ``ultralytics`` is not installed.
    """

    try:
        from ultralytics import YOLO
    except ImportError as exc:
        raise RuntimeError(
            "ultralytics is not installed. On the Pi run: pip3 install ultralytics"
        ) from exc

    print(f"Loading YOLO weights {model_name} (first run downloads them)...")
    model = YOLO(model_name)
    extras = []
    if slices > 1:
        extras.append(f"{slices}x{slices} tiles")
    if ball_color:
        extras.append(f"color blob {ball_color!r} as {ball_class!r}")
    if classes is not None:
        extras.append(f"COCO classes {classes}")
    extra_txt = f" ({', '.join(extras)})" if extras else ""
    print(f"YOLO ready. Inference size {image_size}, confidence {confidence:g}.{extra_txt}")
    return YoloAnnotator(
        model,
        confidence,
        image_size,
        slices=slices,
        ball_color=ball_color,
        ball_class=ball_class,
        min_ball_area=min_ball_area,
        classes=classes,
    )


def load_person_detector(
    model_name: str,
    confidence: float,
    image_size: int,
    slices: int = 1,
) -> YoloAnnotator:
    """YOLOv8n restricted to COCO person (class id 0). Used by chase_person.

    :param model_name: Weights file name or path.
    :param confidence: Minimum detection score.
    :param image_size: Inference size passed to ``predict``.
    :param slices: Tile grid. Person chase defaults to 1 (full frame).
    :returns: A running annotator.
    """

    return load_yolo(
        model_name,
        confidence,
        image_size,
        slices=slices,
        ball_color=None,
        classes=[COCO_PERSON_CLASS_ID],
    )


def load_ball_detector(
    color_name: str,
    min_area: int = 80,
    class_label: str = BALL_CLASS_NAME,
) -> ColorBallAnnotator:
    """HSV + roundness detector. Used by chase_colored_ball. No YOLO.

    :param color_name: Key in ``BALL_HSV_RANGES``.
    :param min_area: Ignore speckles smaller than this many pixels.
    :param class_label: Caption class the chaser locks onto.
    :returns: An annotator that runs inside ``submit``.
    """

    print(
        f"Ball detector: HSV {color_name!r} blobs labelled {class_label!r} "
        f"(min area {min_area} px). YOLO is not loaded."
    )
    return ColorBallAnnotator(color_name, min_area=min_area, class_label=class_label)
