"""
Walk toward one locked instance of ``--class``, not whoever is largest this frame.

Purpose:
    Person chase and ball chase share gait (arc, UART hold) but not
    detection. ``PersonChaser.update`` turns YOLOv8n person boxes into
    walk commands. ``BallChaser.update`` does the same for HSV blobs.
    There is no generic ``--class`` chase. The old person loop had two
    failure modes that look like "the robot is dumb":

    1. It re-picked the largest matching box every camera frame. Two people
       in view meant the dog swapped targets whenever YOLO's boxes wobbled,
       so it "followed everyone" and never committed.
    2. It either turned in place *or* walked straight, never both. A person
       slightly off-center produced a spin (what you read as going sideways)
       even though the box was already on screen. Combined with YOLO lag,
       that bang-bang turn overshot the deadzone forever.

    This file locks onto one box by predicted position, size, overlap, and
    (for people) a cheap color histogram. A person is approached on an
    arc: forward plus a capped yaw. A ball is aligned first, then walked
    to, never both at once. It stops if the lock is lost, rather than
    grabbing the next person in the image center.

Key terms:
    - Lock: The single detection we are willing to walk toward. Once set,
      a bigger person walking through the shot is ignored until this box
      stays missing for ``DEFAULT_LOCK_LOST_S`` seconds.
    - IoU: Intersection over union. How much the new box overlaps the
      locked box, 0 (no overlap) to 1 (identical). Used to answer "is this
      the same person, or a different one?"
    - Center shift: How far a candidate centroid is from where we predicted
      the lock to be, as a fraction of frame width. IoU alone is not enough:
      a huge hallway "person" box can overlap the old pixels after you step
      aside, which is how the populated-room AVI swapped you for a corridor.
    - Appearance / histogram: HSV color histogram of the inset lock crop.
      A blue shirt and a bright hallway score near 0. Used as a veto so a
      high-IoU false box cannot steal the identity.
    - Sticky re-acquire: After the grace period, only lock a box near the
      last predicted heading with similar size/color. Most-centered is a
      *fresh chase* after the original has been gone for
      ``DEFAULT_LOCK_STALE_S``, not a way to stay busy in a crowd.
    - Offset: Horizontal error of the locked box center relative to the
      image center, in ``[-1, 1]``. Negative means the person is on the
      left of the picture, so we yaw left.
    - Deadzone: Person only. Band of offset treated as "straight enough" —
      yaw is zero and we only walk. Narrow on purpose so a modest heading
      error still gets the walk+capped-yaw arc instead of a straight line
      that drifts you off the image. Grows a little when the box is wide,
      because a nearby chest is a noisy heading.
    - Arc: Person only. ``move('x')`` and ``turn()`` in the same command,
      with yaw capped at ``DEFAULT_ARC_YAW_CAP``. The in-between state.
    - Spin offset: Person only. Extreme left/right edge: turn in place so
      we do not walk past them.
    - Align-then-go: Ball only. Turn in place until the blob is centered,
      then walk with yaw zero. Never both. A ball does not walk away while
      we pirouette; a person does, which is why this is not the person law.
    - Align hysteresis: Ball spin continues until offset is inside
      ``DEFAULT_BALL_ALIGN_IN``. Walk continues until offset exceeds
      ``DEFAULT_BALL_ALIGN_OUT``. Same threshold for both directions is
      how a noisy blob chatters start/stop.
    - Stop area: Box area / frame area at which we halt (not true distance).
      A nearby bottle and a far sofa can produce the same fraction.
    - Span: ``max(box_width / frame_width, box_height / frame_height)``.
      Used for *arrival* (you fill the camera). Not used to disable turning.
      A standing person is tall in 720p from several meters away. Treating
      height as "already close, hold heading" made the dog walk a straight
      line while you drifted to the edge of the frame.
    - Width fraction: ``box_width / frame_width``. This is the close-range
      signal for yaw damping. A distant person is tall but skinny. A nearby
      person is wide. We only soften yaw when the box is wide, never because
      it is tall.
    - EMA: Exponential moving average of the box center used for steering.
      New samples are trusted less when the box is *wide* (jitter), not when
      the person is merely tall.
    - Yaw slew: Maximum deg/s the yaw command may change per gait tick.
    - Command hold: Gait commands stay put for ~1/command_hz seconds so a
      walking cycle can finish. Mid-step rewrites are what make the body
      lurch and the camera roll.
    - Turn hold: Offset must stay past the deadzone for
      ``DEFAULT_TURN_HOLD_S`` before we yaw. A single blurry frame is
      camera bounce, not a heading error.
    - Quantize: Walk and yaw snap to coarse steps so 7.3 vs 8.1 mm does
      not restart the gait sequencer.
    - Arc yaw cap: While walking we only allow a few deg/s of turn. Big
      mixed move+turn is how the hallway AVI yawed into a water fountain.

Non-obvious behavior:
    - Lost lock: a one-frame YOLO miss used to call ``stop()`` immediately,
      then the gait-hold blocked walk for another ~0.7 s. That is the
      "stands there for a second or two" in the hallway recording. During
      the grace period we keep walking but slew yaw to 0 so a blink does
      not keep spinning us onto a bystander. After grace we stop and wait
      near the last heading. We still do not search a room.
    - Commands are rate-limited. Spamming UART every camera frame jams
      the gait firmware and shakes the camera.
    - Image-x positive is the right side of the picture. xgolib positive
      ``turn`` is left, so yaw is ``-turn_speed * offset``.
    - A lock is not allowed to jump onto a second "person" box just
      because the old pixels now contain a different box (hallway, door,
      seated stranger). Predicted center, area ratio, and appearance all
      have to agree. IoU is a bonus, not a license to hop.
    - Predicted center = last centroid + box velocity + camera-yaw slide.
      Positive yaw (left) moves objects right in the image.
    - Ball never issues walk and yaw together. Person still does, with a
      narrow deadzone, so a small heading error gets the capped arc.

Exports:
    - pick_target: Most-centered matching box on first acquire; sticky
      near last heading when ``prefer_xy`` is set. Person chase only.
    - ObjectChaser: Shared gait (arc, stop, UART hold).
    - PersonChaser: YOLOv8n person lock with identity persistence.
    - BallChaser: HSV blob lock. Align-then-go gait. Largest blob of that
      color, not a crowd.
"""

from __future__ import annotations

import sys
import time

try:
    import cv2
except ImportError:
    # Histogram matching needs OpenCV. The Pi has it; association still
    # works without color if this file is imported on a laptop for tests.
    cv2 = None

from .helpers import class_name_from_label
from .models import (
    BALL_CLASS_NAME,
    DEFAULT_APPEAR_CORR,
    DEFAULT_ARC_YAW_CAP,
    DEFAULT_BALL_ALIGN_IN,
    DEFAULT_BALL_ALIGN_OUT,
    DEFAULT_CAM_FOV_DEG,
    DEFAULT_DEADZONE_CAP,
    DEFAULT_CENTER_EMA_FAR,
    DEFAULT_CENTER_EMA_NEAR,
    DEFAULT_CHASE_MIN_CONF,
    DEFAULT_CHASE_MIN_HEIGHT,
    DEFAULT_LOCK_AREA_MAX,
    DEFAULT_LOCK_AREA_MIN,
    DEFAULT_LOCK_CENTER_SHIFT,
    DEFAULT_LOCK_IOU,
    DEFAULT_LOCK_LOST_S,
    DEFAULT_LOCK_MAX_SHIFT,
    DEFAULT_LOCK_STALE_S,
    DEFAULT_REACQUIRE_SHIFT,
    DEFAULT_SPIN_OFFSET,
    DEFAULT_STOP_SPAN,
    DEFAULT_TURN_HOLD_S,
    DEFAULT_WALK_QUANT,
    DEFAULT_WIDE_BOX,
    DEFAULT_YAW_QUANT,
    DEFAULT_YAW_SLEW,
    Detection,
)
from .telemetry import XgoTelemetryPoller


def _box_area(detection: Detection) -> int:
    """Pixel area of a detection box.

    :param detection: ``(x1, y1, x2, y2, label, color)``.
    :returns: Non-negative width * height.
    """

    x1, y1, x2, y2 = detection[:4]
    return max(0, x2 - x1) * max(0, y2 - y1)


def _box_center(detection: Detection) -> tuple[float, float]:
    """Center pixel of a detection box.

    :param detection: Box to measure.
    :returns: ``(center_x, center_y)``.
    """

    x1, y1, x2, y2 = detection[:4]
    return ((x1 + x2) / 2.0, (y1 + y2) / 2.0)


def _proximity(
    detection: Detection, frame_width: int, frame_height: int
) -> tuple[float, float]:
    """How much of the frame this box occupies.

    :param detection: Box in display pixels.
    :param frame_width: Image width.
    :param frame_height: Image height.
    :returns: ``(coverage, span)`` where coverage is area fraction and span
        is the larger of width and height fractions.
    """

    x1, y1, x2, y2 = detection[:4]
    box_w = max(0, x2 - x1)
    box_h = max(0, y2 - y1)
    coverage = (box_w * box_h) / max(frame_width * frame_height, 1)
    span = max(box_w / max(frame_width, 1), box_h / max(frame_height, 1))
    return coverage, span


def _quantize(value: float, step: float) -> float:
    """Snap a gait value onto a coarse grid so tiny noise does not restart a step.

    :param value: Walk mm or yaw deg/s.
    :param step: Grid size. Values inside half a step become 0.
    :returns: Quantized value.
    """

    if step <= 0:
        return value
    if abs(value) < step * 0.5:
        return 0.0
    return round(value / step) * step


def _box_iou(a: Detection, b: Detection) -> float:
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
    union = _box_area(a) + _box_area(b) - inter
    return inter / union if union > 0 else 0.0


def _class_detections(detections: list[Detection], target_class: str) -> list[Detection]:
    """Keep boxes whose caption class matches ``target_class``.

    :param detections: All current boxes.
    :param target_class: Caption class (``person`` or ``ball``).
    :returns: Matching detections, possibly empty.
    """

    wanted = target_class.strip().lower()
    return [det for det in detections if class_name_from_label(det[4]) == wanted]


def _score_from_label(label: str) -> float:
    """Read the trailing confidence in a caption like ``person 0.81``.

    :param label: Detection caption drawn on the frame.
    :returns: Score, or 0 if the caption has no number.
    """

    try:
        return float(label.rsplit(" ", 1)[-1])
    except ValueError:
        return 0.0


def _is_usable(
    detection: Detection,
    frame_width: int,
    frame_height: int,
    min_conf: float = DEFAULT_CHASE_MIN_CONF,
    min_height: float = DEFAULT_CHASE_MIN_HEIGHT,
) -> bool:
    """Reject weak / squat boxes that YOLO labels ``person`` on furniture.

    :param detection: Candidate box.
    :param frame_width: Image width in pixels.
    :param frame_height: Image height in pixels.
    :param min_conf: Lowest caption score we will chase.
    :param min_height: Lowest box-height / frame-height we will chase.
    :returns: True if this box is allowed to be a lock or re-acquire.
    """

    del frame_width
    if _score_from_label(detection[4]) < min_conf:
        return False
    x1, y1, x2, y2 = detection[:4]
    box_w = max(1, x2 - x1)
    box_h = max(1, y2 - y1)
    height_frac = box_h / max(frame_height, 1)
    if height_frac < min_height:
        return False
    # Distant false "person" boxes on doors and chairs are squat. A nearby
    # person filling the lens can be wider than they are tall, so skip this
    # check once the box is already huge.
    if height_frac < 0.45 and (box_h / box_w) < 1.12:
        return False
    return True


def _usable_matches(
    detections: list[Detection],
    target_class: str,
    frame_width: int,
    frame_height: int,
) -> list[Detection]:
    """Class filter plus the squat / low-confidence veto.

    :param detections: All current boxes.
    :param target_class: Caption class (``person`` or ``ball``).
    :param frame_width: Image width.
    :param frame_height: Image height.
    :returns: Boxes we are willing to lock onto.
    """

    return [
        det
        for det in _class_detections(detections, target_class)
        if _is_usable(det, frame_width, frame_height)
    ]


def _box_hist(frame: object, detection: Detection):
    """HSV histogram of the inset box. Background on the rim is ignored.

    :param frame: BGR image the box was drawn on.
    :param detection: Pixel box on that image.
    :returns: Normalized 16x16 H-S histogram, or None if the crop is empty.
    """

    if frame is None or cv2 is None:
        return None
    x1, y1, x2, y2 = detection[:4]
    height, width = int(frame.shape[0]), int(frame.shape[1])
    box_w = max(1, x2 - x1)
    box_h = max(1, y2 - y1)
    # Inset so the rectangle edge (floor, wall) does not dominate the color.
    ix1 = max(0, int(x1 + 0.12 * box_w))
    iy1 = max(0, int(y1 + 0.12 * box_h))
    ix2 = min(width, int(x2 - 0.12 * box_w))
    iy2 = min(height, int(y2 - 0.12 * box_h))
    if ix2 <= ix1 or iy2 <= iy1:
        return None
    crop = frame[iy1:iy2, ix1:ix2]
    if crop is None or crop.size == 0:
        return None
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    hist = cv2.calcHist([hsv], [0, 1], None, [16, 16], [0, 180, 0, 256])
    cv2.normalize(hist, hist, 0, 1, cv2.NORM_MINMAX)
    return hist


def _hist_corr(a, b) -> float:
    """Correlation of two HSV histograms in ``[-1, 1]``.

    :param a: First histogram, or None.
    :param b: Second histogram, or None.
    :returns: 1 if either hist is missing (do not veto), else OpenCV correlation.
    """

    if a is None or b is None or cv2 is None:
        return 1.0
    return float(cv2.compareHist(a, b, cv2.HISTCMP_CORREL))


def pick_target(
    detections: list[Detection],
    target_class: str,
    frame_width: int | None = None,
    frame_height: int | None = None,
    prefer_xy: tuple[float, float] | None = None,
    prefer_area: int | None = None,
    prefer_hist=None,
    frame: object | None = None,
) -> Detection | None:
    """Choose one matching box to acquire when we are not already locked.

    With no hint, prefers the box closest to the image center. The old
    "largest box wins" rule followed whoever stood nearest the lens —
    often a bystander at the edge — while the person you faced was ignored.

    With ``prefer_xy``, only a nearby similar-looking box wins. Returning
    None here is deliberate: the caller must not silently fall back to
    "whoever is in the middle of the room."

    :param detections: Boxes already scaled to the display frame.
    :param target_class: Caption class (``person`` or ``ball``).
    :param frame_width: Image width used to score centrality / shift.
    :param frame_height: Image height used to reject squat boxes.
    :param prefer_xy: Last predicted lock center. Sticky re-acquire only.
    :param prefer_area: Last lock area in pixels. Reject huge size jumps.
    :param prefer_hist: Last lock HSV histogram. Reject hallway steals.
    :param frame: BGR image, used only when ``prefer_hist`` is set.
    :returns: One detection tuple, or None if nothing matched.
    """

    width = frame_width if frame_width is not None else 0
    height = frame_height if frame_height is not None else 0
    if width > 0 and height > 0:
        matches = _usable_matches(detections, target_class, width, height)
    else:
        matches = _class_detections(detections, target_class)
    if not matches:
        return None
    if prefer_xy is not None and width > 0:
        px, py = prefer_xy
        nearby: list[Detection] = []
        for candidate in matches:
            cx, cy = _box_center(candidate)
            shift = (((cx - px) ** 2 + (cy - py) ** 2) ** 0.5) / max(width, 1)
            if shift > DEFAULT_REACQUIRE_SHIFT:
                continue
            if prefer_area is not None and prefer_area > 0:
                ratio = _box_area(candidate) / prefer_area
                if ratio < DEFAULT_LOCK_AREA_MIN or ratio > DEFAULT_LOCK_AREA_MAX:
                    continue
            if prefer_hist is not None:
                corr = _hist_corr(prefer_hist, _box_hist(frame, candidate))
                if corr < DEFAULT_APPEAR_CORR:
                    continue
            nearby.append(candidate)
        if not nearby:
            return None
        return min(
            nearby,
            key=lambda det: (
                ((_box_center(det)[0] - px) ** 2 + (_box_center(det)[1] - py) ** 2) ** 0.5
            ),
        )
    if width <= 0:
        return max(matches, key=_box_area)

    def _center_key(detection: Detection) -> tuple[float, int]:
        """Sort by distance from mid-frame, then by larger area.

        :param detection: Candidate box.
        :returns: Lower is better.
        """

        center_x, _center_y = _box_center(detection)
        offset = abs(center_x - (width / 2.0)) / (width / 2.0)
        return (offset, -_box_area(detection))

    return min(matches, key=_center_key)


class ObjectChaser:
    """Shared gait: lock a box, steer toward it, stop when close or lost.

    Person chase arcs (walk plus capped yaw when slightly off-center).
    Ball chase turns in place until centered, then walks straight.

    Do not construct this for a random COCO class. Use ``PersonChaser`` or
    ``BallChaser``. The two detectors fail differently, so association is
    not one generic ``--class`` loop.

    :param target_class: Caption class to follow (``person`` or ``ball``).
    :param walk_speed: Forward step size in mm.
    :param turn_speed: Peak yaw rate in deg/s (positive is left).
    :param center_deadzone: Fraction of half-width treated as "centered."
    :param stop_area: Box-area / frame-area at which we halt.
    :param command_hz: Maximum gait commands per second.
    :param telemetry: Shared serial owner, or None.
    :param dog: Direct ``XGO`` handle used when telemetry is off.
    :param identity: ``person`` (crowd lock + appearance) or ``ball``
      (largest blob of that color, no clothing histogram).
    """

    def __init__(
        self,
        target_class: str,
        walk_speed: float,
        turn_speed: float,
        center_deadzone: float,
        stop_area: float,
        command_hz: float,
        telemetry: XgoTelemetryPoller | None = None,
        dog: object | None = None,
        identity: str = "person",
    ) -> None:
        self._target_class = target_class.strip().lower()
        self._identity = identity.strip().lower()
        self._walk_speed = walk_speed
        self._turn_speed = turn_speed
        self._deadzone = center_deadzone
        self._stop_area = stop_area
        self._min_interval = 1.0 / max(command_hz, 0.5)
        self._telemetry = telemetry
        self._dog = dog
        self._last_command: tuple[float, float] | None = None
        self._last_sent = 0.0
        self._last_yaw = 0.0
        self._locked: Detection | None = None
        self._lost_at: float | None = None
        self._holding_close = False
        self._smooth_cx: float | None = None
        self._last_steer_log = 0.0
        self._offset_since: float | None = None
        self._ball_aligning = False
        self._lock_hist = None
        self._vel_x = 0.0
        self._lock_seen_at: float | None = None
        self._waiting_reacquire = False

    def update(
        self,
        detections: list[Detection],
        frame_width: int,
        frame_height: int,
        frame: object | None = None,
    ) -> Detection | None:
        """Steer toward the locked instance, or stop if there is none.

        :param detections: Current boxes in display coordinates.
        :param frame_width: Width of the image those boxes sit on.
        :param frame_height: Height of that image.
        :param frame: BGR image used for the lock color histogram. Optional.
        :returns: The box being chased, or None if idle.
        """

        if frame_width <= 0 or frame_height <= 0:
            self._send(0.0, 0.0)
            return None

        target = self._associate(
            self._current_matches(detections, frame_width, frame_height),
            frame_width,
            frame_height,
            frame,
        )
        if target is None:
            # YOLO blinks. Cutting the motors here is the 1–2 s freeze in
            # the hallway AVI: stop() plus the gait-hold before walk resumes.
            # Grace: keep walking so a one-frame miss does not halt, but slew
            # yaw to 0 so leftover turn does not spin us onto a bystander.
            # After grace we stop and wait rather than chasing the person
            # now sitting in the middle of the frame.
            if self._locked is None or self._waiting_reacquire:
                self._send(0.0, 0.0)
            else:
                last = self._last_command or (0.0, 0.0)
                self._send(last[0], self._slew_yaw(0.0))
            return None

        coverage, span = _proximity(target, frame_width, frame_height)
        # Hysteresis so we do not twitch stop/go at the stop-area boundary.
        arrived = coverage >= self._stop_area or span >= DEFAULT_STOP_SPAN
        still_close = coverage >= self._stop_area * 0.72 or span >= DEFAULT_STOP_SPAN * 0.88
        if arrived or (self._holding_close and still_close):
            if not self._holding_close:
                print(
                    f"Chase arrived (coverage={coverage:.2f} span={span:.2f}); "
                    "holding heading and stopping."
                )
            self._holding_close = True
            self._last_yaw = 0.0
            self._send(0.0, 0.0)
            return target
        self._holding_close = False

        walk, yaw = self._arc_command(target, frame_width, frame_height)
        self._send(walk, yaw)
        return target

    def stop(self) -> None:
        """Halt the gait. Call from ``finally`` so Ctrl+C does not leave it walking.

        :returns: None.
        """

        self._reset_track()
        self._issue(0.0, 0.0)

    def _current_matches(
        self, detections: list[Detection], frame_width: int, frame_height: int
    ) -> list[Detection]:
        """Boxes this identity is allowed to walk toward.

        :param detections: All current boxes.
        :param frame_width: Image width.
        :param frame_height: Image height.
        :returns: Person-shaped person boxes, or every ball blob.
        """

        # A ping-pong blob is small and round. The person squat/height veto
        # would throw it away, which is why ball chase does not share that filter.
        if self._identity == "ball":
            return _class_detections(detections, self._target_class)
        return _usable_matches(detections, self._target_class, frame_width, frame_height)

    def _associate(
        self,
        matches: list[Detection],
        frame_width: int,
        frame_height: int,
        frame: object | None,
    ) -> Detection | None:
        """Keep the locked person, or acquire a new one without hopping.

        :param matches: Current usable boxes of the target class.
        :param frame_width: Image width for center-distance matching.
        :param frame_height: Image height (needed for a fresh acquire).
        :param frame: BGR image for appearance, or None.
        :returns: The locked box, or None while waiting / idle.
        """

        now = time.time()
        if self._locked is None:
            acquired = self._acquire_new(matches, frame_width, frame_height)
            if acquired is None:
                return None
            self._commit_lock(acquired, now, frame, frame_width, is_acquire=True)
            return acquired

        predicted_cx, predicted_cy = self._predicted_center(frame_width, now)
        best = self._best_continuation(
            matches, frame_width, frame, predicted_cx, predicted_cy
        )
        if best is not None:
            self._commit_lock(best, now, frame, frame_width, is_acquire=False)
            return best

        # Same person missing this snapshot. Do not steal a different person.
        if self._lost_at is None:
            self._lost_at = now
            print(
                f"Chase lock lost sight of {self._target_class!r}; "
                "holding walk and cancelling yaw instead of switching targets."
            )
        if (now - self._lost_at) < DEFAULT_LOCK_LOST_S:
            return None

        # Grace over. Person chase: only a box near the last heading with
        # similar size and color is the same human. Most-centered is how
        # the AVI walked up to the stranger by the window. Ball chase:
        # there is no crowd; take the largest remaining blob of this color.
        if self._identity == "ball":
            if matches:
                picked = max(matches, key=_box_area)
                print(f"Chase re-locked the largest {self._target_class!r} blob.")
                self._commit_lock(picked, now, frame, frame_width, is_acquire=False)
                return picked
            self._waiting_reacquire = True
            return None

        sticky = pick_target(
            matches,
            self._target_class,
            frame_width=frame_width,
            frame_height=frame_height,
            prefer_xy=(predicted_cx, predicted_cy),
            prefer_area=_box_area(self._locked),
            prefer_hist=self._lock_hist,
            frame=frame,
        )
        if sticky is not None:
            print(
                f"Chase re-locked the same {self._target_class!r} near the last heading."
            )
            self._commit_lock(sticky, now, frame, frame_width, is_acquire=False)
            return sticky

        seen_at = self._lock_seen_at if self._lock_seen_at is not None else self._lost_at
        if (now - seen_at) < DEFAULT_LOCK_STALE_S:
            if not self._waiting_reacquire:
                print(
                    f"Chase lock expired; waiting near last heading instead of "
                    f"grabbing another {self._target_class!r}."
                )
            self._waiting_reacquire = True
            return None

        print(
            f"Chase lock stale. Re-acquiring the most-centered {self._target_class!r}."
        )
        self._clear_identity()
        return self._associate(matches, frame_width, frame_height, frame)

    def _acquire_new(
        self,
        matches: list[Detection],
        frame_width: int,
        frame_height: int,
    ) -> Detection | None:
        """First lock: face-center for a person, largest blob for a ball.

        :param matches: Already-filtered boxes of the target class.
        :param frame_width: Image width.
        :param frame_height: Image height.
        :returns: The box to walk toward, or None.
        """

        if not matches:
            return None
        if self._identity == "ball":
            return max(matches, key=_box_area)
        return pick_target(
            matches,
            self._target_class,
            frame_width=frame_width,
            frame_height=frame_height,
        )

    def _best_continuation(
        self,
        matches: list[Detection],
        frame_width: int,
        frame: object | None,
        predicted_cx: float,
        predicted_cy: float,
    ) -> Detection | None:
        """Score current boxes against the predicted lock. Center distance wins.

        IoU is a bonus, not a license to hop. A hallway-sized "person" box
        that overlaps the old pixels after you step aside used to beat the
        real person at the edge because score was ``2 + iou``.

        :param matches: Usable boxes this frame.
        :param frame_width: Image width for normalizing shift.
        :param frame: BGR image for appearance, or None.
        :param predicted_cx: Where the lock center should be, in pixels.
        :param predicted_cy: Matching y, in pixels.
        :returns: Best continuation, or None.
        """

        locked_area = max(_box_area(self._locked), 1)
        turning = abs(self._last_yaw) >= 1.0
        max_shift = DEFAULT_LOCK_MAX_SHIFT + (0.08 if turning else 0.0)
        best: Detection | None = None
        best_score = -1.0
        for candidate in matches:
            cand_cx, cand_cy = _box_center(candidate)
            shift = (
                ((cand_cx - predicted_cx) ** 2 + (cand_cy - predicted_cy) ** 2) ** 0.5
            ) / max(frame_width, 1)
            if shift > max_shift:
                continue
            area_ratio = _box_area(candidate) / locked_area
            if area_ratio < DEFAULT_LOCK_AREA_MIN or area_ratio > DEFAULT_LOCK_AREA_MAX:
                continue
            # Clothing color is how we refuse a hallway "person" box.
            # Ball blobs are already one HSV color; skip the veto.
            corr = 1.0
            if self._identity == "person":
                corr = _hist_corr(self._lock_hist, _box_hist(frame, candidate))
                if corr < DEFAULT_APPEAR_CORR:
                    continue
            iou = _box_iou(self._locked, candidate)
            # Near the prediction is the identity test. IoU and a tiny
            # center-only fallback used to let a different box inherit
            # the lock just by occupying the old pixels.
            if iou < DEFAULT_LOCK_IOU and shift > DEFAULT_LOCK_CENTER_SHIFT:
                continue
            score = (1.0 - shift) + 0.25 * iou + 0.20 * max(corr, 0.0)
            if score > best_score:
                best = candidate
                best_score = score
        return best

    def _predicted_center(self, frame_width: int, now: float) -> tuple[float, float]:
        """Where the locked centroid should sit, given velocity and yaw.

        :param frame_width: Image width in pixels.
        :param now: ``time.time()`` for this update.
        :returns: ``(predicted_x, predicted_y)`` in pixels.
        """

        cx, cy = _box_center(self._locked)
        dt = 0.0
        if self._lock_seen_at is not None:
            dt = max(0.0, min(0.5, now - self._lock_seen_at))
        cx += self._vel_x * dt
        # Positive xgolib yaw is left, which slides scene content right.
        if abs(self._last_yaw) > 0.5 and frame_width > 0:
            px_per_deg = frame_width / max(DEFAULT_CAM_FOV_DEG, 1.0)
            cx += self._last_yaw * dt * px_per_deg
        return cx, cy

    def _commit_lock(
        self,
        detection: Detection,
        now: float,
        frame: object | None,
        frame_width: int,
        is_acquire: bool,
    ) -> None:
        """Store the matched box, its color, and its pixel velocity.

        :param detection: Box we are now willing to walk toward.
        :param now: ``time.time()`` for this update.
        :param frame: BGR image for the histogram, or None.
        :param frame_width: Unused; kept so call sites stay obvious.
        :param is_acquire: True on a fresh lock (reset velocity / announce).
        :returns: None.
        """

        del frame_width
        new_cx, _new_cy = _box_center(detection)
        if not is_acquire and self._locked is not None and self._lock_seen_at is not None:
            dt = now - self._lock_seen_at
            if dt > 0.02:
                old_cx, _old_cy = _box_center(self._locked)
                instant = (new_cx - old_cx) / dt
                self._vel_x = 0.55 * self._vel_x + 0.45 * instant
        else:
            self._vel_x = 0.0
        if self._identity == "person":
            hist = _box_hist(frame, detection)
            if hist is not None:
                self._lock_hist = hist
        self._locked = detection
        self._lost_at = None
        self._waiting_reacquire = False
        self._lock_seen_at = now
        self._smooth_cx = new_cx if is_acquire or self._smooth_cx is None else self._smooth_cx
        if is_acquire:
            print(
                f"Chase locked on {detection[4]!r} at x={new_cx:.0f} "
                f"(ignoring other {self._target_class!r} boxes until this one is lost)."
            )

    def _clear_identity(self) -> None:
        """Drop the lock so the next update may start a fresh chase.

        :returns: None.
        """

        self._locked = None
        self._lost_at = None
        self._holding_close = False
        self._smooth_cx = None
        self._last_yaw = 0.0
        self._offset_since = None
        self._ball_aligning = False
        self._lock_hist = None
        self._vel_x = 0.0
        self._lock_seen_at = None
        self._waiting_reacquire = False

    def _reset_track(self) -> None:
        """Clear lock, smoother, and close-range flags.

        :returns: None.
        """

        self._last_command = None
        self._clear_identity()

    def _smoothed_center_x(self, target: Detection, width_frac: float) -> float:
        """Low-pass the box center so YOLO jitter does not reach the motors.

        :param target: Locked box in display pixels.
        :param width_frac: Box width / frame width. Wide = close and noisy.
        :returns: Smoothed center x in pixels.
        """

        raw_x, _raw_y = _box_center(target)
        if self._smooth_cx is None:
            self._smooth_cx = raw_x
            return raw_x
        # Follow a skinny (distant) box quickly so you cannot drift off-screen
        # while the smoother still thinks you are centered.
        alpha = DEFAULT_CENTER_EMA_NEAR if width_frac >= DEFAULT_WIDE_BOX else DEFAULT_CENTER_EMA_FAR
        self._smooth_cx += alpha * (raw_x - self._smooth_cx)
        return self._smooth_cx

    def _slew_yaw(self, yaw: float) -> float:
        """Clamp how hard yaw may reverse between gait ticks.

        :param yaw: Desired deg/s.
        :returns: Deg/s after the slew limit.
        """

        delta = yaw - self._last_yaw
        if delta > DEFAULT_YAW_SLEW:
            yaw = self._last_yaw + DEFAULT_YAW_SLEW
        elif delta < -DEFAULT_YAW_SLEW:
            yaw = self._last_yaw - DEFAULT_YAW_SLEW
        self._last_yaw = yaw
        return yaw

    def _arc_command(
        self, target: Detection, frame_width: int, frame_height: int
    ) -> tuple[float, float]:
        """Map box position to a walk/yaw pair. Person arcs; ball aligns then goes.

        :param target: Locked box in display pixels.
        :param frame_width: Image width those pixels live in.
        :param frame_height: Image height those pixels live in.
        :returns: ``(walk_mm, yaw_deg_s)``. Positive yaw is left.
        """

        _coverage, span = _proximity(target, frame_width, frame_height)
        x1, _y1, x2, _y2, _label, _color = target
        width_frac = max(0, x2 - x1) / max(frame_width, 1)
        center_x = self._smoothed_center_x(target, width_frac)
        offset = (center_x - (frame_width / 2.0)) / (frame_width / 2.0)
        offset = max(-1.0, min(1.0, offset))
        walk = self._walk_speed * max(0.45, 1.0 - 0.5 * span)
        if self._identity == "ball":
            return self._ball_align_command(offset, walk)

        # Person hybrid. Narrow deadzone so a modest heading error still
        # gets walk + capped yaw instead of a straight line that drifts.
        deadzone = min(DEFAULT_DEADZONE_CAP, self._deadzone + 0.08 * width_frac)
        now = time.time()

        if abs(offset) <= deadzone:
            self._offset_since = None
            return (walk, self._slew_yaw(0.0))

        if self._offset_since is None:
            self._offset_since = now
        # Stride bounce is short. A real heading error lasts longer.
        if now - self._offset_since < DEFAULT_TURN_HOLD_S:
            return (walk, self._slew_yaw(0.0))

        gain = max(0.40, 1.0 - 0.7 * width_frac)
        desired = -self._turn_speed * offset * gain
        if abs(offset) < DEFAULT_SPIN_OFFSET:
            # Walking + large yaw is the lurch. Cap the arc before slew.
            if desired > DEFAULT_ARC_YAW_CAP:
                desired = DEFAULT_ARC_YAW_CAP
            elif desired < -DEFAULT_ARC_YAW_CAP:
                desired = -DEFAULT_ARC_YAW_CAP
        yaw = self._slew_yaw(desired)
        if now - self._last_steer_log >= 1.0:
            print(
                f"Chase steer offset={offset:+.2f} yaw={yaw:+.0f} deg/s "
                f"walk={walk:.0f} width={width_frac:.2f}"
            )
            self._last_steer_log = now
        if abs(offset) >= DEFAULT_SPIN_OFFSET and width_frac < 0.28:
            return (0.0, yaw)
        return (walk, yaw)

    def _ball_align_command(self, offset: float, walk: float) -> tuple[float, float]:
        """Turn until the ball is centered, then walk. Never both.

        A ball does not walk out of frame while we spin, so exclusive
        align-then-go is safe here. Hysteresis (IN vs OUT) stops a noisy
        blob from flipping the gait every frame at one threshold.

        :param offset: Horizontal error in ``[-1, 1]``. Negative is left.
        :param walk: Forward mm to use once aligned.
        :returns: ``(walk_mm, yaw_deg_s)``. One of them is always ~0.
        """

        abs_off = abs(offset)
        if self._ball_aligning:
            if abs_off <= DEFAULT_BALL_ALIGN_IN:
                self._ball_aligning = False
        elif abs_off >= DEFAULT_BALL_ALIGN_OUT:
            self._ball_aligning = True

        now = time.time()
        if self._ball_aligning:
            yaw = self._slew_yaw(-self._turn_speed * offset)
            if now - self._last_steer_log >= 1.0:
                print(f"Ball align offset={offset:+.2f} yaw={yaw:+.0f} deg/s (not walking)")
                self._last_steer_log = now
            return (0.0, yaw)

        if now - self._last_steer_log >= 1.0:
            print(f"Ball go offset={offset:+.2f} walk={walk:.0f} (yaw held at 0)")
            self._last_steer_log = now
        return (walk, self._slew_yaw(0.0))

    def _send(self, walk: float, yaw: float) -> None:
        """Send a gait pair if it is new or the rate limit has elapsed.

        :param walk: Forward mm for ``move('x')``. 0 means no forward.
        :param yaw: Deg/s for ``turn()``. 0 means no yaw.
        :returns: None.
        """

        now = time.time()
        walk = _quantize(walk, DEFAULT_WALK_QUANT)
        yaw = _quantize(yaw, DEFAULT_YAW_QUANT)
        command = (walk, yaw)
        if command == self._last_command:
            return
        stopping = abs(walk) < 0.5 and abs(yaw) < 0.5
        last = self._last_command
        last_was_stop = last is not None and abs(last[0]) < 0.5 and abs(last[1]) < 0.5
        # Hold a walking/turning command so a step can finish. Do not apply
        # that hold when leaving stop — that delay stacked with lock-loss
        # into a second of standing still while the person was still in view.
        if (
            not stopping
            and not last_was_stop
            and last is not None
            and (now - self._last_sent) < self._min_interval
        ):
            return
        self._issue(walk, yaw)
        self._last_command = command
        self._last_sent = now

    def _issue(self, walk: float, yaw: float) -> None:
        """Talk to the motion board once.

        :param walk: Forward mm.
        :param yaw: Deg/s, positive left.
        :returns: None.
        """

        if self._telemetry is not None:
            self._telemetry.command(lambda dog: self._apply(dog, walk, yaw))
            return
        if self._dog is not None:
            self._apply(self._dog, walk, yaw)

    def _apply(self, dog: object, walk: float, yaw: float) -> None:
        """Write both gait channels. Leaving one stale is a spin or a drift.

        :param dog: Live ``XGO`` instance.
        :param walk: Forward mm.
        :param yaw: Deg/s, positive left.
        :returns: None.
        """

        try:
            if abs(walk) < 0.5 and abs(yaw) < 0.5:
                dog.stop()
                return
            dog.move("x", walk)
            dog.turn(yaw)
        except Exception as exc:
            print(f"Walk command walk={walk:g} yaw={yaw:g} failed: {exc}", file=sys.stderr)


class PersonChaser(ObjectChaser):
    """Walk toward one person using YOLOv8n boxes.

    Identity lock, appearance, and "do not grab the bystander" live here.
    Constructed by ``chase_person`` / ``main.py chase-person``.

    :param walk_speed: Forward step size in mm.
    :param turn_speed: Peak yaw rate in deg/s (positive is left).
    :param center_deadzone: Fraction of half-width treated as "centered."
    :param stop_area: Box-area / frame-area at which we halt.
    :param command_hz: Maximum gait commands per second.
    :param telemetry: Shared serial owner, or None.
    :param dog: Direct ``XGO`` handle used when telemetry is off.
    """

    def __init__(
        self,
        walk_speed: float,
        turn_speed: float,
        center_deadzone: float,
        stop_area: float,
        command_hz: float,
        telemetry: XgoTelemetryPoller | None = None,
        dog: object | None = None,
    ) -> None:
        super().__init__(
            target_class="person",
            walk_speed=walk_speed,
            turn_speed=turn_speed,
            center_deadzone=center_deadzone,
            stop_area=stop_area,
            command_hz=command_hz,
            telemetry=telemetry,
            dog=dog,
            identity="person",
        )


class BallChaser(ObjectChaser):
    """Walk toward the largest HSV blob labelled ``ball``.

    No YOLO, no crowd identity. Steering is align-then-go: spin until the
    blob is centered, then walk straight. If two orange things exist, the
    bigger round blob wins on acquire and we follow that instance by overlap.

    :param walk_speed: Forward step size in mm.
    :param turn_speed: Peak yaw rate in deg/s (positive is left).
    :param center_deadzone: Fraction of half-width treated as "centered."
    :param stop_area: Box-area / frame-area at which we halt.
    :param command_hz: Maximum gait commands per second.
    :param telemetry: Shared serial owner, or None.
    :param dog: Direct ``XGO`` handle used when telemetry is off.
    """

    def __init__(
        self,
        walk_speed: float,
        turn_speed: float,
        center_deadzone: float,
        stop_area: float,
        command_hz: float,
        telemetry: XgoTelemetryPoller | None = None,
        dog: object | None = None,
    ) -> None:
        super().__init__(
            target_class=BALL_CLASS_NAME,
            walk_speed=walk_speed,
            turn_speed=turn_speed,
            center_deadzone=center_deadzone,
            stop_area=stop_area,
            command_hz=command_hz,
            telemetry=telemetry,
            dog=dog,
            identity="ball",
        )
