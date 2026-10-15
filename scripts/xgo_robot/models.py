"""
Shared types, constants, and the live-page HTML for the XGO homework package.

Purpose:
    Other modules should not invent their own camera size, YOLO defaults, or
    servo names. This file is the single list of those values. If you change
    the default live-page size, you change it here and both ``stream`` and
    ``chase`` pick it up.

Key terms:
    - Detection: A tuple ``(x1, y1, x2, y2, label, bgr_color)``. Pixel
      corners of one box, a caption like ``sports ball 0.81``, and the
      color used to draw it. Not a class with methods; a plain tuple so
      OpenCV code stays boring.
    - HSV range: Two 3-tuples ``(H, S, V)`` that mark a color in OpenCV's
      hue-saturation-value space. Red wraps around hue 0, so it needs two
      ranges. These are presets for a solid-colored ball YOLO will miss.
    - Servo id: XGO numbers joints as ``leg, joint``. 13 is right-front
      shoulder. The live stats panel uses the human names in SERVO_LABELS.
    - PAGE_HTML: The browser page with the video and the top-right stats
      overlay. Served at ``/``. The raw JPEG sequence is ``/stream``.
    - SCRIPT_DIR: This package folder. AVI files are written next to these
      Python files on the Pi, not wherever you happened to ``cd``.

Non-obvious behavior:
    - JPEG_QUALITY 80 is a Tailscale bandwidth compromise, not "best looking."
    - MJPEG_BOUNDARY must stay ``frame``; the HTTP multipart header uses it.

Exports:
    - Detection, all DEFAULT_* constants, SERVO_IDS, SERVO_LABELS,
      BALL_HSV_RANGES, PAGE_HTML, SCRIPT_DIR, RECORD_FOURCC, JPEG_QUALITY,
      MJPEG_BOUNDARY, QUIT_KEY, chase gait defaults, TARGET_BOX_COLOR.
"""

from __future__ import annotations

from pathlib import Path

# One finished detection: pixel box plus caption plus draw color.
Detection = tuple[int, int, int, int, str, tuple[int, int, int]]

# 1280x720 is four times wider and three times taller than the old 320x240 feed.
# YOLOv8n still infers at 640 so the Pi is not detecting on every full-size pixel.
DEFAULT_SOURCE = "0"
DEFAULT_WIDTH = 1280
DEFAULT_HEIGHT = 720
DEFAULT_FPS = 20.0
DEFAULT_PORT = 8080
DEFAULT_YOLO_MODEL = "yolov8n.pt"
DEFAULT_YOLO_IMGSZ = 640
DEFAULT_YOLO_CONF = 0.25
# COCO id 0 is person. chase_person passes this so chairs/TVs never become boxes.
COCO_PERSON_CLASS_ID = 0
# Caption class written on HSV blobs. chase_colored_ball locks this name, not
# YOLO's "sports ball" (that class is a basketball, not a ping-pong ball).
BALL_CLASS_NAME = "ball"
DEFAULT_XGO_PORT = "/dev/ttyAMA0"
DEFAULT_TELEMETRY_HZ = 4.0
QUIT_KEY = "q"

# Conservative gait. Wiki limits are x:[-25,25] mm and turn:[-150,150] deg/s.
# The camera is bolted to the body. Rewriting move()+turn() at 4 Hz with
# 40 deg/s yaw makes the chassis lurch, the image rolls, YOLO jumps, and
# the next command yaws harder — that is the hallway AVI (lock on a door,
# then a fountain). Slow steps, small yaw, hold each command ~one gait cycle.
DEFAULT_WALK_SPEED = 8.0
DEFAULT_TURN_SPEED = 16.0
# Person hybrid: |offset| inside this band is "straight enough, walk only."
# 0.26 treated most of the frame as centered, so the dog rarely used the
# walk+capped-yaw arc. 0.12 means a modest heading error still steers.
DEFAULT_CENTER_DEADZONE = 0.12
# Close-range extra deadzone used to hit 0.34 and cancel steering. Cap it.
DEFAULT_DEADZONE_CAP = 0.18
DEFAULT_STOP_AREA = 0.38
DEFAULT_COMMAND_HZ = 1.5
DEFAULT_ARC_YAW_CAP = 8.0
DEFAULT_YAW_QUANT = 4.0
DEFAULT_WALK_QUANT = 2.0
DEFAULT_TURN_HOLD_S = 0.45
# Stick to one instance. Re-picking "largest person" every frame is how the
# dog ping-pongs between two people in the same shot.
DEFAULT_LOCK_IOU = 0.25
DEFAULT_LOCK_CENTER_SHIFT = 0.10
DEFAULT_LOCK_LOST_S = 0.8
# Populated-room AVI (20261009_210509): IoU-only matching stole the lock
# onto a hallway "person" box that overlapped the old pixels after you
# stepped aside. Center jump and area jump both have to stay sane.
DEFAULT_LOCK_MAX_SHIFT = 0.30
DEFAULT_LOCK_AREA_MIN = 0.50
DEFAULT_LOCK_AREA_MAX = 2.0
# After the grace period, re-acquire near the last heading — not whoever
# happens to sit in the image center (that is the bystander by the window).
DEFAULT_REACQUIRE_SHIFT = 0.40
DEFAULT_LOCK_STALE_S = 2.5
# Weak boxes in that recording were furniture, doors, and seated strangers
# (person 0.26–0.35). The person being chased scored 0.53+.
DEFAULT_CHASE_MIN_CONF = 0.45
DEFAULT_CHASE_MIN_HEIGHT = 0.16
# HSV-histogram correlation in [-1, 1]. Blue shirt vs bright hallway is
# near 0. Same person across a step is usually well above this floor.
DEFAULT_APPEAR_CORR = 0.22
# Horizontal field of view used to slide the predicted lock while we yaw.
DEFAULT_CAM_FOV_DEG = 70.0
# |offset| above this: person spin in place. Below it (and outside the
# deadzone): walk and turn together. The arc band is now ~0.12–0.55.
DEFAULT_SPIN_OFFSET = 0.55
# Ball is exclusive align-then-walk. Hysteresis so a one-pixel wobble at
# the threshold does not start/stop the gait. Spin until |offset| <= IN,
# then walk until |offset| >= OUT. Never both at once.
DEFAULT_BALL_ALIGN_IN = 0.10
DEFAULT_BALL_ALIGN_OUT = 0.22
# Arrival: a standing person is already ~75% of 720p height from several
# meters down a hallway. Using that as "close enough, halt" made the dog
# freeze mid-chase. Height must nearly fill the frame; area is the main stop.
DEFAULT_STOP_SPAN = 0.92
# Slow EMA: gait bounce looks like the person darting sideways. Trust new
# samples little so the motors never see a stride.
DEFAULT_CENTER_EMA_FAR = 0.18
DEFAULT_CENTER_EMA_NEAR = 0.10
DEFAULT_WIDE_BOX = 0.35
DEFAULT_YAW_SLEW = 5.0
# Cyan (BGR) outline on the instance the gait is following.
TARGET_BOX_COLOR = (255, 255, 0)

# XGO-LITE leg layout: digit 1 = leg corner, digit 2 = joint from foot (1) to shoulder (3).
SERVO_IDS: tuple[int, ...] = (
    11,
    12,
    13,
    21,
    22,
    23,
    31,
    32,
    33,
    41,
    42,
    43,
    51,
    52,
    53,
)
SERVO_LABELS: dict[int, str] = {
    11: "Right Front Foot",
    12: "Right Front Knee",
    13: "Right Front Shoulder",
    21: "Left Front Foot",
    22: "Left Front Knee",
    23: "Left Front Shoulder",
    31: "Right Rear Foot",
    32: "Right Rear Knee",
    33: "Right Rear Shoulder",
    41: "Left Rear Foot",
    42: "Left Rear Knee",
    43: "Left Rear Shoulder",
    51: "Arm Base",
    52: "Arm Elbow",
    53: "Arm Gripper",
}

# HSV ranges for a solid-colored ball. Red wraps around hue 0 so it needs two bands.
BALL_HSV_RANGES: dict[str, list[tuple[tuple[int, int, int], tuple[int, int, int]]]] = {
    "orange": [((5, 80, 60), (25, 255, 255))],
    "yellow": [((20, 70, 70), (40, 255, 255))],
    "green": [((40, 70, 50), (85, 255, 255))],
    "blue": [((95, 70, 50), (130, 255, 255))],
    "red": [((0, 90, 60), (10, 255, 255)), ((170, 90, 60), (180, 255, 255))],
}

# MJPG+AVI is the combination OpenCV on a Pi will actually encode.
RECORD_FOURCC = "MJPG"
JPEG_QUALITY = 80
MJPEG_BOUNDARY = b"frame"
SCRIPT_DIR = Path(__file__).resolve().parent

PAGE_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>XGO camera</title>
  <style>
    body { margin: 0; background: #111; color: #eee; font-family: system-ui, sans-serif; }
    .stage { position: relative; width: 100vw; height: 100vh; overflow: hidden; background: #000; }
    .feed { display: block; width: 100%; height: 100%; object-fit: contain; position: relative; z-index: 1; }
    #stats-panel {
      position: absolute; top: 12px; right: 12px; width: min(340px, 42vw);
      max-height: calc(100vh - 24px); overflow: auto; z-index: 1000;
      background: rgba(10, 12, 18, 0.92); border: 2px solid rgba(120, 200, 255, 0.55);
      border-radius: 10px; padding: 12px 14px; font-size: 13px; line-height: 1.35;
      box-shadow: 0 8px 28px rgba(0,0,0,0.45); backdrop-filter: blur(6px);
    }
    #stats-panel h2 { margin: 0 0 8px; font-size: 15px; font-weight: 600; }
    #stats-panel h3 { margin: 12px 0 6px; font-size: 12px; text-transform: uppercase; letter-spacing: 0.04em; color: #9ab; }
    .stat-row { display: flex; justify-content: space-between; gap: 8px; padding: 2px 0; }
    .stat-row span:last-child { font-variant-numeric: tabular-nums; color: #dfe; }
    .battery-bar { height: 8px; border-radius: 4px; background: #333; margin-top: 6px; overflow: hidden; }
    .battery-fill { height: 100%; background: linear-gradient(90deg, #3d8, #6c6); width: 0%; transition: width 0.3s; }
    .servo-list { max-height: 220px; overflow: auto; padding-right: 4px; }
    .warn { color: #f88; font-size: 12px; margin-top: 8px; }
  </style>
</head>
<body>
  <div class="stage">
    <img class="feed" src="/stream" alt="XGO camera">
    <aside id="stats-panel" aria-live="polite">
      <h2>Robot stats</h2>
      <div id="stats-body">Connecting to telemetry…</div>
    </aside>
  </div>
  <script>
    function fmt(n, digits) {
      if (n === null || n === undefined || Number.isNaN(n)) return "—";
      return Number(n).toFixed(digits);
    }
    function renderStats(data) {
      const root = document.getElementById("stats-body");
      if (!data.ok) {
        root.innerHTML = '<p class="warn">' + (data.error || "Telemetry unavailable") + '</p>';
        return;
      }
      const bat = data.battery_percent;
      const servos = (data.servos || []).map(s =>
        '<div class="stat-row"><span>' + s.name + '</span><span>' + fmt(s.angle_deg, 1) + '°</span></div>'
      ).join("");
      const imu = data.imu || {};
      const acc = imu.acceleration || {};
      const rot = imu.rotation || {};
      const gyro = imu.gyroscope || {};
      root.innerHTML =
        '<h3>Battery</h3>' +
        '<div class="stat-row"><span>Charge</span><span>' + bat + '%</span></div>' +
        '<div class="battery-bar"><div class="battery-fill" style="width:' + Math.max(0, Math.min(100, bat)) + '%"></div></div>' +
        '<h3>Servos</h3><div class="servo-list">' + servos + '</div>' +
        '<h3>IMU acceleration (m/s²)</h3>' +
        '<div class="stat-row"><span>X</span><span>' + fmt(acc.x, 2) + '</span></div>' +
        '<div class="stat-row"><span>Y</span><span>' + fmt(acc.y, 2) + '</span></div>' +
        '<div class="stat-row"><span>Z</span><span>' + fmt(acc.z, 2) + '</span></div>' +
        '<h3>Rotation (°)</h3>' +
        '<div class="stat-row"><span>Roll</span><span>' + fmt(rot.roll, 2) + '</span></div>' +
        '<div class="stat-row"><span>Pitch</span><span>' + fmt(rot.pitch, 2) + '</span></div>' +
        '<div class="stat-row"><span>Yaw</span><span>' + fmt(rot.yaw, 2) + '</span></div>' +
        '<h3>Gyro (°/s)</h3>' +
        '<div class="stat-row"><span>X</span><span>' + fmt(gyro.x, 2) + '</span></div>' +
        '<div class="stat-row"><span>Y</span><span>' + fmt(gyro.y, 2) + '</span></div>' +
        '<div class="stat-row"><span>Z</span><span>' + fmt(gyro.z, 2) + '</span></div>';
    }
    async function refreshStats() {
      try {
        const response = await fetch("/stats", { cache: "no-store" });
        renderStats(await response.json());
      } catch (err) {
        renderStats({ ok: false, error: "Could not load /stats" });
      }
    }
    refreshStats();
    setInterval(refreshStats, 400);
  </script>
</body>
</html>
"""
