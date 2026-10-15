"""
XGO robot homework package: camera, YOLO, live page, and optional chase gait.

First-day orientation / Purpose:
    This folder is the code you copy onto the XGO robot (Raspberry Pi CM4)
    and run after you SSH in over Tailscale. The XGO is a desktop robot dog.
    Its camera is plugged into the Pi on the robot. Your Windows laptop is
    only a remote keyboard. If you run these scripts on Windows with camera
    index 0, you will record the laptop webcam and think the robot is broken.

    Trinity Guest Wi-Fi uses device-to-device isolation. A ``10.202.x.x``
    address on the robot is real, but SSH to it from a laptop on Guest will
    time out. The working SSH target is the Tailscale ``100.x.x.x`` address.
    These scripts do not care which overlay you used; they only care that
    you are executing Python *on the Pi*.

    There used to be two giant files: ``xgo_camera_stream.py`` (~1700 lines)
    and ``xgo_chase_object.py`` (~640 lines). They duplicated the camera
    loop, HTTP server, YOLO worker, and telemetry poller. Chase then
    imported the stream file, so a stale copy on the Pi streamed video but
    could not walk. That split is gone. Logic lives in focused modules.
    ``main.py`` is the one argparse front door. The old filenames remain as
    thin shims so ``python3 xgo_camera_stream.py`` still works.

    Copy the **whole folder** to the Pi (for example ``~/xgo_robot``), not
    a single flattened ``.py``. Flattening breaks the package imports.

Core loop / mental model:
    1. Copy this directory to the robot (scp the folder, not one file).
    2. SSH as ``pi`` to the Tailscale IP, ``cd`` into this folder.
    3. On the Pi, ``pip3 install ultralytics xgolib`` once.
    4. Pick a command:
           python3 main.py stream
           python3 chase_person.py
           python3 chase_colored_ball.py --color orange
           python3 main.py chase-person
           python3 main.py chase-ball --color orange
           python3 xgo_camera_stream.py
    5. OpenCV asks the camera for a 1280x720 picture. Person chase runs
       YOLOv8n (class 0 only) and draws person boxes. Ball chase never
       loads YOLO: it thresholds a named HSV color and keeps round blobs.
       Each annotated frame is appended to an ``.avi`` and compressed to
       a JPEG. Chase sends gait commands toward one locked box.
    6. On your Windows computer, open the ``http://100.x.x.x:8080/`` URL
       the script prints. The browser shows the live camera with YOLO boxes
       and a stats panel (battery, servos, IMU) in the top-right corner.
    7. Press Ctrl+C on the Pi. The walker stops, the file is closed, and
       the page dies. Copy the AVI back with scp if you want it on Windows.
    8. Do not call ``cv2.imshow`` over SSH unless HDMI has a desktop and you
       exported ``DISPLAY=:0``. Qt will abort with "could not connect to
       display". The browser page is the live view that works over SSH.

Key terms / vocabulary:
    - XGO: The Elecfreaks/Luwu robot dog this class is using. Compute is a
      Raspberry Pi CM4 sitting inside the chassis.
    - CM4: Compute Module 4. A Raspberry Pi on a small board, not a Pi 4
      desktop kit. Same Linux, same OpenCV, different physical camera cable.
    - OpenCV (cv2): Library that talks to cameras and writes video files.
    - Frame: One image from the camera, a grid of pixels.
    - VideoCapture: OpenCV's handle to the live camera.
    - VideoWriter: OpenCV's handle to the output movie file.
    - Adjacent file: A file in this same folder, e.g. ``xgo_camera_….avi``
      beside ``main.py``.
    - Tailscale: A mesh VPN. Guest isolation cannot forward laptop→robot
      packets; Tailscale sends them out to the internet and back so SSH works.
    - HDMI session: Monitor plugged into the robot. That is a real display.
      SSH is not.
    - V4L2: Video4Linux2, the Linux camera API. We force this backend on
      the Pi because OpenCV's autodetect is flaky.
    - MJPEG: A live video made of one JPEG still after another, sent over
      HTTP. A browser can show it. It is not the ``.avi`` file.
    - Live page: ``http://<pi-tailscale-ip>:8080/`` served while a command
      is running. The image fills the browser window. Your workstation can
      open it because its Tailscale grant allows every port to ``tag:pi``.
      A person whose grant is only port 22 cannot.
    - YOLO: The detector running on the Pi. It writes rectangles and labels
      such as ``person 0.91`` onto the frames before they are sent. The
      picture keeps updating if the model is slower than the camera.
    - Detection: A tuple ``(x1, y1, x2, y2, label, bgr_color)`` for one box.
    - Tiling / slices: YOLO resizes the whole frame to 640 pixels, which
      erases a ping-pong ball. ``--slices 2`` runs overlapping crops so a
      small ball can survive. It is slower.
    - Color blob / ``--ball-color``: HSV threshold for a solid-colored ball
      YOLO will miss. Use with ``--class "sports ball"``. The COCO name is
      ``sports ball``, not ``ball``.
    - Chase person: Walk toward **one locked person** using YOLOv8n. There
      is no ``--class``. Chairs and TVs are not requested from the network.
    - Chase ball: Walk toward a solid-colored ball using HSV + roundness.
      YOLO's ``sports ball`` class is a basketball, not a ping-pong ball,
      so ball chase does not load ultralytics at all. Steering is
      align-then-go: spin until the blob is centered, then walk. Never both.
    - Lock: Person chase acquires the most-centered standing person, then
      follows that instance by predicted position, size, overlap, and a
      clothing histogram so a hallway box cannot steal the identity. Ball
      chase acquires the largest blob of that color and follows it.
    - Arc: Person only. Forward ``move('x')`` plus a capped ``turn()`` when
      the person is off-center but not at the frame edge. Exclusive
      turn-or-walk on a *person* is why it used to spin sideways instead
      of approaching.
    - Deadzone: Person only. Narrow center band treated as "straight
      ahead." Outside it, the dog arcs rather than walking a line that
      lets you drift off the image.
    - Stop area: Box / frame area at which we halt (not true distance).
    - Telemetry: Background serial reads (battery, servos, IMU) for the
      stats panel. The same UART is used for walking; two ``XGO()`` objects
      on ``/dev/ttyAMA0`` will corrupt each other.
    - LiveHub: One-slot JPEG mailbox. A slow browser skips frames instead
      of stalling the camera thread.
    - Shim: ``xgo_camera_stream.py`` / ``xgo_chase_object.py``. They parse
      the old argument style and call ``main.py``.

Important invariants / boundaries:
    - Capture on the robot. Watch on the laptop through the web page.
      Do not try to pop a Qt window across SSH.
    - Only one process may own ``/dev/ttyAMA0``. Do not run stream and
      chase at the same time, and do not run either while a vendor demo
      still has the serial port.
    - Person chase stops when the locked person is lost or close. It does
      not grab a bystander to "keep busy." Ball chase re-acquires the
      largest remaining blob of that color. Ctrl+C also sends ``stop()``.
      It has no cliff sensor. Keep it on the floor.
    - Do not put homework inside ``RaspberryPi-CM4-main`` on the Pi; that
      is vendor software. A vendor ``cmd.py`` there shadows the stdlib
      ``cmd`` module and crashes PyTorch.
    - Default SSH user on this kit is ``pi``. Do not commit passwords into
      scripts in this package.
    - This package does not configure Wi-Fi, Tailscale, or the XGO gait
      firmware. Copy the whole package; do not flatten files into
      ``~/test_xgo_connectivity``.
    - Sibling modules import each other with relative imports
      (``from .models import Detection``). The barrel re-exports the public
      API. Do not import the shim files from the barrel (circular).

Read order:
    1. ``main.py`` — argparse subcommands and the shared pipeline.
    2. ``models.py`` — Detection type, defaults, live-page HTML.
    3. ``helpers.py`` — box math, camera/writer open, Tailscale URLs, xgolib.
    4. ``detect.py`` — YOLOv8n person detector and HSV ball detector.
    5. ``http_stream.py`` — LiveHub mailbox and the MJPEG HTTP server.
    6. ``telemetry.py`` — serial owner for stats and walk commands.
    7. ``camera.py`` — the capture / annotate / record / publish loop.
    8. ``chase.py`` — PersonChaser / BallChaser gait and lock.
    9. ``chase_person.py`` / ``chase_colored_ball.py`` — the two chase scripts.
    10. ``xgo_camera_stream.py`` / ``xgo_chase_object.py`` — old shims only.
    11. ``xgo_robot_config.json`` — local SSH hints for humans; not imported.

Exports:
    - models: Detection, PAGE_HTML, DEFAULT_* constants.
    - helpers: nms, drawing, capture/writer, print_live_urls, open_xgo.
    - detect: YoloAnnotator, ColorBallAnnotator, load_yolo,
      load_person_detector, load_ball_detector.
    - http_stream: LiveHub, start_http_server.
    - telemetry: XgoTelemetryPoller.
    - camera: run_recording, run_viewer.
    - chase: pick_target, PersonChaser, BallChaser.
    - main.py is the CLI; import ``xgo_robot.main`` rather than this barrel
      so ``python3 main.py`` does not circular-import itself.
"""

from .camera import run_recording, run_viewer
from .chase import BallChaser, ObjectChaser, PersonChaser, pick_target
from .detect import (
    ColorBallAnnotator,
    YoloAnnotator,
    load_ball_detector,
    load_person_detector,
    load_yolo,
)
from .helpers import (
    apply_small_object_flags,
    class_color,
    class_name_from_label,
    detect_colored_ball,
    draw_detections,
    nms_detections,
    open_capture,
    open_writer,
    open_xgo,
    print_live_urls,
    resolve_fps,
    resolve_output_path,
    scale_detections,
)
from .http_stream import LiveHub, start_http_server
from .models import (
    BALL_HSV_RANGES,
    DEFAULT_PORT,
    DEFAULT_SOURCE,
    DEFAULT_XGO_PORT,
    Detection,
    PAGE_HTML,
    SCRIPT_DIR,
)
from .telemetry import XgoTelemetryPoller

__all__ = [
    "BALL_HSV_RANGES",
    "DEFAULT_PORT",
    "DEFAULT_SOURCE",
    "DEFAULT_XGO_PORT",
    "Detection",
    "BallChaser",
    "ColorBallAnnotator",
    "LiveHub",
    "ObjectChaser",
    "PersonChaser",
    "PAGE_HTML",
    "SCRIPT_DIR",
    "XgoTelemetryPoller",
    "YoloAnnotator",
    "apply_small_object_flags",
    "class_color",
    "class_name_from_label",
    "detect_colored_ball",
    "draw_detections",
    "load_ball_detector",
    "load_person_detector",
    "load_yolo",
    "nms_detections",
    "open_capture",
    "open_writer",
    "open_xgo",
    "pick_target",
    "print_live_urls",
    "resolve_fps",
    "resolve_output_path",
    "run_recording",
    "run_viewer",
    "scale_detections",
    "start_http_server",
]
