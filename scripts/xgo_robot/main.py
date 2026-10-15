"""
Central command parser for the XGO homework package.

Purpose:
    This file is the one program you run on the Raspberry Pi. The two old
    mega-scripts (``xgo_camera_stream.py`` and ``xgo_chase_object.py``) did
    almost the same work: open the camera, run YOLO, write an AVI, serve a
    live browser page. One of them also walked toward a class. Duplicating
    that pipeline in two 1500-line files is how the chase script broke when
    the stream file on the Pi was stale.

    Run from this folder:

        python3 main.py stream
        python3 main.py chase-person
        python3 main.py chase-ball --color orange
        python3 chase_person.py
        python3 chase_colored_ball.py --color orange
        python3 main.py view --url http://100.x.x.x:8080/stream

    Or from the parent of this package:

        python3 -m xgo_robot stream

    The old filenames still work. They are thin shims that inject the
    subcommand and call ``main()`` here.

Key terms:
    - Subcommand: The first word after ``main.py``. ``stream`` watches.
      ``chase-person`` walks toward a person (YOLOv8n). ``chase-ball``
      walks toward a colored ball (HSV, no neural net). ``view`` is the
      laptop OpenCV window.
    - Parent flags: Camera, HTTP, and telemetry options shared so
      ``--port`` is not defined twice. YOLO flags live on stream and
      chase-person only. ``--color`` lives on chase-ball.
    - Pipeline: Load YOLO, open the live page, optionally attach a walker,
      then start the camera loop. The page comes up *before* the camera
      so a camera failure still prints a URL (or a port-in-use error).
    - on_frame: Callback the camera loop runs after boxes are drawn.
      Chase uses it to steer and to paint a cyan outline on the chased box.
    - Shim: A tiny leftover ``.py`` whose only job is ``python3 old_name.py``.

Non-obvious behavior:
    - ``python3 main.py`` with no subcommand is an error. Pick
      stream / chase-person / chase-ball / view.
    - There is no generic ``chase --class``. Person and ball use different
      detectors on purpose.
    - ``chase-person`` has no ``--no-yolo``. Without people boxes it cannot walk.
    - ``chase-ball`` never loads ultralytics.
    - The walker is stopped *before* telemetry closes the UART. Stopping
      after ``telemetry.close()`` cannot reach the motion board.

Exports:
    - parse_args: Build the argparse tree.
    - cmd_stream, cmd_chase_person, cmd_chase_ball, cmd_view: Handlers.
    - main: Parse argv and dispatch.
"""

from __future__ import annotations

import argparse
import sys
import threading
from collections.abc import Callable
from pathlib import Path
from typing import Optional

# ``python3 main.py stream`` from inside this folder has an empty __package__,
# so relative imports would fail. Put the parent of xgo_robot on sys.path and
# claim this module belongs to that package *before* the sibling imports below.
if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    __package__ = "xgo_robot"

import cv2

from .camera import run_recording, run_viewer
from .chase import BallChaser, PersonChaser
from .detect import load_ball_detector, load_person_detector, load_yolo
from .helpers import (
    apply_small_object_flags,
    open_capture,
    open_xgo,
    print_live_urls,
    resolve_fps,
    resolve_output_path,
)
from .http_stream import LiveHub, start_http_server
from .models import (
    BALL_HSV_RANGES,
    DEFAULT_CENTER_DEADZONE,
    DEFAULT_COMMAND_HZ,
    DEFAULT_FPS,
    DEFAULT_HEIGHT,
    DEFAULT_PORT,
    DEFAULT_SOURCE,
    DEFAULT_STOP_AREA,
    DEFAULT_TELEMETRY_HZ,
    DEFAULT_TURN_SPEED,
    DEFAULT_WALK_SPEED,
    DEFAULT_WIDTH,
    DEFAULT_XGO_PORT,
    DEFAULT_YOLO_CONF,
    DEFAULT_YOLO_IMGSZ,
    DEFAULT_YOLO_MODEL,
    Detection,
    TARGET_BOX_COLOR,
)
from .telemetry import XgoTelemetryPoller

# These aliases are evaluated at import time. `X | None` is a 3.10 operator;
# Raspberry Pi OS still ships 3.9, so Optional[] is required here even though
# function annotations below are postponed by `from __future__ import annotations`.
FrameCallback = Callable[[object, list[Detection]], None]
Cleanup = Callable[[], None]
WalkSetup = Callable[
    [Optional[XgoTelemetryPoller], list[Cleanup]],
    Optional[FrameCallback],
]

__all__ = [
    "parse_args",
    "cmd_stream",
    "cmd_chase_person",
    "cmd_chase_ball",
    "cmd_view",
    "main",
]


def _add_camera_flags(parser: argparse.ArgumentParser) -> None:
    """Attach camera, HTTP, and telemetry flags shared by stream and chase.

    :param parser: Parent or subparser to mutate.
    :returns: None.
    """

    parser.add_argument("--source", default=DEFAULT_SOURCE, help="Camera index or stream URL.")
    parser.add_argument("--width", type=int, default=DEFAULT_WIDTH)
    parser.add_argument("--height", type=int, default=DEFAULT_HEIGHT)
    parser.add_argument("--output", default=None, help="AVI path. Default: timestamped file in this folder.")
    parser.add_argument("--preview", action="store_true", help="OpenCV window on the Pi (needs HDMI).")
    parser.add_argument("--fps", type=float, default=None, help=f"AVI playback FPS. Default: camera or {DEFAULT_FPS:g}.")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"Live page port. Default: {DEFAULT_PORT}.")
    parser.add_argument("--bind", default="0.0.0.0", help="Listen address. Default 0.0.0.0.")
    parser.add_argument("--no-record", action="store_true", help="Do not write an AVI.")
    parser.add_argument("--no-serve", action="store_true", help="Do not open the live web page.")
    parser.add_argument("--no-telemetry", action="store_true", help="Skip the stats panel.")
    parser.add_argument("--xgo-port", default=DEFAULT_XGO_PORT, help=f"Serial port. Default: {DEFAULT_XGO_PORT}.")


def _add_yolo_flags(parser: argparse.ArgumentParser) -> None:
    """Attach YOLOv8n flags. Not used by chase-ball.

    :param parser: Subparser to mutate.
    :returns: None.
    """

    parser.add_argument("--model", default=DEFAULT_YOLO_MODEL, help=f"YOLO weights. Default: {DEFAULT_YOLO_MODEL}.")
    parser.add_argument("--conf", type=float, default=DEFAULT_YOLO_CONF, help=f"Min YOLO score. Default: {DEFAULT_YOLO_CONF:g}.")
    parser.add_argument("--imgsz", type=int, default=DEFAULT_YOLO_IMGSZ, help=f"YOLO size. Default: {DEFAULT_YOLO_IMGSZ}.")
    parser.add_argument("--slices", type=int, default=1, help="Tile grid. 2 = 2x2 overlapping crops.")
    parser.add_argument("--small-objects", action="store_true", help="Shorthand: --slices 2 --conf 0.12.")


def _add_walk_flags(parser: argparse.ArgumentParser) -> None:
    """Attach gait flags used by both chase commands.

    :param parser: Subparser to mutate.
    :returns: None.
    """

    parser.add_argument("--walk-speed", type=float, default=DEFAULT_WALK_SPEED)
    parser.add_argument("--turn-speed", type=float, default=DEFAULT_TURN_SPEED)
    parser.add_argument("--center-deadzone", type=float, default=DEFAULT_CENTER_DEADZONE)
    parser.add_argument("--stop-area", type=float, default=DEFAULT_STOP_AREA)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    """Build the top-level parser with stream, chase-person, chase-ball, view.

    :param argv: Token list, or None to read ``sys.argv[1:]``.
    :returns: Parsed namespace including ``command``.
    """

    parser = argparse.ArgumentParser(
        description="XGO camera: live page, person chase (YOLO), ball chase (HSV)."
    )
    sub = parser.add_subparsers(dest="command", required=True)

    stream = sub.add_parser("stream", help="Live page + optional YOLO boxes. Does not walk.")
    _add_camera_flags(stream)
    _add_yolo_flags(stream)
    stream.add_argument("--no-yolo", action="store_true", help="Camera only, no detector.")
    stream.add_argument(
        "--ball-color",
        default=None,
        choices=sorted(BALL_HSV_RANGES),
        help="Optional HSV overlay on the stream (does not walk).",
    )
    stream.add_argument("--ball-class", default="ball", help="Label written on stream color blobs.")
    stream.add_argument("--min-ball-area", type=int, default=80, help="Ignore color speckles smaller than this.")

    person = sub.add_parser("chase-person", help="Walk toward one person (YOLOv8n class 0).")
    _add_camera_flags(person)
    _add_yolo_flags(person)
    _add_walk_flags(person)

    ball = sub.add_parser("chase-ball", help="Walk toward a colored ball (HSV, no YOLO).")
    _add_camera_flags(ball)
    _add_walk_flags(ball)
    ball.add_argument(
        "--color",
        dest="ball_color",
        required=True,
        choices=sorted(BALL_HSV_RANGES),
        help="Ball color to threshold, e.g. orange or red.",
    )
    ball.add_argument("--min-ball-area", type=int, default=80, help="Ignore color speckles smaller than this.")

    view = sub.add_parser("view", help="Laptop OpenCV window for an existing /stream URL.")
    view.add_argument("--url", required=True, help="e.g. http://100.74.30.90:8080/stream")

    return parser.parse_args(argv)


def cmd_view(args: argparse.Namespace) -> int:
    """Open an OpenCV window on the machine where this command runs.

    :param args: Must include ``url``.
    :returns: Process exit code.
    """

    try:
        run_viewer(args.url)
    except RuntimeError as exc:
        print(exc, file=sys.stderr)
        return 1
    return 0


def cmd_stream(args: argparse.Namespace) -> int:
    """Serve the live page (and optional AVI) without walking.

    :param args: Flags from the ``stream`` subcommand.
    :returns: Process exit code.
    """

    if args.no_record and args.no_serve:
        print("Nothing to do: both --no-record and --no-serve were set.", file=sys.stderr)
        return 1
    apply_small_object_flags(args)

    def make_annotator() -> object | None:
        """Optional YOLO overlay for stream. ``--no-yolo`` skips it.

        :returns: Annotator or None.
        """

        if getattr(args, "no_yolo", False):
            return None
        return load_yolo(
            args.model,
            args.conf,
            args.imgsz,
            slices=args.slices,
            ball_color=getattr(args, "ball_color", None),
            ball_class=getattr(args, "ball_class", "ball"),
            min_ball_area=getattr(args, "min_ball_area", 80),
        )

    return _run_pipeline(args, walk_setup=None, make_annotator=make_annotator)


def cmd_chase_person(args: argparse.Namespace) -> int:
    """Serve the live page and walk toward one YOLOv8n person.

    :param args: Flags from the ``chase-person`` subcommand.
    :returns: Process exit code.
    """

    apply_small_object_flags(args)
    print("Chasing person (YOLOv8n). Ctrl+C stops the walk. Keep the dog away from stairs.")

    def make_annotator() -> object:
        """Load person-only YOLO.

        :returns: Running ``YoloAnnotator``.
        """

        return load_person_detector(args.model, args.conf, args.imgsz, slices=args.slices)

    return _run_pipeline(
        args,
        walk_setup=_make_walk_setup(PersonChaser, args),
        make_annotator=make_annotator,
    )


def cmd_chase_ball(args: argparse.Namespace) -> int:
    """Serve the live page and walk toward an HSV-colored ball. No YOLO.

    :param args: Flags from the ``chase-ball`` subcommand.
    :returns: Process exit code.
    """

    print(
        f"Chasing {args.ball_color!r} ball (HSV + roundness, no YOLO). "
        "Ctrl+C stops the walk. Keep the dog away from stairs."
    )

    def make_annotator() -> object:
        """Load the color-blob detector.

        :returns: ``ColorBallAnnotator``.
        """

        return load_ball_detector(args.ball_color, min_area=args.min_ball_area)

    return _run_pipeline(
        args,
        walk_setup=_make_walk_setup(BallChaser, args),
        make_annotator=make_annotator,
    )


def _make_walk_setup(chaser_cls: type, args: argparse.Namespace) -> WalkSetup:
    """Build the chase on_frame hook for a PersonChaser or BallChaser class.

    :param chaser_cls: ``PersonChaser`` or ``BallChaser``.
    :param args: Parsed walk-speed flags.
    :returns: walk_setup callable for ``_run_pipeline``.
    """

    def walk_setup(
        telemetry: XgoTelemetryPoller | None,
        cleanups: list[Cleanup],
    ) -> FrameCallback | None:
        """Build the chaser after HTTP is up, then return on_frame.

        :param telemetry: Connected poller, or None when ``--no-telemetry``.
        :param cleanups: Pipeline finally-hooks. We append ``chaser.stop``.
        :returns: Frame callback, or None if walking cannot start.
        """

        dog = None
        walk_telemetry = None
        if telemetry is not None:
            if telemetry.wait_until_connected():
                walk_telemetry = telemetry
            else:
                print(
                    "xgolib did not connect. Stream will keep running; the dog will not walk.",
                    file=sys.stderr,
                )
        else:
            try:
                dog = open_xgo(args.xgo_port)
            except RuntimeError as exc:
                print(f"{exc} Stream will keep running; the dog will not walk.", file=sys.stderr)

        chaser = chaser_cls(
            walk_speed=args.walk_speed,
            turn_speed=args.turn_speed,
            center_deadzone=args.center_deadzone,
            stop_area=args.stop_area,
            command_hz=DEFAULT_COMMAND_HZ,
            telemetry=walk_telemetry,
            dog=dog,
        )
        # Halt the gait before the pipeline closes the UART.
        cleanups.append(chaser.stop)

        def on_frame(display: object, detections: list[Detection]) -> None:
            """Steer, then outline the chased instance in cyan.

            :param display: BGR frame already annotated. Person chase uses
                the pixels for a clothing histogram. Ball chase ignores them.
            :param detections: Boxes in this frame's coordinates.
            :returns: None.
            """

            target = chaser.update(
                detections,
                int(display.shape[1]),
                int(display.shape[0]),
                frame=display,
            )
            if target is None:
                return
            x1, y1, x2, y2, _label, _color = target
            cv2.rectangle(display, (x1, y1), (x2, y2), TARGET_BOX_COLOR, 3)

        return on_frame

    return walk_setup


def _run_pipeline(
    args: argparse.Namespace,
    walk_setup: WalkSetup | None,
    make_annotator: Callable[[], object] | None = None,
) -> int:
    """Start detector, HTTP, telemetry, camera. Shared by stream and chase.

    :param args: Parsed CLI flags.
    :param walk_setup: Chase hook run after the page is listening.
    :param make_annotator: Factory that returns submit/snapshot/close, or None.
    :returns: Process exit code.
    """

    output = resolve_output_path(args.output, args.no_record)
    server = None
    stop = threading.Event()
    hub: LiveHub | None = None
    annotator = None
    telemetry: XgoTelemetryPoller | None = None
    frame_cb: FrameCallback | None = None
    cleanups: list[Cleanup] = []
    try:
        # Load the detector before opening the page so a missing install fails fast.
        if make_annotator is not None:
            annotator = make_annotator()

        # Open the page before the camera so a later failure still leaves a URL.
        if not args.no_serve:
            hub = LiveHub()
            if not args.no_telemetry:
                telemetry = XgoTelemetryPoller(port=args.xgo_port, poll_hz=DEFAULT_TELEMETRY_HZ)
                print("Telemetry: stats panel at GET /stats (open http://<pi>:8080/ not /stream alone)")
            server = start_http_server(args.bind, args.port, hub, stop, telemetry)
            print_live_urls(args.port)
            print("Live feed is up. Open the URL above even if walking fails to start.")

        if walk_setup is not None:
            frame_cb = walk_setup(telemetry, cleanups)

        capture = open_capture(args.source, args.width, args.height)
        fps = resolve_fps(capture, args.fps)
        run_recording(
            capture,
            output,
            fps,
            args.preview,
            hub,
            annotator,
            args.width,
            args.height,
            frame_cb,
        )
    except RuntimeError as exc:
        print(exc, file=sys.stderr)
        return 1
    finally:
        # Walker first: stop() must still own /dev/ttyAMA0.
        for cleanup in reversed(cleanups):
            cleanup()
        stop.set()
        if hub is not None:
            hub.wake()
        if annotator is not None:
            annotator.close()
        if telemetry is not None:
            telemetry.close()
        if server is not None:
            server.shutdown()
            server.server_close()
    return 0


def main(argv: list[str] | None = None) -> int:
    """Parse the command line and run stream, chase, or view.

    :param argv: Optional token list for tests. None reads ``sys.argv``.
    :returns: Process exit code.
    """

    args = parse_args(argv)
    if args.command == "view":
        return cmd_view(args)
    if args.command == "chase-person":
        return cmd_chase_person(args)
    if args.command == "chase-ball":
        return cmd_chase_ball(args)
    return cmd_stream(args)


if __name__ == "__main__":
    raise SystemExit(main())
