"""Drive the XGO Lite from gamepad commands streamed as JSON lines on stdin.

Runs on the robot. Each line is a JSON object from gamepad_sender.py, piped over
SSH, with axes in -1..1:

  {"x": fwd, "y": left, "yaw": ccw, "enable": bool,     walking (enable = LB held)
   "arm": bool, "ax": reach, "az": lift, "claw": close,  arm mode (arm = RB held)
   "bark": presses,                                      B-button press counter
   "kneel": 1 | 0 | -1}                                  LT + D-pad down / up

Walking stops when the sticks centre, enable drops, arm mode starts, input goes
quiet for --timeout seconds, the stream ends, or anything goes wrong. In arm mode
the sticks set arm/claw *speed*; the arm holds its position when they centre.
Kneeling pitches the front down and lowers the body (the vendor's floor-pickup
posture); it holds when released and returns to standing on exit.
"""

import argparse
import queue
import signal
import sys
import threading
import time
from importlib import import_module
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "sensor_monitor"))

from actions import BARK_SOUND, Barker, Fetcher, FullReset, ResetButton  # noqa: E402
from align import alignment_legs_driver, alignment_measure, alignment_reference  # noqa: E402
from body import (ARM_LIMITS, ARM_REACH, LIMITS, ArmController, Driver,  # noqa: E402
                  LegStepper, PosePreparer, Posture, Ramp, robot_state, scale)
from protocol import AXES, ARM_AXES, Message, bounded_number, parse_message  # noqa: E402

# Re-exported so callers and tests can keep using teleop_receiver.<name>.
__all__ = ["ARM_AXES", "ARM_LIMITS", "ARM_REACH", "AXES", "ArmController", "BARK_SOUND", "Barker",
           "Driver", "EOF", "Fetcher", "FullReset", "LIMITS", "LegStepper", "Message",
           "PosePreparer", "Posture", "Ramp", "ResetButton", "alignment_legs_driver",
           "alignment_measure", "alignment_reference", "bounded_number", "build_parser",
           "latest", "main", "make_log", "parse_message", "read_lines", "robot_state", "run", "scale",
           "start_viewer"]

EOF = object()


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    for flag, axis, default, unit in (("max-x", "x", 12, "mm"), ("max-y", "y", 8, "mm"),
                                      ("max-turn", "yaw", 40, "degrees/s")):
        maximum = LIMITS[axis]
        parser.add_argument(
            f"--{flag}", type=lambda v, m=maximum, name=flag: bounded_number(
                v, minimum=1, maximum=m, label=name),
            default=default, metavar="VALUE",
            help=f"full-stick {axis} speed (1–{maximum} {unit}; default: {default})",
        )
    parser.add_argument(
        "--arm-speed", type=lambda v: bounded_number(v, minimum=5, maximum=150, label="arm-speed"),
        default=60, metavar="MM_PER_S", help="full-stick arm speed (5–150 mm/s; default: 60)")
    parser.add_argument(
        "--claw-speed", type=lambda v: bounded_number(v, minimum=20, maximum=500, label="claw-speed"),
        default=170, metavar="UNITS_PER_S",
        help="full-stick claw speed (20–500 of 0–255 per s; default: 170)")
    for axis, default in (("x", 80), ("z", 30)):  # measured reachable
        low, high = ARM_LIMITS[axis]
        parser.add_argument(
            f"--arm-home-{axis}", type=lambda v, lo=low, hi=high, name=axis: bounded_number(
                v, minimum=lo, maximum=hi, label=f"arm-home-{name}"),
            default=default, metavar="MM",
            help=f"arm {axis} the first arm-mode stick move starts from ({low}…{high}; default: {default})")
    parser.add_argument("--claw-start", type=lambda v: bounded_number(v, minimum=0, maximum=255,
                                                                      label="claw-start"),
                        default=128, metavar="VALUE",
                        help="assumed claw position before the first claw move (default: 128)")
    parser.add_argument(
        "--ramp", type=lambda v: bounded_number(v, minimum=0, maximum=10, label="ramp"),
        default=0, metavar="SECONDS",
        help="grow forward/back and side-step strides to the full --max-x/--max-y over this "
             "long while held in one direction (0 = off, the default)")
    parser.add_argument(
        "--ramp-start", type=lambda v: bounded_number(v, minimum=0.1, maximum=1, label="ramp-start"),
        default=0.4, metavar="FRACTION",
        help="fraction of the full stride a ramped move starts at (default: 0.4)")
    parser.add_argument(
        "--kneel-pitch", type=lambda v: bounded_number(v, minimum=0, maximum=10, label="kneel-pitch"),
        default=10, metavar="DEGREES", help="front-down pitch when fully knelt (0–10; default: 10)")
    parser.add_argument(
        "--kneel-height", type=lambda v: bounded_number(v, minimum=60, maximum=110,
                                                        label="kneel-height"),
        default=70, metavar="MM", help="body height when fully knelt (60–110; default: 70)")
    parser.add_argument(
        "--stand-height", type=lambda v: bounded_number(v, minimum=60, maximum=110,
                                                        label="stand-height"),
        default=85, metavar="MM",
        help="body height when standing; 85 is xgolib's neutral value (60–110; default: 85)")
    parser.add_argument(
        "--kneel-time", type=lambda v: bounded_number(v, minimum=0.5, maximum=10, label="kneel-time"),
        default=2, metavar="SECONDS", help="time from standing to fully knelt (default: 2)")
    parser.add_argument("--video-port", type=int, default=0, metavar="PORT",
                        help="serve the live camera on the robot's loopback for an SSH tunnel "
                             "(0 = off)")
    parser.add_argument("--color", choices=("purple", "orange", "light-blue"), default="purple",
                        help="box colour for the recording reference (default: purple)")
    parser.add_argument("--teach-dir", default=str(Path.home() / "xgo_teach"),
                        help="where recordings are stored (default: ~/xgo_teach)")
    parser.add_argument(
        "--record-fps", type=lambda v: bounded_number(v, minimum=1, maximum=30, label="record-fps"),
        default=10, metavar="FPS", help="dataset frames per second (1–30; default: 10)")
    parser.add_argument(
        "--min-free-mb", type=lambda v: bounded_number(v, minimum=50, maximum=5000,
                                                       label="min-free-mb"),
        default=200, metavar="MB", help="stop recording below this much free disk (default: 200)")
    parser.add_argument(
        "--settle", type=lambda v: bounded_number(v, minimum=0.2, maximum=3, label="settle"),
        default=0.7, metavar="SECONDS",
        help="time the robot must stand still before the reference photo (default: 0.7)")
    parser.add_argument(
        "--min-sharpness", type=lambda v: bounded_number(v, minimum=0, maximum=1000,
                                                         label="min-sharpness"),
        default=50, metavar="VALUE",
        help="reject blurrier reference photos (Laplacian variance; a still frame on the "
             "robot measured ~410; default: 50)")
    parser.add_argument("--start-standing", action="store_true",
                        help="stay standing after connecting (default: kneel fully)")
    parser.add_argument("--no-heading-hold", action="store_true",
                        help="don't correct walking drift with the gyro")
    parser.add_argument(
        "--heading-gain", type=lambda v: bounded_number(v, minimum=0.5, maximum=10,
                                                        label="heading-gain"),
        default=2.0, metavar="VALUE", help="turn speed per degree of drift (default: 2)")
    parser.add_argument(
        "--heading-max", type=lambda v: bounded_number(v, minimum=5, maximum=60,
                                                       label="heading-max"),
        default=25, metavar="DEG_PER_S", help="largest drift correction (default: 25)")
    parser.add_argument("--heading-sign", type=int, choices=(-1, 1), default=None,
                        help="gyro direction (default: learned from the first turn and saved)")
    parser.add_argument("--align-motion", action="store_true",
                        help="let holding A move the robot (otherwise A only reports alignment)")
    parser.add_argument("--recalibrate", action="store_true",
                        help="redo the alignment calibration (one short turn) on the next use")
    for flag, default, low, high, help_text in (
            ("align-turn", 40, 10, 60, "turn speed for alignment (probe: ~0.8 deg/s per unit)"),
            ("align-walk", 12, 4, 20, "forward/back speed for alignment"),
            ("tol-u", 0.04, 0.01, 0.2, "aligned when the box centre is this close (image width)"),
            ("tol-h", 0.08, 0.02, 0.3, "aligned when the box size is this close (fraction)"),
            ("tol-v", 0.05, 0.01, 0.2, "allowed vertical difference before 'view mismatch'"),
            ("align-max-moves", 25, 5, 100, "give up after this many alignment moves"),
            ("align-max-seconds", 45, 10, 180, "give up after this long aligning")):
        parser.add_argument(
            f"--{flag}", type=lambda v, lo=low, hi=high, name=flag: bounded_number(
                v, minimum=lo, maximum=hi, label=name),
            default=default, metavar="VALUE", help=f"{help_text} (default: {default})")
    parser.add_argument(
        "--replay-max-gap", type=lambda v: bounded_number(v, minimum=0, maximum=10,
                                                          label="replay-max-gap"),
        default=0.5, metavar="SECONDS",
        help="shorten pauses in a replay to at most this (0 = original timing; default: 0.5)")
    parser.add_argument(
        "--replay-settle", type=lambda v: bounded_number(v, minimum=0, maximum=5,
                                                         label="replay-settle"),
        default=1.0, metavar="SECONDS",
        help="pause after moving to a recording's start pose (default: 1)")
    parser.add_argument(
        "--timeout", type=lambda v: bounded_number(v, minimum=0.1, maximum=2, label="timeout"),
        default=0.5, metavar="SECONDS", help="stop if no command arrives this long (default: 0.5)",
    )
    parser.add_argument("--min-battery", type=int, default=20, metavar="PCT",
                        help="refuse to start below this battery percentage (default: 20)")
    parser.add_argument("--port", default="/dev/ttyAMA0", help="serial port (default: /dev/ttyAMA0)")
    return parser


def read_lines(stream, lines):
    for line in stream:
        lines.put(line)
    lines.put(EOF)


def latest(lines, line):
    """Skip queued commands that went stale while the UART was busy; never skip EOF."""
    while line is not EOF:
        try:
            line = lines.get_nowait()
        except queue.Empty:
            break
    return line


def run(driver, lines, timeout, log=print, first=None, arm=None, barker=None, ramp=None,
        posture=None, teach=None, aligner=None, resetter=None, fullreset=None, fetcher=None):
    """Apply commands until the stream ends; always leaves the robot stopped."""
    try:
        while True:
            if first is not None:
                line, first = first, None
            else:
                try:
                    line = lines.get(timeout=timeout)
                except queue.Empty:
                    if any(driver.current.values()):
                        log("No command for {:.1f}s; stopping.".format(timeout))
                        driver.stop()
                    if ramp is not None:
                        ramp.reset()
                    if arm is not None:
                        arm.idle()
                    if posture is not None:
                        posture.update(0)
                    if teach is not None:
                        teach.watchdog()
                    if aligner is not None:
                        aligner.watchdog()
                    if fetcher is not None:
                        fetcher.watchdog()
                    continue
            line = latest(lines, line)
            if line is EOF:
                log("Command stream ended; stopping.")
                return
            try:
                message = parse_message(line)
                walk, arm_command, bark, kneel = (message.walk, message.arm, message.bark,
                                                  message.kneel)
            except ValueError as exc:
                log(f"Bad command ({exc}); stopping.")
                driver.stop()
                if ramp is not None:
                    ramp.reset()
                if arm is not None:
                    arm.idle()
                if posture is not None:
                    posture.update(0)
                if aligner is not None:
                    aligner.watchdog()
                if fetcher is not None:
                    fetcher.watchdog()
                continue
            fetching = fetcher is not None and fetcher.update(message.fetch, message.manual,
                                                              message.attended)
            fetch_replay = fetching and getattr(fetcher, "replaying", False)
            aligning = fetching and not fetch_replay  # fetch drives the aligner itself
            if aligner is not None and not fetching:
                busy = teach is not None and teach.busy  # A is ignored while recording/replaying
                aligning = aligner.update(message.align and not busy, message.manual)
            replaying = teach is not None and teach.update(message, message.manual,
                                                           blocked=aligning)
            if not aligning and not replaying:  # aligner and replay drive the legs themselves
                if ramp is not None:
                    walk = ramp.apply(walk)
                driver.apply(scale(walk, driver.args))
            elif ramp is not None:
                ramp.reset()
            if aligning:
                replaying = True  # arm/kneel belong to the aligner too
            if resetter is not None:
                resetter.update(message.reset, allowed=not replaying)
            if fullreset is not None:
                busy = replaying or (teach is not None and teach.busy)
                fullreset.update(message.full_reset, allowed=not busy)
            if arm is not None and not replaying:
                if arm_command is None:
                    arm.idle()
                else:
                    arm.update(arm_command)
            if barker is not None:
                barker.update(bark)
            if posture is not None and not replaying:
                posture.update(kneel)
            if hasattr(driver, "refresh"):
                # keep correcting walking drift (also replayed walks); alignment moves the legs
                # itself, so its turns must not be measured as gyro drift
                driver.refresh(external_motion=aligning)
    finally:
        driver.stop()
        try:
            if teach is not None:
                teach.close()
        except Exception as exc:  # a recording problem must never stop the robot standing up
            try:
                log(f"Recording shutdown failed: {exc}")
            except OSError:
                pass
        finally:
            if posture is not None:
                posture.stand()


def make_log(stream, path, now=None, max_bytes=2_000_000):
    """Print to the terminal (stderr; stdout stays unused) and append timestamped lines to a
    session log file on the robot, so runs can be read afterwards over SSH. The file rotates
    to <name>.1 above max_bytes; file errors (e.g. a full disk) never stop the program."""
    now = now or (lambda: time.strftime("%Y-%m-%d %H:%M:%S"))
    handle = None
    try:
        path = Path(path)
        if path.exists() and path.stat().st_size > max_bytes:
            path.replace(path.with_name(path.name + ".1"))
        handle = open(path, "a")
    except OSError:
        pass

    def log(text):
        print(text, file=stream, flush=True)
        if handle is not None:
            try:
                handle.write(f"{now()} {text}\n")
                handle.flush()
            except OSError:
                pass

    return log


def start_viewer(cam, port, log, reference=None):
    """Serve live video on the robot's loopback only; the sender reaches it via an SSH tunnel
    (the tailnet allows SSH to the robot but not other ports)."""
    import video

    overlay = video.reference_overlay(reference) if reference else None
    try:
        viewer = video.VideoServer(cam, video.LOCAL_HOST, port, overlay=overlay)
        viewer.start()
    except (OSError, RuntimeError) as exc:
        log(f"Live video unavailable: {exc}.")
        return None
    log(f"Live video ready on the robot at {viewer.url} (tunnelled to your computer).")
    return viewer


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    teach_dir = Path(args.teach_dir).expanduser()
    try:
        teach_dir.mkdir(parents=True, exist_ok=True)
    except OSError:
        pass
    log = make_log(sys.stderr, teach_dir / "session.log")
    log(f"--- session start: {' '.join(sys.argv[1:]) or '(defaults)'}")
    try:
        from stream_sensors import require_free_port
        require_free_port(args.port)
        xgolib = import_module("xgolib")
    except (ImportError, RuntimeError) as exc:
        parser.exit(2, f"teleop_receiver: {exc}\n")

    for signum in (signal.SIGHUP, signal.SIGTERM):  # SSH teardown: still run stop() below
        signal.signal(signum, lambda *_: sys.exit(1))
    lines = queue.Queue()
    threading.Thread(target=read_lines, args=(sys.stdin, lines), daemon=True).start()
    first = lines.get()  # the sender only starts streaming after the operator confirms
    if first is EOF:
        parser.exit(0, "teleop_receiver: no commands received; robot untouched.\n")

    log("Connecting to the robot (XGO init resets and stands the dog)...")
    dog = xgolib.XGO(port=args.port, version="xgolite")
    driver = Driver(dog, args)
    try:
        battery = dog.read_battery() if hasattr(dog, "read_battery") else None
        if isinstance(battery, (int, float)) and 0 < battery < args.min_battery:
            log(f"Battery {battery}% is below {args.min_battery}%; refusing to drive.")
            return 3
        log(f"Ready (battery {battery}%). Limits x={args.max_x} y={args.max_y} turn={args.max_turn}.")
        if not BARK_SOUND.is_file():
            log(f"Bark sound missing at {BARK_SOUND}; B will do nothing.")
        import camera as camera_module
        import teach as teach_module
        from detect_boxes import detect_boxes

        arm, posture = ArmController(dog, args), Posture(dog, args)
        ramp = Ramp(args.ramp, args.ramp_start)
        if not args.start_standing:
            posture.set_level(1.0)  # start fully kneeled: the box stays in camera view
            log("Kneeling (start position; --start-standing to stay up).")
        cam = camera_module.Camera(0)
        cam.start_async()  # warm up in the background so X is ready without blocking
        recorder = teach_module.Recorder(Path(args.teach_dir).expanduser(), cam, detect_boxes,
                                         args.color, fps=args.record_fps,
                                         min_free_mb=args.min_free_mb,
                                         min_sharpness=args.min_sharpness)
        import heading

        hold = heading.HeadingHold(
            heading.make_yaw_reader(dog.ser), time.monotonic, log=log, sign=args.heading_sign,
            sign_path=Path(args.teach_dir).expanduser() / "heading.json",
            enabled=not args.no_heading_hold, gain=args.heading_gain,
            max_turn=args.heading_max)
        driver = heading.HeadingDriver(driver, hold)
        arm.listener = posture.listener = driver.listener = recorder.on_send
        controller = teach_module.TeachController(
            cam, recorder, teach_module.Replayer(arm, posture, settle=args.replay_settle,
                                                 driver=driver, max_gap=args.replay_max_gap),
            Path(args.teach_dir).expanduser(), lambda: robot_state(driver, arm, posture), log,
            settle=args.settle)
        viewer = start_viewer(
            cam, args.video_port, log,
            reference=lambda: teach_module.load_reference(Path(args.teach_dir).expanduser()),
        ) if args.video_port else None
        import align as align_module

        teach_dir = Path(args.teach_dir).expanduser()
        speeds = {"turn": int(args.align_turn), "forward": int(args.align_walk)}
        measure = alignment_measure(args.color)
        aligner = align_module.GyroAligner(
            legs=LegStepper(alignment_legs_driver(driver), speeds),
            pose=PosePreparer(arm, posture),
            observer=align_module.Observer(cam, measure, teach_module.laplacian_sharpness,
                                           time.monotonic, min_sharpness=args.min_sharpness),
            read_yaw=heading.make_yaw_reader(dog.ser),
            reference=alignment_reference(lambda: teach_module.load_reference(teach_dir),
                                          measure),
            calibration_path=teach_dir / "calibration.json", log=log, clock=time.monotonic,
            motion=args.align_motion, recalibrate=args.recalibrate, settle=args.settle,
            speeds=speeds, tol={"u": args.tol_u, "h": args.tol_h, "v": args.tol_v},
            max_seconds=args.align_max_seconds, max_moves=int(args.align_max_moves))
        fullreset = FullReset(dog, driver, arm, posture, ramp, log)
        fetcher = Fetcher(fullreset, aligner, controller, log, time.monotonic,
                          motion=args.align_motion)
        try:
            run(driver, lines, args.timeout, log, first=first, arm=arm, barker=Barker(log=log),
                ramp=ramp, posture=posture, teach=controller,
                aligner=aligner, resetter=ResetButton(arm, args, log),
                fullreset=fullreset, fetcher=fetcher)
        finally:
            if viewer is not None:
                viewer.close()
    finally:
        driver.stop()
        log("Robot stopped.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
