#!/usr/bin/env python3
"""Observe three boxes, simulate stacking, or run a fully taught supervised workcell."""

import argparse
import json
import shlex
import signal
import subprocess
import sys
import time
from pathlib import Path

from vision import COLORS, InventoryTracker, Perception, features, make_marker
from workflow import Tower, validate_order


def positive(value):
    number = int(value)
    if not 1 <= number <= 3000:
        raise argparse.ArgumentTypeError("must be 1..3000")
    return number


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    modes = p.add_mutually_exclusive_group(required=True)
    modes.add_argument("--observe", action="store_true", help="camera/image only; no motors")
    modes.add_argument("--simulate", action="store_true", help="synthetic three-box tower; no hardware")
    modes.add_argument("--check-config", action="store_true", help="validate all taught assets; no hardware")
    modes.add_argument("--import-recording", type=Path, metavar="SESSION",
                       help="copy a teleop recording into the workcell as --asset; no hardware")
    modes.add_argument("--setup-report", action="store_true",
                       help="list everything still missing or invalid in the workcell; no hardware")
    modes.add_argument("--execute", action="store_true", help="supervised live run through SSH heartbeat")
    modes.add_argument("--make-marker", type=Path, metavar="PNG")
    modes.add_argument("--capture-reference", type=Path, metavar="DIRECTORY",
                       help="save one camera-only reference at an already taught pose")
    p.add_argument("--order", nargs=3, choices=COLORS, default=list(COLORS), metavar="COLOR",
                   help="each color once, bottom to top (default: purple orange light-blue)")
    p.add_argument("--source", default="0", help="camera index, video or image; observation only")
    p.add_argument("--frames", type=positive, default=60)
    p.add_argument("--output", type=Path, help="save last annotated observation")
    p.add_argument("--marker-id", type=int, default=0)
    p.add_argument("--config", default=str(Path(__file__).with_name("stack_config.example.json")))
    p.add_argument("--attempts", type=int, choices=range(1, 6), default=3,
                   help="pickup attempts per color including first, default 3")
    p.add_argument("--fail-first", choices=COLORS, help="simulation: first pickup misses")
    p.add_argument("--drop-first", choices=COLORS, help="simulation: first carry drops the box")
    p.add_argument("--uncertain", choices=COLORS, help="simulation: no proof of grip; must stop")
    p.add_argument("--host", default="pi@100.74.30.90")
    p.add_argument("--robot", action="store_true", help=argparse.SUPPRESS)
    p.add_argument("--yes", action="store_true", help="operator already confirmed the printed live checklist")
    p.add_argument("--workdir", default="~/xgo_teach/stacking")
    p.add_argument("--log", type=Path, help="append phase/evidence JSONL (no video recording)")
    p.add_argument("--asset", help="import target: survey, carry-check, pick-<color>, "
                   "retry-<color> or layer-<0..2>/<to-stack|place|inspect|return|recover-drop>")
    p.add_argument("--target", choices=(*COLORS, "marker"), default="marker",
                   help="reference capture target")
    p.add_argument("--pose-file", type=Path,
                   help='reference capture: JSON with explicit {"arm": [x,z], "kneel": level}')
    return p


def emit(event):
    print(json.dumps(event, allow_nan=False), flush=True)


def annotate(frame, report, scene, order):
    import cv2

    result = frame.copy()
    height, width = frame.shape[:2]
    for color, boxes in scene["boxes"].items():
        for box in boxes:
            x, y, w, h = box["bbox"]
            first = (int(x * width), int(y * height))
            last = (int((x + w) * width), int((y + h) * height))
            cv2.rectangle(result, first, last, (0, 255, 0), 2)
            text = "#%d" % (order.index(color) + 1)
            cv2.putText(result, text, (first[0], max(12, first[1] - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    for index, color in enumerate(order):
        track = report.get("tracks", {}).get(color, {})
        text = "#%d %s [%s]" % (index + 1, color, track.get("id", "missing/ambiguous"))
        cv2.putText(result, text, (10, 20 + index * 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    return result


def frames(source, limit):
    import cv2

    path = Path(source)
    if path.suffix.lower() in (".png", ".jpg", ".jpeg", ".bmp", ".webp"):
        frame = cv2.imread(str(path))
        if frame is None:
            raise ValueError("cannot read image " + str(path))
        yield frame, time.monotonic(), True
        return
    cap = cv2.VideoCapture(int(source) if source.isdecimal() else source)
    try:
        if not cap.isOpened():
            raise ValueError("cannot open camera/video; stop competing camera owners first")
        for _ in range(limit):
            ok, frame = cap.read()
            if not ok:
                break
            yield frame, time.monotonic(), False
    finally:
        cap.release()


def observe(args, perception):
    import cv2

    tracker = InventoryTracker(args.order, [0, 0, 1, 1])
    last = None
    for frame, stamp, single in frames(args.source, args.frames):
        if single:
            tracker.count = 1  # explicitly labeled single-image evidence, never live readiness
        scene = perception.observe(frame)
        report = tracker.update(scene, stamp)
        emit(dict(mode="observe", order_bottom_to_top=args.order, inventory=report,
                  marker=scene["marker"], evidence="single_image" if single else "fresh_frames",
                  note="color candidates only; no grasp or floor-plane calibration implied"))
        last = annotate(frame, report, scene, args.order)
    if last is None:
        raise ValueError("no input frames")
    if args.output and not cv2.imwrite(str(args.output), last):
        raise ValueError("could not save observation")
    return 0 if report["ready"] else 2


def capture(args, perception):
    import cv2
    from profiles import pose

    if args.pose_file is None:
        raise ValueError("--pose-file is required; capture never guesses or commands the pose")
    saved = json.loads(args.pose_file.read_text())
    saved = saved.get("state", saved)
    state = pose({key: saved.get(key) for key in ("arm", "kneel")})
    folder = args.capture_reference
    if folder.exists():
        raise ValueError("reference output already exists")
    latest = None
    for frame, _, _ in frames(args.source, min(args.frames, 30)):
        latest = frame
    if latest is None:
        raise ValueError("no camera frame")
    scene = perception.observe(latest)
    if args.target == "marker":
        measured = scene["marker"]
    else:
        boxes = scene["boxes"][args.target]
        measured = features(boxes[0]) if len(boxes) == 1 else None
    if measured is None:
        raise ValueError("capture requires one unambiguous " + args.target)
    folder.mkdir(parents=True)
    if not cv2.imwrite(str(folder / "reference.jpg"), latest):
        raise ValueError("cannot save reference image")
    data = {"version": 1, "target": args.target, "state": state, "features": measured,
            "image": "reference.jpg", "marker_id": args.marker_id}
    (folder / "reference.json").write_text(json.dumps(data, indent=2) + "\n")
    emit(dict(mode="capture", reference=str(folder / "reference.json"), motion_sent=False))
    return 0


def remote(args, mode):
    parts = ["python3", "-u", "scripts/stacking/stack_boxes.py", mode, "--config", args.config,
             "--marker-id", str(args.marker_id), "--order", *args.order]
    if mode == "--execute":
        parts += ["--robot", "--attempts", str(args.attempts), "--workdir", args.workdir]
    return "cd ~/project/xgo_robot_kit_playing_fetch && exec " + shlex.join(parts)


def launch(args):
    base = ["ssh", "-T", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8",
            "-o", "ServerAliveInterval=2", "-o", "ServerAliveCountMax=2", args.host]
    # On the Pi config paths are resolved by the Pi, not the Mac. Validate everything
    # remotely before asking the operator to start anything physical.
    result = subprocess.run(base + [remote(args, "--check-config")])
    if result.returncode:
        return result.returncode
    print("Live stacking uses camera 0 and UART /dev/ttyAMA0. XGO startup resets/stands. "
          "It will walk/turn, move the arm/claw, and release boxes at taught layer poses. "
          "Floor only; unplug charger; battery >=20%; empty gripper; marker fixed and clear "
          "taught routes. Stay beside the power switch. Ctrl+C cancels.", file=sys.stderr)
    if not args.yes and input("Confirm all conditions and start? [yes/no] ").strip().lower() != "yes":
        return 2
    process = subprocess.Popen(base + [remote(args, "--execute")], stdin=subprocess.PIPE, text=True)
    try:
        while process.poll() is None:
            process.stdin.write('{"heartbeat":true}\n')
            process.stdin.flush()
            time.sleep(0.05)
    finally:
        try:
            process.stdin.close()
        except OSError:
            pass
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
    return process.returncode


def run_robot(args, plan, perception):
    from robot import Heartbeat, Robot

    path = args.log or Path(args.workdir).expanduser() / ("events-" + time.strftime('%Y%m%d-%H%M%S') + '.jsonl')
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a") as handle:
        def log(event):
            event = dict(event, time=time.time(), mode="live")
            handle.write(json.dumps(event, allow_nan=False) + "\n")
            handle.flush()
            emit(event)

        backend = Robot(plan, perception, Heartbeat(sys.stdin), log, args.workdir)
        try:
            result = Tower(backend, args.order, args.attempts, log).run()
            emit(dict(result, mode="live", verification="vision; no force sensor"))
        finally:
            backend.close()
    return 0


def main(argv=None):
    p = parser()
    args = p.parse_args(argv)
    try:
        args.order = validate_order(args.order)
        if args.make_marker:
            make_marker(args.make_marker, args.marker_id)
            emit({"marker": str(args.make_marker), "dictionary": "DICT_4X4_50", "id": args.marker_id})
            return 0
        if args.execute and not args.robot:
            # The default example is intentionally disabled and paths differ on Mac/Pi.
            if args.config == str(Path(__file__).with_name("stack_config.example.json")):
                raise ValueError("--execute needs the path to a taught configuration ON THE PI")
            return launch(args)
        perception = Perception(args.marker_id)
        if args.observe:
            return observe(args, perception)
        if args.capture_reference:
            return capture(args, perception)
        if args.simulate:
            from simulation import Simulation

            backend = Simulation(args.fail_first, args.drop_first, args.uncertain)
            result = Tower(backend, args.order, args.attempts,
                           lambda event: emit(dict(event, mode="simulation"))).run()
            emit(dict(result, mode="simulation", physical_success=False))
            return 0
        if args.import_recording:
            from importer import import_recording

            if not args.asset:
                raise ValueError("--import-recording needs --asset")
            emit(dict(import_recording(args.import_recording, args.asset, args.config,
                                       perception), mode="import", motion_sent=False))
            return 0
        if args.setup_report:
            from profiles import setup_report

            report = setup_report(args.config, perception)
            for item in report["items"]:
                if item["status"] != "ok":
                    print("%-11s %s%s" % (item["status"], item["item"],
                                          ": " + item["detail"] if item["detail"] else ""),
                          file=sys.stderr)
            emit(dict(report, mode="setup-report", motion_sent=False))
            return 0 if report["ready"] else 2
        from profiles import Plan

        plan = Plan(args.config, perception)  # all assets and validation before motor import
        if args.check_config:
            emit({"status": "configured", "marker_id": plan.marker_id,
                  "order_bottom_to_top": args.order, "motion_sent": False})
            return 0
        def cancel(signum, _frame):
            raise KeyboardInterrupt("signal " + str(signum))

        for sig in (signal.SIGTERM, signal.SIGHUP):
            signal.signal(sig, cancel)
        return run_robot(args, plan, perception)
    except (ValueError, OSError, KeyError, TypeError, RuntimeError, EOFError, KeyboardInterrupt) as exc:
        emit({"status": "stopped", "reason": str(exc) or "operator cancelled", "mode":
              "simulation" if args.simulate else "live" if args.execute else "preflight"})
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
