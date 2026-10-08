"""Teach-and-repeat: record a gamepad pickup (motion + video dataset) and replay it.

A session folder holds the reference view of the box, the arm/claw/kneel targets that
were sent (the replayable motion) and, for training later, camera frames paired with
the gamepad command and robot state at that moment.
"""

import json
import os
import shutil
import sys
import time
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "box_detection"))

STALE_SECONDS = 1.0
START_TIMEOUT = 2.0  # how long X keeps retrying for a usable reference frame
EDGE_PX = 2  # a box this close to the image border may be cut off
MOTION_KINDS = ("arm", "claw", "kneel", "walk")
START_KINDS = ("arm", "claw", "kneel")  # the pose a replay moves to first; walking starts stopped
WALK_AXES = ("x", "y", "yaw")


def box_features(detections, color, frame_shape):
    """(features, None) for exactly one unclipped box of `color`, else (None, reason)."""
    candidates = detections.get(color, [])
    if not candidates:
        return None, f"no {color} box in view"
    if len(candidates) > 1:
        return None, f"{len(candidates)} {color} boxes in view (ambiguous)"
    height, width = frame_shape[:2]
    x, y, w, h = candidates[0]["bbox"]
    if x <= EDGE_PX or y <= EDGE_PX or x + w >= width - EDGE_PX or y + h >= height - EDGE_PX:
        return None, f"{color} box touches the image edge"
    cx, cy = candidates[0]["center"]
    return {"u": round(cx / width, 4), "v": round(cy / height, 4), "w": round(w / width, 4),
            "h": round(h / height, 4), "bbox": [x, y, w, h], "frame_size": [width, height]}, None


def laplacian_sharpness(frame):
    """Variance of the Laplacian: high for crisp edges, low for motion blur."""
    import cv2  # type: ignore[import-not-found]

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def disk_free_mb(path):
    return shutil.disk_usage(str(path)).free // (1024 * 1024)


def encode_jpeg(frame):
    import cv2  # type: ignore[import-not-found]

    ok, data = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
    if not ok:
        raise RuntimeError("JPEG encoding failed")
    return data.tobytes()


def fsync_dir(path):
    """Make a just-created or renamed file's directory entry survive a power cut."""
    fd = os.open(str(path), os.O_RDONLY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def write_json_atomic(path, data):
    temporary = path.with_suffix(path.suffix + ".tmp")
    with open(temporary, "w") as handle:
        json.dump(data, handle, indent=1)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary, path)


class Recorder:
    def __init__(self, root, camera, detect, color, fps=10, min_free_mb=200,
                 clock=time.monotonic, free_mb=None, now=datetime.now, encode=None,
                 sharpness=None, min_sharpness=50):
        self.root = Path(root)
        self.camera = camera
        self.detect = detect
        self.color = color
        self.period = 1.0 / fps
        self.min_free_mb = min_free_mb
        self.clock = clock
        self.free_mb = free_mb or disk_free_mb
        self.now = now
        self.encode = encode or encode_jpeg
        self.sharpness = sharpness or laplacian_sharpness
        self.min_sharpness = min_sharpness
        self.session = None

    @property
    def active(self):
        return self.session is not None

    def low_disk(self):
        existing = self.root if self.root.exists() else self.root.parent
        free = self.free_mb(existing)
        if free < self.min_free_mb:
            return f"only {free} MB free (need {self.min_free_mb})"
        return None

    def fresh_frame(self):
        frame, stamp = self.camera.latest()
        if frame is None:
            return None, None, "no camera frame"
        if self.clock() - stamp > STALE_SECONDS:
            return None, None, "camera frame is stale"
        return frame, stamp, None

    def start(self, state, not_before=None):
        """Begin recording; returns None, or the reason recording could not start.

        not_before: the reference frame must have been captured at or after this time
        (the camera shakes while walking and for a moment after).
        """
        if self.active:
            return "already recording"
        reason = self.low_disk()
        if reason:
            return reason
        frame, stamp, reason = self.fresh_frame()
        if reason:
            return reason
        if not_before is not None and stamp < not_before:
            return "waiting for a frame taken after the robot settled"
        sharpness = self.sharpness(frame)
        if sharpness < self.min_sharpness:
            return (f"camera image is blurry (sharpness {sharpness:.0f} < {self.min_sharpness:.0f}"
                    "; hold still)")
        features, reason = box_features(self.detect(frame), self.color, frame.shape)
        if reason:
            return reason
        files = []
        try:
            self.root.mkdir(parents=True, exist_ok=True)
            name = self.now().strftime("%Y%m%d-%H%M%S")
            session, suffix = self.root / name, 2
            while session.exists():
                session, suffix = self.root / f"{name}-{suffix}", suffix + 1
            (session / "frames").mkdir(parents=True)
            (session / "reference.jpg").write_bytes(self.encode(frame))
            write_json_atomic(session / "reference.json", {
                "version": 1, "color": self.color, "features": features, "state": state,
                "sharpness": round(sharpness, 1),
                "created": self.now().isoformat(timespec="seconds")})
            files.append(open(session / "motion_events.jsonl", "a"))
            files.append(open(session / "dataset.jsonl", "a"))
            fsync_dir(session)
            fsync_dir(self.root)
        except OSError as exc:  # e.g. disk full: no session (it has no motion.json anyway)
            for handle in files:
                handle.close()
            return f"cannot write recording ({exc})"
        self.events_file, self.dataset_file = files
        self.start_state = {key: state[key] for key in START_KINDS}
        self.events = []
        self.frames = 0
        self.started = self.clock()
        self.last_frame_time = None
        self.last_stamp = None
        self.last_sync = self.started
        self.session = session
        return None

    def elapsed(self):
        return round(self.clock() - self.started, 3)

    def on_send(self, kind, value):
        """Listener for ArmController/Posture: one replayable motion event."""
        if not self.active:
            return
        event = {"t": self.elapsed(), kind: value}
        self.events.append(event)
        try:
            self.events_file.write(json.dumps(event) + "\n")
            self.events_file.flush()
            os.fsync(self.events_file.fileno())  # few, small, and the replayable part
        except OSError:
            pass  # the in-memory list still reaches motion.json at stop

    def tick(self, command, state):
        """Called once per received gamepad message; returns a stop reason or None."""
        if not self.active:
            return None
        now = self.clock()
        if (self.last_frame_time is not None
                and now - self.last_frame_time < self.period - 1e-6):  # float-safe
            return None
        reason = self.low_disk()
        if reason:
            self.stopped_summary = self.stop()
            return reason
        frame, stamp, _ = self.fresh_frame()
        if frame is None or stamp == self.last_stamp:
            return None
        name = "frames/{:06d}.jpg".format(self.frames)
        try:
            (self.session / name).write_bytes(self.encode(frame))
            features, _ = box_features(self.detect(frame), self.color, frame.shape)
            moving = any((state.get("walk") or {}).values())  # walking shakes the camera
            self.dataset_file.write(json.dumps({
                "i": self.frames, "t": self.elapsed(), "frame": name, "command": command,
                "state": state, "detection": features, "moving": moving,
                "sharpness": round(self.sharpness(frame), 1)}) + "\n")
            self.dataset_file.flush()
            if now - self.last_sync >= 1.0:  # bound what a power cut can lose
                os.fsync(self.dataset_file.fileno())
                self.last_sync = now
        except OSError as exc:
            self.stopped_summary = self.stop()
            return f"write failed ({exc})"
        self.frames += 1
        self.last_frame_time = now
        self.last_stamp = stamp
        return None

    def stop(self):
        """Finish the session; returns a summary dict, or None if not recording."""
        if not self.active:
            return None
        duration = self.elapsed()
        error = None
        try:
            write_json_atomic(self.session / "motion.json", {
                "version": 1, "color": self.color, "start": self.start_state,
                "duration": duration, "events": self.events})
            fsync_dir(self.session)
        except OSError as exc:
            error = f"motion.json not saved ({exc}); motion_events.jsonl has the events"
        finally:
            for handle in (self.events_file, self.dataset_file):
                try:
                    handle.close()
                except OSError:
                    pass
            session, self.session = self.session, None
        return {"session": str(session), "frames": self.frames, "events": len(self.events),
                "duration": duration, "error": error}


def latest_session(root):
    """Newest session folder with a finished motion.json, or None."""
    root = Path(root)
    if not root.is_dir():
        return None
    finished = sorted(path for path in root.iterdir() if (path / "motion.json").is_file())
    return finished[-1] if finished else None


def valid_walk(steps):
    return (isinstance(steps, dict) and set(steps) == set(WALK_AXES)
            and all(isinstance(steps[axis], int) and not isinstance(steps[axis], bool)
                    for axis in WALK_AXES))


def load_motion(session):
    """Read and validate motion.json; raises ValueError (or OSError) if unusable."""
    try:
        motion = json.loads((Path(session) / "motion.json").read_text())
    except json.JSONDecodeError as exc:
        raise ValueError(f"motion.json is damaged ({exc})") from exc
    if not isinstance(motion, dict) or motion.get("version") != 1:
        raise ValueError("unsupported motion.json version")
    for event in motion.get("events", []):
        kinds = [kind for kind in MOTION_KINDS if kind in event]
        time_ok = isinstance(event.get("t"), (int, float)) and event["t"] >= 0
        if len(kinds) != 1 or not time_ok or set(event) != {"t", kinds[0]}:
            raise ValueError(f"bad motion event {event}")
        if kinds[0] == "walk" and not valid_walk(event["walk"]):
            raise ValueError(f"bad walk event {event}")
    if not isinstance(motion.get("start"), dict):
        raise ValueError("motion.json has no start state")
    return motion


def load_reference(root):
    """{"features", "state"} from the latest finished recording's reference.json, or None."""
    session = latest_session(root)
    if session is None:
        return None
    try:
        data = json.loads((session / "reference.json").read_text())
        return {"features": data["features"], "state": data.get("state") or {}}
    except (OSError, ValueError, KeyError, TypeError):
        return None


class Replayer:
    """Play a recorded motion through the arm/kneel controllers, driven by tick()."""

    def __init__(self, arm, posture, clock=time.monotonic, settle=1.0, driver=None):
        self.arm = arm
        self.posture = posture
        self.driver = driver  # legs: recorded walking is replayed blind
        self.clock = clock
        self.settle = settle
        self.active = False

    def apply(self, kind, value):
        if value is None:
            return
        if kind == "arm":
            self.arm.set_pose(*value)
        elif kind == "claw":
            self.arm.set_claw(value)
        elif kind == "walk":
            if self.driver is not None:
                self.driver.apply(dict(value))
        else:
            self.posture.set_level(value)

    def start(self, motion):
        self.events = sorted(motion["events"], key=lambda event: event["t"])
        self.duration = motion.get("duration", self.events[-1]["t"] if self.events else 0)
        start = motion["start"]
        for kind in ("kneel", "arm", "claw"):  # posture first: it moves the arm base
            self.apply(kind, start.get(kind))
        self.begin = self.clock() + self.settle
        self.index = 0
        self.active = True

    def tick(self):
        if not self.active:
            return None
        elapsed = self.clock() - self.begin + 1e-6  # event times are rounded to 1 ms
        while self.index < len(self.events) and self.events[self.index]["t"] <= elapsed:
            event = self.events[self.index]
            kind = next(kind for kind in MOTION_KINDS if kind in event)
            self.apply(kind, event[kind])
            self.index += 1
        if self.index >= len(self.events):
            self.active = False
            self.stop_legs()
            return "done"
        return None

    def abort(self):
        if self.active:
            self.stop_legs()
        self.active = False

    def stop_legs(self):
        if self.driver is not None:
            self.driver.stop()


class TeachController:
    """Route X (record) and Y (replay) presses; start the camera on first use."""

    def __init__(self, camera, recorder, replayer, root, state, log=print,
                 clock=time.monotonic, settle=0.7):
        self.camera = camera
        self.recorder = recorder
        self.replayer = replayer
        self.root = Path(root)
        self.state = state
        self.log = log
        self.clock = clock
        self.settle = settle
        self.seen = {"record": 0, "replay": 0}
        self.last_moving = None  # last time the legs were walking (the camera shakes)
        self.pending = None  # X pressed; waiting for a steady, usable reference frame

    @property
    def busy(self):
        return self.recorder.active or self.replayer.active or self.pending is not None

    def pressed(self, key, count):
        new = count > self.seen[key]
        self.seen[key] = count  # also resyncs if a counter ever goes backwards
        return new

    def update(self, message, manual, blocked=False):
        """Call once per message. Returns True while a replay owns the arm and kneel.

        blocked: alignment is running; X/Y presses are consumed and ignored.
        """
        if blocked:
            for key, label in (("record", "Record"), ("replay", "Replay")):
                if self.pressed(key, getattr(message, key)):
                    self.log(f"{label} button ignored while aligning.")
            return False
        now = self.clock()
        if any((self.state().get("walk") or {}).values()):
            self.last_moving = now
        if self.pressed("record", message.record):
            self.toggle_recording()
        if self.pressed("replay", message.replay):
            if manual:  # starting would move the arm/kneel only to abort on the same tick
                self.log("Replay needs the sticks centred and LB/RB/LT released; "
                         "release them and press Y again.")
            else:
                self.start_replay()
        if self.pending is not None:
            self.try_start(now)
        if self.replayer.active:
            if manual:
                self.replayer.abort()
                self.log("Replay aborted (manual input).")
                return False
            if self.replayer.tick() == "done":
                self.log("Replay finished.")
            return self.replayer.active
        if self.recorder.active:
            reason = self.recorder.tick(message.raw, self.state())
            if reason:
                self.log(f"Recording stopped: {reason}.")
                self.report(getattr(self.recorder, "stopped_summary", None))
        return False

    def toggle_recording(self):
        if self.replayer.active:
            self.log("Record button ignored during replay.")
            return
        if self.recorder.active:
            self.report(self.recorder.stop())
            return
        if self.pending is not None:
            self.pending = None
            self.log("Recording cancelled.")
            return
        self.camera.start_async()  # never blocks the control loop
        self.pending = {"deadline": None, "reason": None}
        now = self.clock()
        if self.last_moving is not None and now - self.last_moving < self.settle:
            self.log("Waiting for the camera to settle before taking the reference...")
        self.try_start(now)

    def try_start(self, now):
        """Start once the robot has been still for `settle` seconds; retry for a short while."""
        if self.last_moving is not None and now - self.last_moving < self.settle:
            return  # still walking, or the body is still swaying
        if self.pending["deadline"] is None:
            self.pending["deadline"] = now + START_TIMEOUT
        not_before = None if self.last_moving is None else self.last_moving + self.settle
        reason = self.recorder.start(self.state(), not_before=not_before)
        if reason is None:
            self.pending = None
            self.log(f"Recording to {self.recorder.session} (press X again to stop).")
            return
        if reason == "no camera frame":
            reason = self.camera.error or "camera is starting; press X again in a moment"
        if now >= self.pending["deadline"]:
            self.pending = None
            self.log(f"Cannot record: {reason}.")
        elif reason != self.pending["reason"]:
            self.pending["reason"] = reason
            self.log(f"Not ready to record yet: {reason} (retrying).")

    def start_replay(self):
        if self.recorder.active or self.pending is not None:
            self.log("Replay button ignored while recording.")
            return
        session = latest_session(self.root)
        if session is None:
            self.log(f"No recording in {self.root} yet; press X to record one.")
            return
        try:
            motion = load_motion(session)
        except (OSError, ValueError) as exc:
            self.log(f"Cannot replay {session.name}: {exc}.")
            return
        self.replayer.start(motion)
        self.log(f"Replaying {session.name} ({motion.get('duration', 0):.1f} s); any input aborts.")

    def watchdog(self):
        if self.replayer.active:
            self.replayer.abort()
            self.log("Replay aborted (no commands).")

    def report(self, summary):
        if summary is None:
            return
        self.log("Recording saved: {session} ({frames} frames, {events} motion events, "
                 "{duration:.1f} s).".format(**summary))
        if summary.get("error"):
            self.log(f"Warning: {summary['error']}.")

    def close(self):
        """Finish any recording, then always release the camera."""
        self.pending = None
        try:
            self.replayer.abort()
            summary = self.recorder.stop()
            try:
                self.report(summary)
            except OSError:
                pass  # stderr may be gone after an SSH drop
        finally:
            self.camera.close()
