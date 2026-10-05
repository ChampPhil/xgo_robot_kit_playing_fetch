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
EDGE_PX = 2  # a box this close to the image border may be cut off
MOTION_KINDS = ("arm", "claw", "kneel")


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


def disk_free_mb(path):
    return shutil.disk_usage(str(path)).free // (1024 * 1024)


def encode_jpeg(frame):
    import cv2  # type: ignore[import-not-found]

    ok, data = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
    if not ok:
        raise RuntimeError("JPEG encoding failed")
    return data.tobytes()


def write_json_atomic(path, data):
    temporary = path.with_suffix(path.suffix + ".tmp")
    with open(temporary, "w") as handle:
        json.dump(data, handle, indent=1)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary, path)


class Recorder:
    def __init__(self, root, camera, detect, color, fps=10, min_free_mb=200,
                 clock=time.monotonic, free_mb=None, now=datetime.now, encode=None):
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

    def start(self, state):
        """Begin recording; returns None, or the reason recording could not start."""
        if self.active:
            return "already recording"
        reason = self.low_disk()
        if reason:
            return reason
        frame, stamp, reason = self.fresh_frame()
        if reason:
            return reason
        features, reason = box_features(self.detect(frame), self.color, frame.shape)
        if reason:
            return reason
        self.root.mkdir(parents=True, exist_ok=True)
        name = self.now().strftime("%Y%m%d-%H%M%S")
        session, suffix = self.root / name, 2
        while session.exists():
            session, suffix = self.root / f"{name}-{suffix}", suffix + 1
        (session / "frames").mkdir(parents=True)
        (session / "reference.jpg").write_bytes(self.encode(frame))
        write_json_atomic(session / "reference.json", {
            "version": 1, "color": self.color, "features": features, "state": state,
            "created": self.now().isoformat(timespec="seconds")})
        self.events_file = open(session / "motion_events.jsonl", "a")
        self.dataset_file = open(session / "dataset.jsonl", "a")
        self.start_state = {key: state[key] for key in MOTION_KINDS}
        self.events = []
        self.frames = 0
        self.started = self.clock()
        self.last_frame_time = None
        self.last_stamp = None
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
        self.events_file.write(json.dumps(event) + "\n")
        self.events_file.flush()

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
            self.stop()
            return reason
        frame, stamp, _ = self.fresh_frame()
        if frame is None or stamp == self.last_stamp:
            return None
        name = "frames/{:06d}.jpg".format(self.frames)
        (self.session / name).write_bytes(self.encode(frame))
        features, _ = box_features(self.detect(frame), self.color, frame.shape)
        self.dataset_file.write(json.dumps({
            "i": self.frames, "t": self.elapsed(), "frame": name, "command": command,
            "state": state, "detection": features}) + "\n")
        self.dataset_file.flush()
        self.frames += 1
        self.last_frame_time = now
        self.last_stamp = stamp
        return None

    def stop(self):
        """Finish the session; returns a summary dict, or None if not recording."""
        if not self.active:
            return None
        duration = self.elapsed()
        write_json_atomic(self.session / "motion.json", {
            "version": 1, "color": self.color, "start": self.start_state,
            "duration": duration, "events": self.events})
        self.events_file.close()
        self.dataset_file.close()
        summary = {"session": str(self.session), "frames": self.frames,
                   "events": len(self.events), "duration": duration}
        self.session = None
        return summary
