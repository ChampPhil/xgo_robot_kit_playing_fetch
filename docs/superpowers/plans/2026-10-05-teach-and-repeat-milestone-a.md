# Teach-and-Repeat Milestone A (record, replay, live video) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Press X to record a gamepad pickup (replay motion + video/state dataset), press Y to replay it, and optionally watch the robot camera live with `gamepad_sender.py --video`.

**Architecture:** `teleop_receiver.py` stays the single owner of the UART and becomes the camera owner. A background `Camera` thread always holds the newest frame; `Recorder`/`Replayer`/`TeachController` in `teach.py` and an MJPEG server in `video.py` consume it. The arm and kneel controllers gain setters (for replay) and a send listener (for recording), so replayed and manual motion share one state.

**Tech Stack:** Python 3.9 on the robot (system `python3`, OpenCV 4.7, xgolib 2023), Python 3.14 + pygame-ce on the Mac, `unittest`, `http.server`.

**Spec:** `docs/superpowers/specs/2026-10-05-teach-and-repeat-design.md` (milestone A only; alignment is a later plan). Live video was requested after the spec; its requirements are in Task 6.

## Global Constraints

- Robot code must parse as Python 3.9: no `match`, no `X | Y` types, no new robot dependencies (only stdlib, OpenCV, numpy, pyserial, xgolib already installed).
- Buttons: **X** toggles recording, **Y** replays the latest pickup, B still barks; A is reserved for alignment (not in this plan).
- Replayed motion is arm, claw and kneel only; walking is never replayed. The dataset records everything commanded, walking included.
- Recordings live on the robot only, in `~/xgo_teach/<YYYYmmdd-HHMMSS>/` (outside the git checkout).
- Disk guard: refuse to start and stop recording when free space on the recordings filesystem is below `--min-free-mb` (default 200).
- Dataset frames: JPEG quality 80 at the camera's native size, `--record-fps` default 10; each `dataset.jsonl` line and `motion_events.jsonl` line is flushed as written.
- `motion.json` is written atomically (temp file, fsync, `os.replace`).
- Live video binds only to the robot's Tailscale IPv4 address, port `--video-port` (sender default 8090 when `--video` is given); never `0.0.0.0`.
- Every new hardware-facing failure (camera busy, no recording, bad file, no Tailscale address) is logged and leaves driving working.
- Run tests with: `.venv/bin/python -m unittest discover -s scripts/teleop -p 'test_*.py'`.

## Review Focus

- Power cut mid-recording: frames and dataset lines written so far, and `motion_events.jsonl`, stay readable even though `motion.json` was never written → test in Task 4 (`test_events_are_on_disk_before_stop`).
- Camera busy (old camera stream still running) when X is pressed or `--video` is used: a clear message, no crash, driving continues → tests in Task 5 (`test_record_reports_camera_failure`) and Task 6 (`test_video_start_failure_is_reported`).
- Y with no recordings, or a truncated/corrupt `motion.json` from a power cut: reported, nothing moves → Task 5 (`test_replay_without_recordings`, `test_replay_with_corrupt_motion_file`).
- The same camera frame handed out twice (camera thread stalled): it must not be written to the dataset twice → Task 4 (`test_same_frame_is_not_written_twice`).
- Session ends while recording (START, EOF, SSH drop): files are finalised before the robot stands up → Task 5 (`test_run_finalises_recording_on_exit`).

---

## File Structure

| File | Status | Responsibility |
| --- | --- | --- |
| `scripts/teleop/camera.py` | create | Background frame grabber with `latest() -> (frame, stamp)`. |
| `scripts/teleop/teach.py` | create | `box_features`, `Recorder`, `load_motion`, `latest_session`, `Replayer`, `TeachController`. |
| `scripts/teleop/video.py` | create | `tailscale_ipv4()`, `VideoServer` (MJPEG over HTTP from a `Camera`). |
| `scripts/teleop/teleop_receiver.py` | modify | `Message` parsing (record/replay counters), controller setters + listener, flags, wiring in `run()`/`main()`. |
| `scripts/teleop/gamepad_sender.py` | modify | X/Y counters, `--video` (passes `--video-port`, opens the browser). |
| `scripts/teleop/test_camera.py`, `test_teach.py`, `test_video.py` | create | Unit tests for the new modules. |
| `scripts/teleop/test_teleop.py` | modify | Protocol, setter/listener and `run()` wiring tests. |
| `scripts/teleop/README.md` | modify | Controls, recording layout, video. |

---

### Task 1: Message protocol with record/replay counters

**Files:**
- Modify: `scripts/teleop/teleop_receiver.py` (`parse_message`, `parse_command`, `run`)
- Modify: `scripts/teleop/gamepad_sender.py` (`read_sticks`, `make_message`, `stream`)
- Test: `scripts/teleop/test_teleop.py`

**Interfaces:**
- Produces: `teleop_receiver.Message` — `NamedTuple(walk: dict, arm: Optional[dict], bark: int, kneel: int, record: int, replay: int, raw: dict)`; `parse_message(line) -> Message`.
- Produces: sender message keys `"record"` and `"replay"` (non-negative press counters); `make_message(state, arming, deadzone, barks, records=0, replays=0)`.

- [ ] **Step 1: Write the failing tests** (append to `SenderTests` and `KneelTests`'s neighbour `ReceiverTests` in `test_teleop.py`; update the existing unpacking test)

In `ArmBarkRampTests.test_arm_mode_overrides_walking` replace `walk, arm, _, _ = receiver.parse_message(` with:

```python
        message = receiver.parse_message(arm_line(ax=0.5, x=1))
        walk, arm = message.walk, message.arm
```

and delete the old two-line unpack. Add to `SenderTests`:

```python
    def test_x_and_y_press_counters_are_sent(self):
        message = sender.make_message(self.state(), sender.Arming(deadman=True), 0.1, 0,
                                      records=3, replays=2)
        self.assertEqual((message["record"], message["replay"]), (3, 2))
```

Add to `ReceiverTests`:

```python
    def test_message_carries_counters_and_raw(self):
        message = receiver.parse_message(line(record=2, replay=1))
        self.assertEqual((message.record, message.replay), (2, 1))
        self.assertEqual(message.raw["record"], 2)
        self.assertEqual(receiver.parse_message(line()).record, 0)

    def test_bad_counters_are_rejected(self):
        for key in ("record", "replay"):
            for bad in (-1, 1.5, True, "1"):
                with self.assertRaises(ValueError):
                    receiver.parse_message(line(**{key: bad}))
```

In `KneelTests.test_kneel_parses_and_rejects_bad_values` replace `[3]` with `.kneel` (two places). In the sender pipe test's expected dict (`test_stream_writes_through_ssh_pipe`) add `"record": 0, "replay": 0`; in `test_rb_switches_sticks_to_arm_and_disables_walking` add the same two keys to the expected dict. In `SenderTests.state` add `x=False, y=False` to the defaults, and add `CONTROLLER_BUTTON_X=2, CONTROLLER_BUTTON_Y=3` to the fake `pygame` namespace in the pipe test.

- [ ] **Step 2: Run tests to verify they fail**

Run: `.venv/bin/python -m unittest discover -s scripts/teleop -p 'test_*.py'`
Expected: errors such as `AttributeError: 'tuple' object has no attribute 'walk'` and `KeyError: 'record'`.

- [ ] **Step 3: Implement**

In `teleop_receiver.py` add `from typing import NamedTuple, Optional` to the imports, then replace `parse_message`/`parse_command`:

```python
class Message(NamedTuple):
    walk: dict
    arm: Optional[dict]
    bark: int
    kneel: int
    record: int
    replay: int
    raw: dict


def counter(message, key):
    value = message.get(key, 0)
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{key} must be a non-negative integer")
    return value


def parse_message(line):
    """Parse one JSON command line into a Message. Raises ValueError on bad input.

    walk is {axis: -1..1}, all zero unless enable is true and arm mode is off.
    arm is {ax, az, claw: -1..1} in arm mode, else None. bark/record/replay are press
    counters. kneel is 1 (kneel down), -1 (stand up) or 0.
    """
    message = json.loads(line)
    if not isinstance(message, dict):
        raise ValueError("command must be a JSON object")
    bark, record, replay = (counter(message, key) for key in ("bark", "record", "replay"))
    kneel = message.get("kneel", 0)
    if isinstance(kneel, bool) or kneel not in (-1, 0, 1):
        raise ValueError("kneel must be -1, 0 or 1")
    arm = None
    if message.get("arm") is True:
        arm = {axis: axis_value(message, axis) for axis in ARM_AXES}
    walk = dict.fromkeys(AXES, 0.0)
    if message.get("enable") is True and arm is None:
        walk = {axis: axis_value(message, axis) for axis in AXES}
    return Message(walk, arm, bark, kneel, record, replay, message)


def parse_command(line):
    """Walking part of a message only (see parse_message)."""
    return parse_message(line).walk
```

In `run()` replace `walk, arm_command, bark, kneel = parse_message(line)` with:

```python
                message = parse_message(line)
                walk, arm_command, bark, kneel = (message.walk, message.arm, message.bark,
                                                  message.kneel)
```

In `gamepad_sender.py`: in `read_sticks` add `"x": button(pygame.CONTROLLER_BUTTON_X), "y": button(pygame.CONTROLLER_BUTTON_Y),` to the returned dict. Change `make_message`'s signature and end:

```python
def make_message(state, arming, deadzone, barks, records=0, replays=0):
```

```python
    command["bark"] = barks
    command["record"] = records  # X presses: toggle recording
    command["replay"] = replays  # Y presses: replay the latest recording
```

In `stream()`, replace the bark edge-detection block with a generic one:

```python
    presses = {"b": 0, "x": 0, "y": 0}
    was_down = dict.fromkeys(presses, False)
```

(before the loop, replacing `barks, b_was_down = 0, False`) and inside the loop:

```python
        for button in presses:  # counters, so a skipped line cannot lose a press
            if state[button] and not was_down[button]:
                presses[button] += 1
            was_down[button] = state[button]
        command = make_message(state, arming, args.deadzone, presses["b"],
                               records=presses["x"], replays=presses["y"])
```

Update the module docstring line `B barks. START or Ctrl+C quits.` to `B barks. X starts/stops recording a pickup, Y replays it. START or Ctrl+C quits.` and the printed hint in `main()` to include `X records, Y replays;`.

- [ ] **Step 4: Run tests to verify they pass**

Run: `.venv/bin/python -m unittest discover -s scripts/teleop -p 'test_*.py'`
Expected: `OK`.

- [ ] **Step 5: Commit**

```bash
git add scripts/teleop/teleop_receiver.py scripts/teleop/gamepad_sender.py scripts/teleop/test_teleop.py
git commit -m "Add X/Y record and replay press counters to the teleop protocol"
```

---

### Task 2: Controller setters, state and send listener

**Files:**
- Modify: `scripts/teleop/teleop_receiver.py` (`ArmController`, `Posture`)
- Test: `scripts/teleop/test_teleop.py`

**Interfaces:**
- Produces: `ArmController.set_pose(x, z)`, `ArmController.set_claw(value)`, `ArmController.listener` (callable `(kind, value)` or `None`; kinds `"arm"` with `[x, z]` ints, `"claw"` with an int).
- Produces: `Posture.set_level(level)`, `Posture.listener` (kind `"kneel"`, value = level rounded to 4 places).
- Produces: `robot_state(driver, arm, posture) -> {"arm": [x, z] | None, "claw": int | None, "kneel": float, "walk": {"x", "y", "yaw"}}` (module-level function in `teleop_receiver.py`).

- [ ] **Step 1: Write the failing tests** (add to `ArmBarkRampTests` and `KneelTests`)

```python
    def test_set_pose_and_claw_send_and_notify(self):
        heard = []
        self.arm.listener = lambda kind, value: heard.append((kind, value))
        self.arm.set_pose(100, 20)
        self.arm.set_claw(200)
        self.arm.set_pose(100, 20)  # unchanged: nothing resent
        self.assertEqual(self.dog.calls, [("arm", 100, 20), ("claw", 200)])
        self.assertEqual(heard, [("arm", [100, 20]), ("claw", 200)])
        self.tick(0, az=1)
        self.tick(0.1, az=1)  # manual control continues from the set pose
        self.assertEqual(self.dog.calls[-1], ("arm", 100, 26))

    def test_manual_moves_notify_listener(self):
        heard = []
        self.arm.listener = lambda kind, value: heard.append((kind, value))
        self.tick(0, claw=1)
        self.assertEqual(heard, [("claw", 128)])
```

```python
    def test_set_level_sends_and_notifies(self):
        heard = []
        self.posture.listener = lambda kind, value: heard.append((kind, value))
        self.posture.set_level(0.5)
        self.assertEqual(self.dog.calls, [("attitude", "p", 5.0), ("translation", "z", 77.5)])
        self.assertEqual(heard, [("kneel", 0.5)])
        self.posture.set_level(7)  # clamped
        self.assertEqual(self.dog.calls[-2:], [("attitude", "p", 10), ("translation", "z", 70)])

    def test_robot_state_snapshot(self):
        arm = receiver.ArmController(self.dog, self.args, clock=self.clock)
        driver = receiver.Driver(self.dog, self.args)
        self.assertEqual(receiver.robot_state(driver, arm, self.posture),
                         {"arm": None, "claw": None, "kneel": 0.0,
                          "walk": {"x": 0, "y": 0, "yaw": 0}})
        arm.set_pose(90, 10)
        self.posture.set_level(0.25)
        state = receiver.robot_state(driver, arm, self.posture)
        self.assertEqual((state["arm"], state["kneel"]), ([90, 10], 0.25))
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `.venv/bin/python -m unittest discover -s scripts/teleop -p 'test_*.py'`
Expected: `AttributeError: 'ArmController' object has no attribute 'listener'` (and similar).

- [ ] **Step 3: Implement**

In `ArmController.__init__` add `self.listener = None`. Replace the sending block at the end of `update()` with `self.send()` and add:

```python
    def set_pose(self, x, z):
        """Jump the arm target (used by replay); manual control continues from here."""
        self.pose = [float(x), float(z)]
        self.idle()
        self.send()

    def set_claw(self, value):
        self.grip = float(clamp(value, ARM_LIMITS["claw"]))
        self.send()

    def send(self):
        if self.pose is not None:
            target = (int(round(self.pose[0])), int(round(self.pose[1])))
            if target != self.sent["arm"]:
                self.dog.arm(*target)
                self.sent["arm"] = target
                if self.listener:
                    self.listener("arm", list(target))
        if self.grip is not None:
            target = int(round(self.grip))
            if target != self.sent["claw"]:
                self.dog.claw(target)
                self.sent["claw"] = target
                if self.listener:
                    self.listener("claw", target)
```

In `Posture.__init__` add `self.listener = None`; add

```python
    def set_level(self, level):
        """Jump to a kneel level (used by replay)."""
        self.level = clamp(float(level), (0.0, 1.0))
        self.last = None
        self.send()
```

and at the end of `Posture.send()`, inside the `if (pitch, height) != self.sent:` block, after `self.sent = (pitch, height)`:

```python
            if self.listener:
                self.listener("kneel", round(self.level, 4))
```

Add the module-level function after `Posture`:

```python
def robot_state(driver, arm, posture):
    """Snapshot of what has been commanded, for recordings."""
    return {"arm": list(arm.sent["arm"]) if arm.sent["arm"] else None,
            "claw": arm.sent["claw"], "kneel": round(posture.level, 4),
            "walk": dict(driver.current)}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `.venv/bin/python -m unittest discover -s scripts/teleop -p 'test_*.py'`
Expected: `OK` (all earlier tests too: `update()` now delegates to `send()` with the same calls).

- [ ] **Step 5: Commit**

```bash
git add scripts/teleop/teleop_receiver.py scripts/teleop/test_teleop.py
git commit -m "Add arm/kneel setters and a send listener for recording and replay"
```

---

### Task 3: Background camera

**Files:**
- Create: `scripts/teleop/camera.py`
- Test: `scripts/teleop/test_camera.py`

**Interfaces:**
- Produces: `camera.Camera(source=0, open_capture=None)`; `.start(timeout=3.0)` (raises `RuntimeError` with a readable reason); `.latest() -> (frame | None, stamp | None)` with `stamp` from `time.monotonic()`; `.running` (bool); `.close()`. `start()` on a running camera is a no-op.

- [ ] **Step 1: Write the failing tests**

```python
"""Offline checks for the background camera; no real camera required."""

import time
import unittest

import numpy as np

import camera


class FakeCapture:
    def __init__(self, opened=True, frames=True):
        self.opened, self.frames, self.released, self.count = opened, frames, False, 0

    def isOpened(self):
        return self.opened

    def read(self):
        time.sleep(0.005)
        if not self.frames:
            return False, None
        self.count += 1
        return True, np.full((4, 4, 3), self.count % 255, dtype=np.uint8)

    def release(self):
        self.released = True


class CameraTests(unittest.TestCase):
    def test_latest_returns_newest_frame_with_timestamp(self):
        capture = FakeCapture()
        cam = camera.Camera(open_capture=lambda source: capture)
        cam.start()
        try:
            frame, stamp = cam.latest()
            self.assertEqual(frame.shape, (4, 4, 3))
            self.assertLessEqual(stamp, time.monotonic())
            time.sleep(0.05)
            self.assertGreater(cam.latest()[1], stamp)
        finally:
            cam.close()
        self.assertTrue(capture.released)
        self.assertFalse(cam.running)

    def test_unopenable_camera_raises_readable_error(self):
        cam = camera.Camera(open_capture=lambda source: FakeCapture(opened=False))
        with self.assertRaisesRegex(RuntimeError, "cannot open camera"):
            cam.start()
        self.assertFalse(cam.running)

    def test_camera_without_frames_raises_and_releases(self):
        capture = FakeCapture(frames=False)
        cam = camera.Camera(open_capture=lambda source: capture)
        with self.assertRaisesRegex(RuntimeError, "no frames"):
            cam.start(timeout=0.2)
        self.assertTrue(capture.released)

    def test_start_twice_is_harmless(self):
        opened = []
        cam = camera.Camera(open_capture=lambda source: opened.append(1) or FakeCapture())
        cam.start()
        cam.start()
        cam.close()
        self.assertEqual(len(opened), 1)

    def test_latest_before_start(self):
        self.assertEqual(camera.Camera().latest(), (None, None))


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `.venv/bin/python -m unittest discover -s scripts/teleop -p 'test_camera.py'`
Expected: `ModuleNotFoundError: No module named 'camera'`.

- [ ] **Step 3: Implement `scripts/teleop/camera.py`**

```python
"""Background camera reader that always hands out the newest frame and its capture time.

Reading continuously in a thread avoids OpenCV's buffered (stale) frames; callers compare
the timestamp with time.monotonic() to reject old images.
"""

import threading
import time


def open_cv2_capture(source):
    import cv2  # type: ignore[import-not-found]

    return cv2.VideoCapture(source)


class Camera:
    def __init__(self, source=0, open_capture=None):
        self.source = source
        self.open_capture = open_capture or open_cv2_capture
        self.lock = threading.Lock()
        self.frame = None
        self.stamp = None
        self.capture = None
        self.thread = None
        self.running = False

    def start(self, timeout=3.0):
        if self.running:
            return
        self.capture = self.open_capture(self.source)
        if self.capture is None or not self.capture.isOpened():
            self.capture = None
            raise RuntimeError(f"cannot open camera {self.source} "
                               "(is the camera stream or another program using it?)")
        self.running = True
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.latest()[0] is not None:
                return
            time.sleep(0.02)
        self.close()
        raise RuntimeError(f"camera {self.source} opened but produced no frames")

    def loop(self):
        while self.running:
            ok, frame = self.capture.read()
            if ok and frame is not None:
                with self.lock:
                    self.frame, self.stamp = frame, time.monotonic()
            else:
                time.sleep(0.05)

    def latest(self):
        with self.lock:
            return self.frame, self.stamp

    def close(self):
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=1.0)
            self.thread = None
        if self.capture is not None:
            self.capture.release()
            self.capture = None
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `.venv/bin/python -m unittest discover -s scripts/teleop -p 'test_camera.py'`
Expected: `OK` (5 tests).

- [ ] **Step 5: Commit**

```bash
git add scripts/teleop/camera.py scripts/teleop/test_camera.py
git commit -m "Add background camera reader with timestamped latest frame"
```

---

### Task 4: Recorder (reference, motion events, video dataset, disk guard)

**Files:**
- Create: `scripts/teleop/teach.py`
- Test: `scripts/teleop/test_teach.py`

**Interfaces:**
- Consumes: `Camera.latest()` (Task 3); `detect_boxes.detect_boxes(frame) -> {color: [{"bbox": [x, y, w, h], "center": [cx, cy], "area_px": int}]}`; the state dict from `robot_state` (Task 2).
- Produces: `teach.box_features(detections, color, frame_shape) -> (features | None, reason | None)`; `teach.Recorder(root, camera, detect, color, fps=10, min_free_mb=200, clock=time.monotonic, free_mb=None, now=datetime.now, encode=None)` with `.active`, `.start(state) -> reason | None`, `.on_send(kind, value)`, `.tick(command, state) -> stop_reason | None`, `.stop() -> summary dict | None`. `STALE_SECONDS = 1.0`.
- Session files: `reference.jpg`, `reference.json` (`{"version": 1, "color", "features", "state", "created"}`), `motion_events.jsonl` (`{"t", <kind>: value}` lines), `motion.json` (`{"version": 1, "color", "start": {"arm", "claw", "kneel"}, "duration", "events": [...]}`), `frames/000000.jpg`…, `dataset.jsonl` (`{"i", "t", "frame", "command", "state", "detection"}` lines).

- [ ] **Step 1: Write the failing tests** (`scripts/teleop/test_teach.py`)

```python
"""Offline checks for teach-and-repeat; no robot, camera or GPU required."""

import json
import tempfile
import unittest
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

import teach

BOX = {"bbox": [140, 100, 40, 40], "center": [160, 120], "area_px": 1600}


def detections(*boxes, color="purple"):
    found = {"purple": [], "orange": [], "light-blue": []}
    found[color] = list(boxes)
    return found


class FakeClock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


class FakeCamera:
    def __init__(self, clock):
        self.clock = clock
        self.frame = np.zeros((240, 320, 3), dtype=np.uint8)
        self.stamp = clock.now

    def latest(self):
        return self.frame, self.stamp

    def new_frame(self):
        self.stamp = self.clock.now


def purple_frame():
    hsv = np.zeros((240, 320, 3), dtype=np.uint8)
    cv2.rectangle(hsv, (140, 100), (180, 140), (140, 200, 210), -1)
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


STATE = {"arm": [80, 30], "claw": 128, "kneel": 0.0, "walk": {"x": 0, "y": 0, "yaw": 0}}


class FeatureTests(unittest.TestCase):
    def test_features_of_one_box_from_the_real_detector(self):
        import detect_boxes

        frame = purple_frame()
        features, reason = teach.box_features(detect_boxes.detect_boxes(frame), "purple",
                                              frame.shape)
        self.assertIsNone(reason)
        self.assertAlmostEqual(features["u"], 0.5, places=1)
        self.assertAlmostEqual(features["h"], 41 / 240, places=2)
        self.assertEqual(features["frame_size"], [320, 240])

    def test_missing_ambiguous_and_clipped_boxes_are_rejected(self):
        shape = (240, 320, 3)
        self.assertIn("no purple", teach.box_features(detections(), "purple", shape)[1])
        self.assertIn("ambiguous", teach.box_features(detections(BOX, BOX), "purple", shape)[1])
        edge = {"bbox": [0, 100, 40, 40], "center": [20, 120], "area_px": 1600}
        self.assertIn("edge", teach.box_features(detections(edge), "purple", shape)[1])


class RecorderTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name) / "xgo_teach"
        self.clock = FakeClock()
        self.camera = FakeCamera(self.clock)
        self.free = [1000]
        self.seen = detections(BOX)
        self.recorder = teach.Recorder(
            self.root, self.camera, lambda frame: self.seen, "purple", fps=10, min_free_mb=200,
            clock=self.clock, free_mb=lambda path: self.free[0],
            now=lambda: datetime(2026, 10, 5, 12, 0, 0), encode=lambda frame: b"jpeg")

    def tearDown(self):
        self.tmp.cleanup()

    def session(self):
        return self.root / "20261005-120000"

    def test_start_writes_reference_and_refuses_without_a_unique_box(self):
        self.seen = detections()
        self.assertIn("no purple", self.recorder.start(STATE))
        self.assertFalse(self.recorder.active)
        self.seen = detections(BOX)
        self.assertIsNone(self.recorder.start(STATE))
        reference = json.loads((self.session() / "reference.json").read_text())
        self.assertEqual(reference["color"], "purple")
        self.assertEqual(reference["state"], STATE)
        self.assertEqual((self.session() / "reference.jpg").read_bytes(), b"jpeg")

    def test_start_refuses_stale_frame_and_low_disk(self):
        self.clock.now += 5
        self.assertIn("stale", self.recorder.start(STATE))
        self.camera.new_frame()
        self.free[0] = 100
        self.assertIn("MB free", self.recorder.start(STATE))
        self.assertFalse(self.root.exists() and any(self.root.iterdir()))

    def test_motion_events_and_atomic_motion_file(self):
        self.recorder.start(STATE)
        self.clock.now += 0.5
        self.recorder.on_send("arm", [90, 30])
        self.clock.now += 0.25
        self.recorder.on_send("claw", 200)
        self.recorder.on_send("kneel", 0.5)
        self.clock.now += 1
        summary = self.recorder.stop()
        motion = json.loads((self.session() / "motion.json").read_text())
        self.assertEqual(motion["start"], {"arm": [80, 30], "claw": 128, "kneel": 0.0})
        self.assertEqual(motion["events"], [{"t": 0.5, "arm": [90, 30]},
                                            {"t": 0.75, "claw": 200},
                                            {"t": 0.75, "kneel": 0.5}])
        self.assertEqual(motion["duration"], 1.75)
        self.assertEqual(summary["events"], 3)
        self.assertFalse(self.recorder.active)
        self.assertEqual(list(self.session().glob("*.tmp")), [])

    def test_events_are_on_disk_before_stop(self):
        self.recorder.start(STATE)
        self.recorder.on_send("arm", [90, 30])
        lines = (self.session() / "motion_events.jsonl").read_text().splitlines()
        self.assertEqual(json.loads(lines[0]), {"t": 0.0, "arm": [90, 30]})
        self.assertFalse((self.session() / "motion.json").exists())

    def test_events_ignored_when_not_recording(self):
        self.recorder.on_send("arm", [90, 30])
        self.assertFalse(self.root.exists())

    def test_dataset_frames_at_fps_with_command_state_and_detection(self):
        self.recorder.start(STATE)
        for _ in range(10):  # 20 Hz ticks for 0.5 s at 10 fps -> 5 frames
            self.recorder.tick({"x": 1}, STATE)
            self.clock.now += 0.05
            self.camera.new_frame()
        lines = [json.loads(line)
                 for line in (self.session() / "dataset.jsonl").read_text().splitlines()]
        self.assertEqual(len(lines), 5)
        self.assertEqual(lines[0]["frame"], "frames/000000.jpg")
        self.assertEqual(lines[0]["command"], {"x": 1})
        self.assertEqual(lines[0]["state"], STATE)
        self.assertEqual(lines[0]["detection"]["bbox"], BOX["bbox"])
        self.assertTrue((self.session() / "frames" / "000004.jpg").exists())
        self.assertEqual(self.recorder.stop()["frames"], 5)

    def test_same_frame_is_not_written_twice(self):
        self.recorder.start(STATE)
        self.recorder.tick({}, STATE)
        self.clock.now += 0.2  # time passes but the camera delivered nothing new
        self.recorder.tick({}, STATE)
        self.assertEqual(len((self.session() / "dataset.jsonl").read_text().splitlines()), 1)

    def test_lost_box_is_recorded_as_null_detection(self):
        self.recorder.start(STATE)
        self.seen = detections()
        self.recorder.tick({}, STATE)
        line = json.loads((self.session() / "dataset.jsonl").read_text())
        self.assertIsNone(line["detection"])

    def test_disk_guard_stops_recording(self):
        self.recorder.start(STATE)
        self.free[0] = 150
        self.assertIn("MB free", self.recorder.tick({}, STATE))
        self.assertFalse(self.recorder.active)
        self.assertTrue((self.session() / "motion.json").exists())

    def test_two_sessions_in_one_second_get_distinct_folders(self):
        self.recorder.start(STATE)
        self.recorder.stop()
        self.recorder.start(STATE)
        self.recorder.stop()
        self.assertEqual(sorted(p.name for p in self.root.iterdir()),
                         ["20261005-120000", "20261005-120000-2"])


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `.venv/bin/python -m unittest discover -s scripts/teleop -p 'test_teach.py'`
Expected: `ModuleNotFoundError: No module named 'teach'`.

- [ ] **Step 3: Implement `scripts/teleop/teach.py` (recorder part)**

```python
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
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `.venv/bin/python -m unittest discover -s scripts/teleop -p 'test_teach.py'`
Expected: `OK` (12 tests).

- [ ] **Step 5: Commit**

```bash
git add scripts/teleop/teach.py scripts/teleop/test_teach.py
git commit -m "Add pickup recorder with reference view, motion events and video dataset"
```

---

### Task 5: Replayer, TeachController and receiver wiring

**Files:**
- Modify: `scripts/teleop/teach.py` (append)
- Modify: `scripts/teleop/teleop_receiver.py` (`build_parser`, `run`, `main`)
- Test: `scripts/teleop/test_teach.py`, `scripts/teleop/test_teleop.py`

**Interfaces:**
- Consumes: `ArmController.set_pose/set_claw`, `Posture.set_level`, `.listener` (Task 2); `Recorder` (Task 4); `Camera` (Task 3); `Message` (Task 1); `robot_state` (Task 2).
- Produces: `teach.latest_session(root) -> Path | None`; `teach.load_motion(session) -> dict` (raises `ValueError`/`OSError`); `teach.Replayer(arm, posture, clock=time.monotonic, settle=1.0)` with `.active`, `.start(motion)`, `.tick() -> "done" | None`, `.abort()`; `teach.TeachController(camera, recorder, replayer, root, state, log)` with `.update(message, manual) -> bool` (True while a replay owns the arm/kneel), `.watchdog()`, `.close()`.
- Produces: `run(..., teach=None)` keyword; receiver flags `--color`, `--teach-dir`, `--record-fps`, `--min-free-mb`, `--replay-settle`.

- [ ] **Step 1: Write the failing tests**

Append to `test_teach.py`:

```python
class FakeArm:
    def __init__(self):
        self.calls = []

    def set_pose(self, x, z):
        self.calls.append(("arm", x, z))

    def set_claw(self, value):
        self.calls.append(("claw", value))


class FakePosture:
    def __init__(self, calls):
        self.calls = calls

    def set_level(self, level):
        self.calls.append(("kneel", level))


MOTION = {"version": 1, "color": "purple", "duration": 2.0,
          "start": {"arm": [80, 30], "claw": 0, "kneel": 0.0},
          "events": [{"t": 0.5, "arm": [90, 20]}, {"t": 1.0, "claw": 200},
                     {"t": 1.5, "kneel": 0.5}]}


class ReplayTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.clock = FakeClock()
        self.arm = FakeArm()
        self.posture = FakePosture(self.arm.calls)
        self.replayer = teach.Replayer(self.arm, self.posture, clock=self.clock, settle=1.0)

    def tearDown(self):
        self.tmp.cleanup()

    def save(self, name, motion):
        (self.root / name).mkdir()
        (self.root / name / "motion.json").write_text(json.dumps(motion))

    def test_latest_session_and_load(self):
        self.assertIsNone(teach.latest_session(self.root))
        self.save("20261005-110000", MOTION)
        self.save("20261005-120000", MOTION)
        (self.root / "20261005-130000").mkdir()  # unfinished: no motion.json
        self.assertEqual(teach.latest_session(self.root).name, "20261005-120000")
        self.assertEqual(teach.load_motion(teach.latest_session(self.root)), MOTION)
        self.assertIsNone(teach.latest_session(self.root / "missing"))

    def test_load_rejects_bad_motion(self):
        for bad in ({"version": 2}, dict(MOTION, events=[{"t": -1, "arm": [1, 2]}]),
                    dict(MOTION, events=[{"t": 1, "leg": 3}]),
                    dict(MOTION, events=[{"t": 1, "arm": [1, 2], "claw": 3}])):
            self.save("bad", bad)
            with self.assertRaises(ValueError):
                teach.load_motion(self.root / "bad")
            (self.root / "bad" / "motion.json").unlink()
            (self.root / "bad").rmdir()
        self.save("cut", MOTION)
        (self.root / "cut" / "motion.json").write_text('{"version": 1, "ev')  # power cut
        with self.assertRaises(ValueError):
            teach.load_motion(self.root / "cut")

    def test_replay_sends_start_then_events_on_time(self):
        self.replayer.start(MOTION)
        self.assertEqual(self.arm.calls, [("kneel", 0.0), ("arm", 80, 30), ("claw", 0)])
        self.clock.now += 1.4  # 0.4 s after the 1 s settle
        self.assertIsNone(self.replayer.tick())
        self.assertEqual(len(self.arm.calls), 3)
        self.clock.now += 0.2
        self.replayer.tick()
        self.assertEqual(self.arm.calls[-1], ("arm", 90, 20))
        self.clock.now += 1.0
        self.assertEqual(self.replayer.tick(), "done")
        self.assertEqual(self.arm.calls[-2:], [("claw", 200), ("kneel", 0.5)])
        self.assertFalse(self.replayer.active)

    def test_replay_skips_unknown_start_pose_and_abort_stops(self):
        self.replayer.start(dict(MOTION, start={"arm": None, "claw": None, "kneel": 0.0}))
        self.assertEqual(self.arm.calls, [("kneel", 0.0)])
        self.replayer.abort()
        self.clock.now += 10
        self.assertIsNone(self.replayer.tick())
        self.assertEqual(len(self.arm.calls), 1)


class FakeMessage:
    def __init__(self, record=0, replay=0, raw=None):
        self.record, self.replay, self.raw = record, replay, raw or {}


class FakeRecorder:
    def __init__(self, reason=None):
        self.active, self.reason, self.ticks, self.stopped = False, reason, [], 0

    def start(self, state):
        if self.reason:
            return self.reason
        self.active = True
        self.session = "/tmp/session"
        return None

    def tick(self, command, state):
        self.ticks.append(command)
        return None

    def stop(self):
        if not self.active:
            return None
        self.active = False
        self.stopped += 1
        return {"session": "/tmp/session", "frames": 3, "events": 2, "duration": 1.5}


class FakeCam:
    def __init__(self, error=None):
        self.error, self.started, self.closed = error, 0, False

    def start(self):
        if self.error:
            raise RuntimeError(self.error)
        self.started += 1

    def close(self):
        self.closed = True


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.clock = FakeClock()
        self.arm = FakeArm()
        self.replayer = teach.Replayer(self.arm, FakePosture(self.arm.calls), clock=self.clock,
                                       settle=0)
        self.recorder = FakeRecorder()
        self.camera = FakeCam()
        self.log = []
        self.teach = teach.TeachController(self.camera, self.recorder, self.replayer, self.root,
                                           lambda: STATE, self.log.append)

    def tearDown(self):
        self.tmp.cleanup()

    def save_motion(self):
        (self.root / "20261005-120000").mkdir()
        (self.root / "20261005-120000" / "motion.json").write_text(json.dumps(MOTION))

    def test_x_toggles_recording_and_ticks_while_recording(self):
        self.teach.update(FakeMessage(record=1), manual=False)
        self.assertTrue(self.recorder.active)
        self.assertEqual(self.camera.started, 1)
        self.teach.update(FakeMessage(record=1, raw={"x": 1}), manual=True)
        self.assertEqual(self.recorder.ticks[-1], {"x": 1})
        self.teach.update(FakeMessage(record=2), manual=False)
        self.assertFalse(self.recorder.active)
        self.assertTrue(any("3 frames" in line for line in self.log))

    def test_record_refusal_is_reported(self):
        self.recorder.reason = "no purple box in view"
        self.teach.update(FakeMessage(record=1), manual=False)
        self.assertFalse(self.recorder.active)
        self.assertTrue(any("no purple box" in line for line in self.log))

    def test_record_reports_camera_failure(self):
        self.camera.error = "cannot open camera 0"
        self.teach.update(FakeMessage(record=1), manual=False)
        self.assertFalse(self.recorder.active)
        self.assertTrue(any("cannot open camera" in line for line in self.log))

    def test_replay_without_recordings(self):
        self.assertFalse(self.teach.update(FakeMessage(replay=1), manual=False))
        self.assertEqual(self.arm.calls, [])
        self.assertTrue(any("No recording" in line for line in self.log))

    def test_replay_with_corrupt_motion_file(self):
        (self.root / "20261005-120000").mkdir()
        (self.root / "20261005-120000" / "motion.json").write_text("{not json")
        self.assertFalse(self.teach.update(FakeMessage(replay=1), manual=False))
        self.assertEqual(self.arm.calls, [])
        self.assertTrue(any("Cannot replay" in line for line in self.log))

    def test_replay_runs_and_manual_input_aborts(self):
        self.save_motion()
        self.assertTrue(self.teach.update(FakeMessage(replay=1), manual=False))
        self.assertFalse(self.teach.update(FakeMessage(replay=1), manual=True))
        self.assertFalse(self.replayer.active)
        self.assertTrue(any("aborted" in line for line in self.log))

    def test_replay_and_record_exclude_each_other(self):
        self.save_motion()
        self.teach.update(FakeMessage(record=1), manual=False)
        self.teach.update(FakeMessage(record=1, replay=1), manual=False)
        self.assertFalse(self.replayer.active)
        self.teach.update(FakeMessage(record=2, replay=1), manual=False)
        self.teach.update(FakeMessage(record=2, replay=2), manual=False)
        self.assertTrue(self.replayer.active)
        self.teach.update(FakeMessage(record=3, replay=2), manual=False)
        self.assertFalse(self.recorder.active)
        self.assertTrue(any("ignored" in line for line in self.log))

    def test_watchdog_aborts_replay_and_close_finalises(self):
        self.save_motion()
        self.teach.update(FakeMessage(replay=1), manual=False)
        self.teach.watchdog()
        self.assertFalse(self.replayer.active)
        self.teach.update(FakeMessage(replay=1, record=1), manual=False)
        self.teach.close()
        self.assertEqual(self.recorder.stopped, 1)
        self.assertTrue(self.camera.closed)
```

Append to `test_teleop.py` (end of `ReceiverTests`):

```python
    def test_run_records_motion_and_replays_through_teach(self):
        import teach

        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        clock = FakeClock()
        args = receiver.build_parser().parse_args([])
        dog = FakeDog()
        arm = receiver.ArmController(dog, args, clock=clock)
        posture = receiver.Posture(dog, args, clock=clock)
        driver = receiver.Driver(dog, args)
        frame = __import__("numpy").zeros((240, 320, 3), dtype="uint8")
        camera = SimpleNamespace(start=lambda: None, close=lambda: None,
                                 latest=lambda: (frame, clock.now))
        box = {"bbox": [140, 100, 40, 40], "center": [160, 120], "area_px": 1600}
        recorder = teach.Recorder(Path(tmp.name), camera, lambda f: {"purple": [box]}, "purple",
                                  clock=clock, free_mb=lambda p: 10 ** 6, encode=lambda f: b"j")
        arm.listener = posture.listener = recorder.on_send
        replayer = teach.Replayer(arm, posture, clock=clock, settle=0)
        controller = teach.TeachController(
            camera, recorder, replayer, Path(tmp.name),
            lambda: receiver.robot_state(driver, arm, posture), lambda text: None)
        idle = dict(enable=False, record=2, replay=1)  # no LB: not manual input
        lines = [line(record=1),
                 arm_line(az=1, record=1), arm_line(az=1, record=1),
                 line(record=2),
                 line(**idle), line(**idle), line(**idle), receiver.EOF]
        clock_lines = Paced(*lines)
        real_get = clock_lines.get

        def get(timeout=None):
            clock.now += 0.1
            return real_get(timeout)

        clock_lines.get = get
        receiver.run(driver, clock_lines, 0.5, lambda text: None, arm=arm, posture=posture,
                     teach=controller)
        session = next(Path(tmp.name).iterdir())
        motion = json.loads((session / "motion.json").read_text())
        self.assertEqual(motion["events"][0]["arm"], [80, 30])
        replayed = [call for call in dog.calls if call[0] == "arm"]
        self.assertEqual(replayed[-1], ("arm", 80, 36))  # the replay ends at the last pose

    def test_run_finalises_recording_on_exit(self):
        events = []
        teach_stub = SimpleNamespace(
            update=lambda message, manual: False, watchdog=lambda: None,
            close=lambda: events.append("close"))
        posture = receiver.Posture(self.dog, self.args, clock=FakeClock())
        posture.stand = lambda: events.append("stand")
        receiver.run(self.driver, Paced(line(), receiver.EOF), 0.5, self.log.append,
                     posture=posture, teach=teach_stub)
        self.assertEqual(events, ["close", "stand"])

    def test_teach_flags(self):
        args = receiver.build_parser().parse_args([])
        self.assertEqual((args.color, args.record_fps, args.min_free_mb, args.replay_settle),
                         ("purple", 10, 200, 1.0))
        self.assertTrue(args.teach_dir.endswith("xgo_teach"))
        with patch("sys.stderr", io.StringIO()):
            for argv in (["--color", "red"], ["--record-fps", "0"], ["--min-free-mb", "10"]):
                with self.assertRaises(SystemExit):
                    receiver.build_parser().parse_args(argv)
```

and at the top of `test_teleop.py` add `import tempfile` and `from pathlib import Path`. (`FakeClock`, `Paced`, `line`, `arm_line` already exist in that file.)

- [ ] **Step 2: Run tests to verify they fail**

Run: `.venv/bin/python -m unittest discover -s scripts/teleop -p 'test_*.py'`
Expected: `AttributeError: module 'teach' has no attribute 'Replayer'`, and `run() got an unexpected keyword argument 'teach'`.

- [ ] **Step 3: Implement**

Append to `teach.py`:

```python
def latest_session(root):
    """Newest session folder with a finished motion.json, or None."""
    root = Path(root)
    if not root.is_dir():
        return None
    finished = sorted(path for path in root.iterdir() if (path / "motion.json").is_file())
    return finished[-1] if finished else None


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
    if not isinstance(motion.get("start"), dict):
        raise ValueError("motion.json has no start state")
    return motion


class Replayer:
    """Play a recorded motion through the arm/kneel controllers, driven by tick()."""

    def __init__(self, arm, posture, clock=time.monotonic, settle=1.0):
        self.arm = arm
        self.posture = posture
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
        elapsed = self.clock() - self.begin
        while self.index < len(self.events) and self.events[self.index]["t"] <= elapsed:
            event = self.events[self.index]
            kind = next(kind for kind in MOTION_KINDS if kind in event)
            self.apply(kind, event[kind])
            self.index += 1
        if self.index >= len(self.events):
            self.active = False
            return "done"
        return None

    def abort(self):
        self.active = False


class TeachController:
    """Route X (record) and Y (replay) presses; start the camera on first use."""

    def __init__(self, camera, recorder, replayer, root, state, log=print):
        self.camera = camera
        self.recorder = recorder
        self.replayer = replayer
        self.root = Path(root)
        self.state = state
        self.log = log
        self.seen = {"record": 0, "replay": 0}

    def pressed(self, key, count):
        new = count > self.seen[key]
        self.seen[key] = count  # also resyncs if a counter ever goes backwards
        return new

    def update(self, message, manual):
        """Call once per message. Returns True while a replay owns the arm and kneel."""
        if self.pressed("record", message.record):
            self.toggle_recording()
        if self.pressed("replay", message.replay):
            self.start_replay()
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
        return False

    def toggle_recording(self):
        if self.replayer.active:
            self.log("Record button ignored during replay.")
            return
        if self.recorder.active:
            summary = self.recorder.stop()
            self.log("Recording saved: {session} ({frames} frames, {events} motion events, "
                     "{duration:.1f} s).".format(**summary))
            return
        try:
            self.camera.start()
        except RuntimeError as exc:
            self.log(f"Cannot record: {exc}.")
            return
        reason = self.recorder.start(self.state())
        if reason:
            self.log(f"Cannot record: {reason}.")
        else:
            self.log(f"Recording to {self.recorder.session} (press X again to stop).")

    def start_replay(self):
        if self.recorder.active:
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

    def close(self):
        self.replayer.abort()
        summary = self.recorder.stop()
        if summary:
            self.log("Recording saved: {session} ({frames} frames).".format(**summary))
        self.camera.close()
```

In `teleop_receiver.py` `build_parser()`, before `--timeout`, add:

```python
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
        "--replay-settle", type=lambda v: bounded_number(v, minimum=0, maximum=5,
                                                         label="replay-settle"),
        default=1.0, metavar="SECONDS",
        help="pause after moving to a recording's start pose (default: 1)")
```

Change `run`'s signature to add `teach=None`:

```python
def run(driver, lines, timeout, log=print, first=None, arm=None, barker=None, ramp=None,
        posture=None, teach=None):
```

In the watchdog (`except queue.Empty:`) branch, before `continue`, add:

```python
                    if teach is not None:
                        teach.watchdog()
```

After the `message = parse_message(line)` block (inside the loop, after the `except ValueError` branch's `continue`), replace the arm/barker/posture application section with:

```python
            if ramp is not None:
                walk = ramp.apply(walk)
            driver.apply(scale(walk, driver.args))
            manual = (message.raw.get("enable") is True or arm_command is not None
                      or kneel != 0)
            replaying = teach is not None and teach.update(message, manual)
            if arm is not None and not replaying:
                if arm_command is None:
                    arm.idle()
                else:
                    arm.update(arm_command)
            if barker is not None:
                barker.update(bark)
            if posture is not None and not replaying:
                posture.update(kneel)
```

and change the `finally` block to:

```python
    finally:
        driver.stop()
        if teach is not None:
            teach.close()
        if posture is not None:
            posture.stand()
```

In `main()`, replace the existing `run(...)` call (inside the `try:` after the bark-sound check, which built `ArmController`/`Posture` inline) with this block, keeping its indentation:

```python
        import camera as camera_module
        import teach as teach_module
        from detect_boxes import detect_boxes

        arm, posture = ArmController(dog, args), Posture(dog, args)
        cam = camera_module.Camera(0)
        recorder = teach_module.Recorder(Path(args.teach_dir).expanduser(), cam, detect_boxes,
                                         args.color, fps=args.record_fps,
                                         min_free_mb=args.min_free_mb)
        arm.listener = posture.listener = recorder.on_send
        controller = teach_module.TeachController(
            cam, recorder, teach_module.Replayer(arm, posture, settle=args.replay_settle),
            Path(args.teach_dir).expanduser(), lambda: robot_state(driver, arm, posture), log)
        run(driver, lines, args.timeout, log, first=first, arm=arm, barker=Barker(log=log),
            ramp=Ramp(args.ramp, args.ramp_start), posture=posture, teach=controller)
```

The `import teach` makes `sys.path` include `box_detection` before `from detect_boxes import detect_boxes`.

- [ ] **Step 4: Run tests to verify they pass**

Run: `.venv/bin/python -m unittest discover -s scripts/teleop -p 'test_*.py'`
Expected: `OK`.

- [ ] **Step 5: Commit**

```bash
git add scripts/teleop/teach.py scripts/teleop/test_teach.py scripts/teleop/teleop_receiver.py scripts/teleop/test_teleop.py
git commit -m "Wire X/Y recording and replay into the teleop receiver"
```

---

### Task 6: Live video stream (`--video`)

**Files:**
- Create: `scripts/teleop/video.py`
- Create: `scripts/teleop/test_video.py`
- Modify: `scripts/teleop/teleop_receiver.py` (`--video-port`, `main`)
- Modify: `scripts/teleop/gamepad_sender.py` (`--video`, `remote_command`, `main`)
- Test: `scripts/teleop/test_teleop.py`

**Requirements (from the user, after the spec):** `gamepad_sender.py --video` shows the robot camera live in the browser; it uses the same camera as recording; it binds only to the robot's Tailscale IPv4 on port 8090 by default; failure is reported and driving continues.

**Interfaces:**
- Consumes: `Camera.start()`, `Camera.latest()` (Task 3); `teach.encode_jpeg` (Task 4).
- Produces: `video.tailscale_ipv4(run=subprocess.run) -> str | None`; `video.VideoServer(camera, host, port, encode=None, fps=10)` with `.start()` (raises `OSError`/`RuntimeError`), `.url`, `.close()`; routes `/` (HTML page with the image) and `/stream` (`multipart/x-mixed-replace; boundary=frame`).
- Produces: receiver `--video-port` (0 = off, default 0); sender `--video` (passes `--video-port 8090`, opens `http://<robot host>:8090/`).

- [ ] **Step 1: Write the failing tests** (`scripts/teleop/test_video.py`)

```python
"""Offline checks for the live MJPEG stream; uses a local socket, no robot camera."""

import subprocess
import time
import unittest
import urllib.request
from types import SimpleNamespace

import numpy as np

import video


class VideoTests(unittest.TestCase):
    def test_tailscale_ipv4_parses_first_address(self):
        ok = lambda *a, **k: SimpleNamespace(returncode=0, stdout="100.74.30.90\n")  # noqa: E731
        self.assertEqual(video.tailscale_ipv4(run=ok), "100.74.30.90")
        bad = lambda *a, **k: SimpleNamespace(returncode=1, stdout="")  # noqa: E731
        self.assertIsNone(video.tailscale_ipv4(run=bad))

        def missing(*a, **k):
            raise FileNotFoundError("tailscale")

        self.assertIsNone(video.tailscale_ipv4(run=missing))
        timeout = lambda *a, **k: (_ for _ in ()).throw(subprocess.TimeoutExpired("t", 3))  # noqa: E731
        self.assertIsNone(video.tailscale_ipv4(run=timeout))

    def test_stream_serves_jpeg_parts_and_index(self):
        frame = np.zeros((4, 4, 3), dtype=np.uint8)
        camera = SimpleNamespace(start=lambda: None, latest=lambda: (frame, time.monotonic()))
        server = video.VideoServer(camera, "127.0.0.1", 0, encode=lambda f: b"JPEGDATA", fps=50)
        server.start()
        try:
            page = urllib.request.urlopen(server.url, timeout=3).read().decode()
            self.assertIn('src="/stream"', page)
            with urllib.request.urlopen(server.url + "stream", timeout=3) as response:
                self.assertIn("multipart/x-mixed-replace", response.headers["Content-Type"])
                chunk = response.read(200)
            self.assertIn(b"--frame", chunk)
            self.assertIn(b"JPEGDATA", chunk)
        finally:
            server.close()

    def test_video_start_failure_is_reported(self):
        def broken():
            raise RuntimeError("cannot open camera 0")

        server = video.VideoServer(SimpleNamespace(start=broken), "127.0.0.1", 0)
        with self.assertRaisesRegex(RuntimeError, "cannot open camera"):
            server.start()


if __name__ == "__main__":
    unittest.main()
```

Append to `SenderTests` in `test_teleop.py`:

```python
    def test_video_flag_passes_port_and_url(self):
        args = sender.build_parser().parse_args(["--video"])
        self.assertIn("--video-port 8090", sender.remote_command(args))
        self.assertEqual(sender.video_url(args), "http://100.74.30.90:8090/")
        self.assertNotIn("--video-port", sender.remote_command(sender.build_parser().parse_args([])))
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `.venv/bin/python -m unittest discover -s scripts/teleop -p 'test_*.py'`
Expected: `ModuleNotFoundError: No module named 'video'` and `AttributeError: ... 'video_url'`.

- [ ] **Step 3: Implement**

`scripts/teleop/video.py`:

```python
"""Live MJPEG view of the robot camera, served only on the robot's Tailscale address."""

import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

PAGE = (b"<!doctype html><title>XGO camera</title>"
        b"<body style='margin:0;background:#111'>"
        b"<img src=\"/stream\" style='width:100%;height:100vh;object-fit:contain'></body>")


def tailscale_ipv4(run=subprocess.run):
    """The robot's Tailscale IPv4 address, or None if Tailscale is unavailable."""
    try:
        result = run(["tailscale", "ip", "-4"], capture_output=True, text=True, timeout=3)
    except (OSError, subprocess.TimeoutExpired):
        return None
    lines = result.stdout.split() if result.returncode == 0 else []
    return lines[0] if lines else None


class VideoServer:
    def __init__(self, camera, host, port, encode=None, fps=10):
        self.camera = camera
        self.host = host
        self.port = port
        self.encode = encode
        self.period = 1.0 / fps
        self.server = None

    def start(self):
        self.camera.start()
        if self.encode is None:
            from teach import encode_jpeg
            self.encode = encode_jpeg
        outer = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):  # keep the driving terminal quiet
                pass

            def do_GET(self):
                if self.path == "/":
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html")
                    self.end_headers()
                    self.wfile.write(PAGE)
                elif self.path == "/stream":
                    self.send_response(200)
                    self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                    self.end_headers()
                    outer.stream_to(self.wfile)
                else:
                    self.send_error(404)

        self.server = ThreadingHTTPServer((self.host, self.port), Handler)
        self.server.daemon_threads = True
        self.port = self.server.server_address[1]
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    @property
    def url(self):
        return f"http://{self.host}:{self.port}/"

    def stream_to(self, out):
        last = None
        try:
            while self.server is not None:
                frame, stamp = self.camera.latest()
                if frame is not None and stamp != last:
                    data = self.encode(frame)
                    out.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                              + str(len(data)).encode() + b"\r\n\r\n" + data + b"\r\n")
                    out.flush()
                    last = stamp
                time.sleep(self.period)
        except (BrokenPipeError, ConnectionResetError):
            pass  # viewer closed the page

    def close(self):
        if self.server is not None:
            server, self.server = self.server, None
            server.shutdown()
            server.server_close()
```

In `teleop_receiver.py` `build_parser()` add:

```python
    parser.add_argument("--video-port", type=int, default=0, metavar="PORT",
                        help="serve the live camera on the Tailscale address (0 = off)")
```

In `main()`, right after the `controller = ...` line from Task 5 and before `run(...)`:

```python
        viewer = None
        if args.video_port:
            import video

            host = video.tailscale_ipv4()
            if host is None:
                log("Live video unavailable: no Tailscale IPv4 address found.")
            else:
                try:
                    viewer = video.VideoServer(cam, host, args.video_port)
                    viewer.start()
                    log(f"Live video: {viewer.url}")
                except (OSError, RuntimeError) as exc:
                    log(f"Live video unavailable: {exc}.")
                    viewer = None
```

and wrap the `run(...)` call so the server closes:

```python
        try:
            run(driver, lines, args.timeout, log, first=first, arm=arm, barker=Barker(log=log),
                ramp=Ramp(args.ramp, args.ramp_start), posture=posture, teach=controller)
        finally:
            if viewer is not None:
                viewer.close()
```

In `gamepad_sender.py`: add `import webbrowser` to the imports; in `build_parser()` add

```python
    parser.add_argument("--video", action="store_true",
                        help="show the robot camera live in the browser (port 8090)")
```

add after `remote_command`:

```python
VIDEO_PORT = 8090


def video_url(args):
    return f"http://{args.host.split('@')[-1]}:{VIDEO_PORT}/"
```

in `remote_command`, before the `return`, add:

```python
    if args.video:
        receiver += ["--video-port", str(VIDEO_PORT)]
```

and in `main()`, right after `ssh = start_ssh(args)`:

```python
    if args.video:
        url = video_url(args)
        print(f"Live video: {url} (opens once the robot is ready)", file=sys.stderr)
        threading.Timer(4.0, webbrowser.open, args=(url,)).start()
```

with `import threading` added to the imports. (The 4 s delay lets the receiver stand the dog and start the server; if the page opens too early, reload it.)

- [ ] **Step 4: Run tests to verify they pass**

Run: `.venv/bin/python -m unittest discover -s scripts/teleop -p 'test_*.py'`
Expected: `OK`.

- [ ] **Step 5: Commit**

```bash
git add scripts/teleop/video.py scripts/teleop/test_video.py scripts/teleop/teleop_receiver.py scripts/teleop/gamepad_sender.py scripts/teleop/test_teleop.py
git commit -m "Add --video live camera stream on the robot's Tailscale address"
```

---

### Task 7: Docs, full verification and deployment

**Files:**
- Modify: `scripts/teleop/README.md`

- [ ] **Step 1: Update the README** — add to the controls table:

```markdown
| **X** | Start / stop recording a pickup (needs one fully visible box of `--color`) |
| **Y** | Replay the latest recorded pickup; any stick, LB/RB or kneel press aborts |
```

and a new section after the controls:

```markdown
## Recording and replaying a pickup

Put the box directly in front of the robot, press **X**, pick it up with the controller,
and press **X** again. Each recording is a folder in `~/xgo_teach/<date-time>/` on the robot:

| File | Contents |
| --- | --- |
| `reference.jpg`, `reference.json` | The camera view and box position/size when recording started |
| `motion.json` | Arm, claw and kneel targets with timings — what **Y** replays (walking is never replayed) |
| `motion_events.jsonl` | The same events, written as they happen (survives a power cut) |
| `frames/*.jpg`, `dataset.jsonl` | Video frames (10 fps) with the gamepad command, robot state and box detection at each frame — training data |

Recording refuses to start unless exactly one box of `--color` (default purple) is fully in view,
and stops by itself when free disk drops below `--min-free-mb` (200). At 640×480 a minute of
recording takes roughly 15–20 MB.

## Live video

`gamepad_sender.py --video` serves the robot camera at `http://100.74.30.90:8090/` (bound to the
robot's Tailscale address only) and opens it in your browser. It shares the camera with
recording. The old camera stream (`test_xgo_connectivity`) must not be running.
```

- [ ] **Step 2: Full local verification**

Run:

```bash
for d in teleop movement_tests box_detection sensor_monitor; do .venv/bin/python -m unittest discover -s scripts/$d -p 'test_*.py' 2>&1 | tail -1; done
.venv/bin/python -m pyflakes scripts/teleop
for f in scripts/teleop/teleop_receiver.py scripts/teleop/teach.py scripts/teleop/camera.py scripts/teleop/video.py; do python3 -c "import ast,sys; ast.parse(open('$f').read(), feature_version=(3,9))"; done
git diff --check
```

Expected: four `OK`, no pyflakes output, no grammar errors, no whitespace errors.

- [ ] **Step 3: Commit, push (authorised by the user for this project), deploy and run the tests on the robot**

```bash
git add scripts/teleop/README.md
git commit -m "Document pickup recording, replay and live video"
git push origin main
ssh -o BatchMode=yes -o ConnectTimeout=8 pi@100.74.30.90 'cd ~/project/xgo_robot_kit_playing_fetch && git status --short --branch && git pull --ff-only origin main && sync && PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s scripts/teleop -p "test_*.py" 2>&1 | tail -1'
```

Expected: clean status, fast-forward, `sync`, `OK` on Python 3.9.2. (`sync` guards against the power-loss corruption seen on 2026-10-05.)

- [ ] **Step 4: Hardware milestone A (supervised, user present, robot on the floor)**

Before: verify the UART and camera are free (`sudo fuser -v /dev/ttyAMA0 /dev/video0`), stop the boot menu / camera stream if needed (targeted PIDs, SIGINT for the stream), read the battery with the logger, check `df -h /`.

User runs: `.venv/bin/python scripts/teleop/gamepad_sender.py --video --ramp 2 --max-x 20 --max-y 14 --max-turn 25`

1. Live video opens; box visible.
2. Press X → `Recording to …`. Pick the box up with RB/LT. Press X → `Recording saved: … frames …`.
3. Put the box back in the same spot; return the arm with RB; press Y → replay; observe whether the box is lifted.
4. After the session: list the session folder size and frame count over SSH; confirm `motion.json` loads.

Success: the replay lifts the box at least once. Record the result (and failures) for the user.
