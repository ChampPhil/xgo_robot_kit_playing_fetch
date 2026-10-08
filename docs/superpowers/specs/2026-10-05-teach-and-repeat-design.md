# Teach-and-repeat box pickup: design

Date: 2026-10-05. Status: draft for review.

## Goal

The user places a box directly in front of the robot and picks it up with the gamepad. That
demonstration is recorded and becomes a **fixed pickup motion**. The autonomous task is then
reduced to "get the robot into the same position relative to the box", judged by the camera:
when the box looks the way it did at the start of the demonstration, replaying the motion
should pick it up. Every recording also stores **video plus commands and robot state**, so the
data can later train a learned alignment model ("drive until the box is at this place in the
image").

Success: (A) a recorded pickup replays successfully with the box put back in the same spot;
(B) a live readout shows how far the current view is from the reference; (C) holding a button
moves the robot in small steps until the view matches, after which replay picks the box up.

## Decisions (agreed with the user)

| Topic | Decision |
| --- | --- |
| Replayed motion | Walk, arm, claw and kneel. *Changed 2026-10-08:* the box is only fully visible from a stand-off distance, so a recording starts there and includes walking forward; replayed walking is blind and aborts on any input. |
| Video/dataset | Recorded with everything commanded (walking included) plus robot state. |
| Buttons | **X** toggles recording, **Y** replays the latest pickup, **hold A** aligns. B still barks. |
| Storage | On the robot only, with a free-disk guard. |
| Alignment start | Only moves while A is held; release stops. |
| Alignment ⇒ replay | Manual (press Y after "aligned") until alignment is proven reliable. |

## Architecture

The robot-side `teleop_receiver.py` stays the **single owner** of the UART and becomes the owner of
the camera too: recording, replay and alignment all run inside its existing loop, which already
receives gamepad messages at 20 Hz and holds the walk/arm/kneel state. New logic goes into
focused modules next to it so the receiver only wires them together:

| Unit | Responsibility | Depends on |
| --- | --- | --- |
| `camera.py` | Background thread grabbing frames from `/dev/video0` (OpenCV); `latest()` returns `(frame, timestamp)` so callers can reject stale frames. Opened lazily on first X/A press. | OpenCV |
| `teach.py` | `Recorder` (session directory, reference capture, motion events, dataset log, disk guard), `Replayer` (time-indexed playback through the existing arm/kneel controllers), motion-file load/save. | camera, `detect_boxes` |
| `align.py` | Pure functions: box features from detections, error vs. reference, next-step decision; plus the small move/stop/settle/observe state machine. | `detect_boxes` output only |
| `teleop_receiver.py` | Routes the new message fields to the units above; exposes setters on `ArmController`/`Posture` so replay continues smoothly into manual control. | all of the above |
| `gamepad_sender.py` | Adds `record` (X press counter), `replay` (Y press counter), `align` (A held) to each message; shows recorder/aligner status. | — |

Press counters (like `bark`) mean a skipped stale message cannot lose a press.

## Recording (X)

Pressing X starts a session; pressing X again stops it. On start:

1. The camera grabs a fresh frame and `detect_boxes` must find **exactly one** box of the target
   colour (`--color`, default purple) that does not touch the image border. Otherwise recording
   refuses to start and says why (missing / ambiguous / clipped). This frame is the **reference**.
2. The session directory `~/xgo_teach/<YYYYmmdd-HHMMSS>/` is created (outside the git checkout).
3. The current arm pose, claw and kneel level are stored as the **start state** (pose may be
   unknown if the arm has not moved yet this session; replay then begins at the first event).

While recording:

- **Motion events** (`motion.json`): every arm target, claw value and kneel level actually sent
  to the robot, with time since start. Walking is not included.
- **Dataset** (`frames/000123.jpg` + one line per frame in `dataset.jsonl`), at `--record-fps`
  (default 10) and JPEG quality 80 at the camera's native 640×480:
  `{"i", "t", "frame", "command": <the raw gamepad message>, "state": {arm, claw, kneel, walk
  steps sent}, "detection": <target-colour box or null>}`. Each line is flushed as written, so a
  power cut loses at most the last frame rather than corrupting the session.
- **Disk guard:** before each frame, if free space on `/` is below `--min-free-mb` (default 200),
  recording stops with a message. Starting is refused below that threshold too.

Motion events are also appended to `motion_events.jsonl` as they happen, so a power cut leaves a
recoverable log. On stop, `motion.json` is written atomically (temp file + rename + fsync) and the session is
logged
with its frame count and duration. `reference.jpg` and `reference.json` (normalised box features,
frame size, colour, kneel level and arm pose at the time) are written at start.

Storage estimate: ~25–35 KB per frame ⇒ ~15–20 MB per recorded minute at 10 fps. With ~300 MB free
today that is about 15 minutes in total; the existing camera-stream AVIs (~789 MB) are the main
reclaimable space and are only removed if the user approves.

## Replay (Y)

Y replays the **latest** session's `motion.json`:

1. Send the start state (kneel level, arm pose, claw) and wait `--replay-settle` (default 1 s).
2. Send each event at its recorded time offset, driven from the receiver loop (no blocking
   sleeps), through the `ArmController`/`Posture` setters so manual control continues from the
   final pose.
3. **Abort** immediately on any stick outside the deadzone, LB/RB/LT, START, EOF, the watchdog
   firing, or a malformed message. Walking is stopped throughout.

Y is ignored while recording or aligning.

## Alignment (hold A)

Features from the target detection, normalised by frame size: centre `(u, v)` and box height `h`
(height is less affected by the box's yaw than width). Reference values come from
`reference.json`. Errors: `eu = u − u_ref`, `ev = v − v_ref`, `eh = h / h_ref − 1`.

While A is held:

1. **Prepare:** move to the reference kneel level and arm pose (the camera view depends on body
   pitch and the arm may occlude it), then settle.
2. **Loop** as a state machine, never commanding longer than one step at a time:
   observe a **fresh** frame (timestamp after the last settle) → decide → move for `--align-step`
   (default 0.3 s) → stop → settle (`--align-settle`, default 0.5 s) → observe.
3. **Decide**, in priority order:
   - `|eu| > --tol-u` (default 0.04): turn toward the box;
   - `|eh| > --tol-h` (default 0.08): walk forward if the box looks too small, back if too large;
   - otherwise, if `|ev| > --tol-v` (default 0.05): **abort** with "view mismatch". On a flat floor
     size and vertical position both track distance, so a correct size with the wrong height in
     the image means the camera pose differs from the reference (kneel, arm, box or floor), which
     walking cannot fix and which would otherwise make forward/back corrections oscillate;
   - otherwise count one "in tolerance" observation.
4. **Aligned** after 3 consecutive in-tolerance observations: stop, report "ALIGNED — press Y".
5. **Abort** (stop and report why) if: A released; target missing/ambiguous/touching the border on
   3 consecutive observations; the camera frame is stale (> 1 s old); total error grows on 3
   consecutive steps; more than `--align-max-steps` (default 40) steps or 30 s in one hold.

Speeds for alignment steps are small fixed values (`--align-walk`, default 8; `--align-turn`,
default 20). Directions (which way to turn for `eu > 0`, which way to walk for `eh < 0`) are
encoded as constants that milestone B verifies on the real robot before any motion is enabled.

**Motion gate:** without `--align-motion`, holding A is **display-only**: it runs the full
observe/decide loop and prints the errors and the step it *would* take, without moving the legs.

## Status display

The receiver logs short status lines to stderr, which appear in the Mac terminal:
`REC 00:12 120 frames`, `REPLAY 3.1/8.4 s`, `ALIGN eu=+0.06 eh=-0.12 ev=+0.01 → turn right`,
`ALIGNED — press Y`, and every refusal/abort reason.

## Error handling

- Camera fails to open or is busy (e.g. the camera stream is running): X and A report it and do
  nothing; driving keeps working.
- No recording exists: Y reports it.
- Corrupt or missing `motion.json`: Y reports it; nothing moves.
- Recording in progress when the session ends (EOF, START, signal): the recorder finalises its
  files in the receiver's `finally` block before the robot stands up.

## Testing

Offline (no hardware), following the existing fake-dog style:

- Feature extraction and error computation on synthetic detections, including border/ambiguous
  rejection.
- Decision function: each priority branch, tolerance edges, sign constants.
- Alignment state machine with a fake clock and scripted observations: aligned after 3, aborts
  on lost target, stale frame, growing error, step/time limits, release; display-only mode sends
  no walk commands.
- Recorder: refuses without a unique target; records arm/claw/kneel events but not walking;
  dataset lines and frame files; disk guard stops recording; atomic motion file.
- Replayer: timing, start state, abort on input/watchdog, ignored while recording/aligning.
- Sender: X/Y press counters and A held in messages.
- Python 3.9 grammar check, tests run on the robot's Python 3.9.

Hardware validation, one milestone at a time, supervised, on the floor:

- **A — record and replay:** record one pickup; put the box back where it was; press Y. Success =
  the box is lifted.
- **B — display-only alignment:** move the box/robot by hand and confirm the error signs and the
  suggested step directions are right.
- **C — motorised alignment:** with `--align-motion`, start slightly off (rotated, too far, too
  close), hold A, then press Y.

## Out of scope (for now)

Grasp verification from the claw readback; automatic replay after alignment; strafing for
lateral alignment; boxes at arbitrary orientations; training a learned model (the dataset format
is designed to convert to LeRobot later); stacking.
