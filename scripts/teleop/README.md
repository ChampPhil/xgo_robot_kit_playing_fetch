# Gamepad teleop over Tailscale SSH

Drive the XGO Lite with a gamepad plugged into your computer. The sender reads the
sticks and pipes one JSON command per tick into an SSH session; the receiver on
the robot turns them into `move_x` / `move_y` / `turn` calls.

```
[gamepad] -> gamepad_sender.py (Mac) --stdin of ssh pi@100.74.30.90--> teleop_receiver.py (robot) -> /dev/ttyAMA0
```

| Control | Robot |
| --- | --- |
| **LB (hold)** | Walk mode, dead-man switch: the robot only walks while held |
| ↳ Left stick up / down | Walk forward / backward |
| ↳ Left stick left / right | Strafe left / right |
| ↳ Right stick left / right | Turn left / right in place |
| **RB (hold)** | Arm mode: walking stops, the sticks move the arm (overrides LB) |
| ↳ Left stick up / down | Raise / lower the arm |
| ↳ Left stick right / left | Reach out / pull in |
| ↳ Right stick right / left | Close / open the claw |
| **LT (hold) + D-pad down / up** | Kneel the front down toward the floor / stand back up |
| **B** | Bark (robot speaker) |
| **X** | Start / stop recording a pickup (needs one fully visible box of `--color`) |
| **Y** | Replay the latest recorded pickup; any stick, LB/RB or kneel press aborts |
| **A (hold)** | Align with the latest recording's reference view (moves only with `--align-motion`) |
| **BACK** | Reset the claw: open it, then move the arm back to its start pose (80, 30) |
| **RT** | Full reset: stop, run the robot's own reset (`dog.reset()`), open the claw fully, then kneel fully |
| START, or Ctrl+C | Stop and quit |

Arm mode is **speed** control: hold a stick and the arm keeps moving, centre it
and the arm holds where it is. The arm's real pose cannot be read back (the
arm/claw registers return no usable value), so the **first** arm stick move sends
the arm to `--arm-home-x/--arm-home-z` (default 80, 30 mm) before moving from
there, and the claw starts from `--claw-start` (128). The claw is 0 (open)…255 (closed).

The arm only reaches a **ring** 80–140 mm from its base, in front of the body; the
firmware silently ignores targets outside it, which used to make the arm look
frozen. Measured on the robot: (140, 30) and (80, 115) are followed; (150, 30),
(80, 130) and (55, 30) are ignored; (80, −90) is followed, close to the API floor
of z −95. The controller now moves the arm only as far as that ring allows and
stops at its edge, so reversing a stick always moves the arm straight away. The manufacturer says not to carry more than 20 g.

Kneeling pitches the front of the body down (up to +10°, the Lite's limit) and
lowers the body (to 70 mm), the posture xgolib's own floor-pickup demo uses. Hold
the D-pad to keep going; release it and the robot holds that pose. It is
independent of LB/RB, so the arm and claw work normally while knelt, and lowering
the body brings the whole arm reach ring closer to the floor. Quitting returns the
robot to standing. Body height while standing is assumed to be 85 mm (xgolib's
neutral value); if the first kneel press makes the body jump, adjust
`--stand-height`.

## Keyboard control (`--keyboard`)

Add `--keyboard` to drive from the laptop keyboard instead of the gamepad (the gamepad still
works without the flag). After you confirm in the terminal, a small **"XGO keyboard control"**
window opens; click it, then **hold** keys — releasing a key stops that motion, and clicking
another window releases everything (the robot stops). Keys give fixed speeds (half, or full with
**Shift**), which makes recordings cleaner and more repeatable than analog sticks.

| Hold | Action | Press | Action |
| --- | --- | --- | --- |
| **W / S** | Walk forward / back | **X** | Start / stop recording |
| **A / D** | Side-step left / right | **Y** | Replay the latest recording |
| **Q / E** | Turn left / right | **C** | Reset the claw (open, arm to start) |
| **I / K** | Arm up / down | **B** | Bark |
| **J / L** | Arm in / out | **Esc**, Ctrl+C | Quit (the robot stops and stands) |
| **U / O** | Claw open / close | **Z** | Full reset: claw fully open, kneel fully |
| **G / T** | Kneel down / stand up | **F** | **Fetch**: reset, align by itself, pick up |
| **Shift** | Full speed (with the keys above) | | |
| **Space** | Align (same as holding A) | | |

Arm keys take over from walk keys (walking stops while the arm moves), like RB on the gamepad.

## Fetch (keyboard F, needs `--align-motion`)

One press does the whole job: **full reset** (claw open, fully kneeled, arm back to its default,
so the camera sees exactly what it saw for the reference photo), 1.5 s to settle, **align by
itself** with the latest recording's start view (calibrating with one 15° turn the first time),
then **replay the recorded pickup** once aligned. Nothing needs to be held. **F again, any
movement key, Esc, Ctrl+C in the terminal, or a lost connection cancels** it and stops the legs.
Clicking away from the keyboard window does *not* cancel a fetch (it only releases held keys);
click back into the window to cancel with F or Esc. It is ignored while recording or replaying, and needs `--align-motion`.

## Start position and full reset

After connecting, the robot now goes straight to **fully kneeled** (front down 10°, body at
70 mm), so a box in front stays in the camera's view; `--start-standing` keeps it standing.
**RT** (gamepad) or **Z** (keyboard) does a **full reset** at any time: legs stop, the robot's
built-in reset returns posture, arm and gait parameters to neutral (about 0.75 s), the program
forgets the arm/claw/kneel state it had sent, the claw opens fully, heading hold re-measures
the gyro drift (the reset may recalibrate the gyro), and the robot kneels fully again. It is ignored while
recording, replaying or aligning. Stand still about 3 s afterwards before walking.

## Walking straight (heading hold)

The gait drifts, so "walk straight" alone curves. While the legs walk without a turn command,
the receiver locks the heading and reads the IMU's fused yaw (register `0x64`) every 0.2 s,
adding a small corrective turn (`--heading-gain` 2 deg/s per degree, at most `--heading-max`
25 deg/s, ignoring drift under 2 deg). Any turn you command wins; the heading re-locks when you
stop turning or stop walking. It also corrects replayed walks (recordings store what you asked
for, not the corrections, so nothing is applied twice).

- **Gyro drift:** the IMU's yaw drifts steadily even standing still (measured **6.9 deg/s** on
  this robot, never wrapped to ±180). Whenever the legs have been still for 0.5 s, heading hold
  fits the drift rate over the last few seconds and subtracts it, so only real rotation is
  corrected. After starting, **stand still for about 2.5 s** before walking; until then it says
  "stand still for 2 s" and doesn't correct.
- **First use:** it does nothing until it has seen one real turn (Q/E, the turn stick, or the
  alignment's calibration turn), from which it learns which way the gyro counts. That is
  saved to `~/xgo_teach/heading.json`; `--heading-sign 1|-1` overrides it.
- **Safety:** yaw is read with the logger's validated protocol (50 ms timeout); a failed read
  means *no* correction (xgolib's `read_yaw()` would wait 1 s and return 0, which looks like a
  sudden turn). If drift still reaches 30° while correcting, heading hold switches itself off for
  the session and says so. `--no-heading-hold` turns it off.

## Recording and replaying a pickup

The box is only fully visible from a little way back, out of the claw's reach, so a recording
starts at that **stand-off** position and includes the approach:

1. Centre the box in the video from where it is fully visible (later, holding **A** does this).
2. Press **X** (the reference photo is taken here, once the legs have settled).
3. Walk forward with LB, stop, then pick the box up with RB (and LT + D-pad to kneel).
4. Press **X** again.

**Y** replays all of it: the recorded walking (blind, at the recorded speeds and times), then
the arm/claw/kneel moves. Any stick, LB/RB or kneel input aborts the replay and stops the legs,
as does losing the connection; the legs are always stopped at the end. Each recording is a
folder in `~/xgo_teach/<date-time>/` on the robot:

| File | Contents |
| --- | --- |
| `reference.jpg`, `reference.json` | The camera view and box position/size when recording started |
| `motion.json` | Walk, arm, claw and kneel commands with timings — what **Y** replays |
| `motion_events.jsonl` | The same events, written as they happen (survives a power cut) |
| `frames/*.jpg`, `dataset.jsonl` | Video frames (10 fps) with the gamepad command, robot state and box detection at each frame — training data |

The camera shakes heavily while walking, so pressing **X** waits until the legs have been still
for `--settle` seconds (0.7) and then takes the reference from a frame captured after that; it
also rejects a blurry reference (`--min-sharpness`, Laplacian variance, default 50 — a still
frame on the robot measured about 410) and keeps retrying for 2 s. Press **X** again while it
waits to cancel. Every dataset line records `"moving"` (legs walking) and `"sharpness"`, so
shaky frames can be filtered out for training.

Recording refuses to start unless exactly one box of `--color` (default purple) is fully in view,
and stops by itself when free disk drops below `--min-free-mb` (200). At 640×480 a minute of
recording takes roughly 15–20 MB.

## Aligning with the box (hold A, or F = fetch)

Alignment moves the robot back to where the latest recording's first frame was taken, so the
recorded pickup can be replayed. Design notes: `docs/superpowers/specs/2026-10-07-self-calibrating-alignment-design.md`
(revision 2026-10-15).

Measured on the robot (pulse probe, 2026-10-15): a leg command does **nothing for the first
~0.5 s** (gait start-up), then turns at roughly the commanded speed in deg/s (speed 40 ≈ 32
deg/s); the camera spans **~94°**; after the program's startup reset the **gyro tracks turns
with no drift**. Alignment therefore:

- **Turns in one move, on the gyro:** the angle comes from where the box sits in the image
  (degrees per image width), and the turn stops when the gyro reports it. A failed gyro read
  stops the turn.
- **Walks in timed segments** that include the 0.5 s start-up; since apparent size ∝ 1/distance,
  1/size falls linearly while walking, so the time to the target size is planned from the
  measured rate. Short final corrections walk at half speed.
- **Calibrates once** with a single 15° turn toward the box (degrees per image width and which
  way the gyro counts), saved to `~/xgo_teach/calibration.json`; `--recalibrate` redoes it.
- **Sees far boxes:** at ~60 cm the purple box is only ~11 px and dark (S≈62, V≈40, hue up to
  ~175), so alignment uses a wider purple range (H115–178, S≥40, V≥25), a 40 px minimum, and
  takes a box at least twice the area of any other blob (background specks). The reference
  photo is re-measured with the same detector so sizes compare like for like.
- **Display-only by default:** without `--align-motion`, holding A only logs the error and the
  move it would make.
- Aligned = box centre within `--tol-u` (0.04 of the width) and size within `--tol-h` (8%) on two
  looks in a row. It stops and says why on release, any manual input, no commands for 0.5 s, the
  box lost 3 times, a stalled camera, the error growing 3 moves in a row, `--align-max-moves`
  (25) or `--align-max-seconds` (45), or "view mismatch" (right size, wrong height in the image).

Tuning: `--align-turn` (40), `--align-walk` (12). Floor only, supervised, within reach.

**Replays are shortened:** pauses longer than `--replay-max-gap` (0.5 s) between recorded moves,
including the wait before the first move, are cut to 0.5 s (`0` keeps the original timing).

## Live video

The stream shows a **red "target" box** where the box was in the latest recording's first frame
(its reference), drawn only on the viewer's copy — recordings are untouched — and refreshed every
2 s so a new recording shows up. With `--keyboard --video` the video appears **inside the keyboard
window, beside the key help** (no browser tab, so the robot only encodes one stream).

### How the stream is reached

`gamepad_sender.py --video` shows the robot camera at `http://localhost:8090/` and opens it in
your browser. The tailnet allows only SSH from this computer to the robot, so the receiver
serves video on the robot's loopback (`127.0.0.1:8090`) and the sender forwards it through its
SSH session (`ssh -L 8090:127.0.0.1:8090`); nobody else on the network can view it. It shares
the camera with recording. The old camera stream (`test_xgo_connectivity`) must not be running.
If port 8090 is busy on this computer, SSH prints a warning and driving still works.

## Session log

Everything the robot program prints is also appended, timestamped, to
`~/xgo_teach/session.log` on the robot (rotated to `session.log.1` above 2 MB), so a run —
e.g. why a fetch stopped — can be read afterwards over SSH:
`ssh pi@100.74.30.90 tail -50 xgo_teach/session.log`.

## Code map

| File | Runs on | Job |
| --- | --- | --- |
| `gamepad_sender.py` | Mac | Reads the gamepad (or `--keyboard`), sends one JSON command per tick over SSH, opens the video tunnel |
| `keyboard_input.py` | Mac | The keyboard window: key → controller state, the side-by-side video panel |
| `teleop_receiver.py` | Robot | Flags, startup (reset, battery check, kneel), the 20 Hz loop that routes each command, shutdown |
| `protocol.py` | Robot | The command format (`Message`, `parse_message`) and value checks |
| `body.py` | Robot | Legs (`Driver`, `Ramp`, `LegStepper`), arm and claw with the measured reach ring (`ArmController`), kneel (`Posture`) |
| `actions.py` | Robot | Buttons: bark, reset claw (BACK/C), full reset (RT/Z), fetch (F) |
| `heading.py` | Robot | Gyro reader and heading hold |
| `teach.py` | Robot | Recording (X), replay (Y), reference photo |
| `align.py` | Robot | Alignment (A / fetch): observer, gyro turns, walk segments, its detector settings |
| `camera.py`, `video.py` | Robot | Background camera reader; live MJPEG stream with the red target box |

## Setup (computer side only)

```bash
.venv/bin/python -m pip install -r scripts/teleop/requirements-mac.txt
```

pygame-ce is **not** needed on the robot; the receiver uses only the robot's
existing `python3`, `xgolib` and `pyserial`.

**Logitech F310:** set the switch on the controller to **D**. In **X** mode it
presents as an Xbox 360 pad (USB class 255), which macOS cannot read, and the
sender reports "no gamepad found". Other SDL-recognised controllers (Xbox, PS4/5,
8BitDo) should also work; the sender uses SDL's standard controller layout.

## Run

1. Put the robot **on the floor** with clear space. There is no edge or obstacle
   detection; never drive on the tabletop.
2. Make sure the robot checkout has this folder (`git pull --ff-only` there).
3. Free the UART: the boot menu (`main.py`) and the camera stream with telemetry
   both hold `/dev/ttyAMA0`. The receiver refuses to start while anything owns it.
   See AGENTS.md section 5 for how to stop them safely.
4. From the repo root on your computer:

```bash
.venv/bin/python scripts/teleop/gamepad_sender.py
# gentler first run:
.venv/bin/python scripts/teleop/gamepad_sender.py --max-x 8 --max-y 6 --max-turn 25
```

After you confirm, the receiver creates `XGO`, which **resets and stands the
dog**, checks the battery (refuses below 20%), and then follows the sticks.

Check the stream without a robot: `gamepad_sender.py --print` writes the commands
to stdout.

## Speed limits and acceleration

Full stick deflection maps to the receiver's limits (defaults are about half the
hardware maximum):

| Flag | Default | Range |
| --- | --- | --- |
| `--max-x` | 12 | 1–25 |
| `--max-y` | 8 | 1–18 |
| `--max-turn` | 40 | 1–100 |
| `--ramp` (seconds) | 0 (off) | 0–10 |
| `--ramp-start` (fraction) | 0.4 | 0.1–1 |
| `--arm-speed` (mm/s) | 60 | 5–150 |
| `--claw-speed` (per s, of 0–255) | 170 | 20–500 |
| `--arm-home-x` / `--arm-home-z` (mm) | 80 / 30 | inside the reach ring |
| `--claw-start` | 128 | 0–255 |
| `--kneel-pitch` (degrees, front down) | 10 | 0–10 |
| `--kneel-height` (mm) | 70 | 60–110 |
| `--stand-height` (mm) | 85 | 60–110 |
| `--kneel-time` (seconds, stand → full kneel) | 2 | 0.5–10 |

With `--ramp N`, holding forward/back or a strafe in one direction grows the
stride from `--ramp-start` × the max to the full max over N seconds; it resets
when the stick centres or reverses, LB is released, arm mode starts, or the
watchdog fires. Turning is not ramped. Raise the max to use the extra range,
for example `--ramp 2 --max-x 25 --max-y 18`.

All of these can be given to `gamepad_sender.py`, which passes them through.

## When the robot stops

The receiver calls `stop()` when any of these happens:

- all sticks return to the deadzone
- LB is released
- no command arrives for `--timeout` (0.5 s)
- the stream ends (quit, Ctrl+C, SSH or Tailscale drop)
- a malformed command arrives
- the receiver gets SIGHUP or SIGTERM
- the receiver exits for any reason (`finally`)

Commands that queued up while the UART was busy are skipped, so it always acts on
the newest one.

Barks play with `aplay` on the robot's default sound device (the WM8960 speaker);
presses during a bark are ignored. The sound is credited in `sounds/CREDITS.md`.

These are software stops. They do **not** protect against SIGKILL, a hung robot
process, or the motion controller ignoring the UART, so stay within reach of the
power switch. After quitting, the robot is left standing, and the boot menu stays
stopped until you restart it or reboot.

Without the dead-man switch (`--no-deadman`), the sticks must be centred once
before anything moves.

## Tests

```bash
.venv/bin/python -m unittest discover -s scripts/teleop -p 'test_*.py'
```

Offline only: covers stick-to-axis mapping, deadzone, dead-man/arming, scaling,
clamping, bad input, watchdog, EOF, interrupt and stale-backlog handling, using a
fake dog. They do not prove physical behaviour or latency.
