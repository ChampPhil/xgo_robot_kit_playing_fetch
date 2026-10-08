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

## Aligning with the box (hold A)

Holding **A** steps the robot until the box looks the way it did in the latest recording's
reference photo, then logs `ALIGNED - release A, then press Y`. Release A to stop at any time.
Design: `docs/superpowers/specs/2026-10-07-self-calibrating-alignment-design.md`.

- **Display-only by default.** Without `--align-motion`, holding A moves nothing; it logs the
  error (`eu` = sideways, `eh` = size, `ev` = height in the image) and the step it *would* take.
  Use this first to check the directions on the real robot.
- **Calibration (first A hold with `--align-motion`, or `--recalibrate`).** The robot puts the arm
  and kneel where the reference was taken, then turns a **full 360° in small stop-and-look steps**
  (the box leaves the view and comes back), then one step forward/back and left/right. That
  measures how each step moves the box in the image. Saved to `~/xgo_teach/calibration.json` and
  reused until the step speeds, step time or the recording's arm/kneel change. Keep about half a
  metre clear around the robot; it takes roughly a minute.
- **Aligning.** Each step: move briefly, stop, wait `--settle` (0.7 s) for the camera to stop
  shaking, then look at two sharp frames that agree. It picks the turn, walk or side-step predicted
  to help most and keeps refining its model from what each step actually did. Aligned = box
  centre within `--tol-u` (0.04 of the width) and size within `--tol-h` (8%) on 3 looks in a row.
- **It stops and says why** when: A is released; any stick/LB/RB/kneel input; no commands for
  0.5 s; the box is missing/ambiguous/at the edge 3 times; the camera stalls; the error grows 3
  steps in a row; 40 steps or 60 s; or the box is the right size but at a different height
  ("view mismatch": kneel/arm/floor differ from the recording). A is ignored while recording or
  replaying, and X/Y are ignored while aligning.

Floor only, supervised, within reach. Tuning: `--align-turn` (20), `--align-walk` (8),
`--align-strafe` (6), `--align-step` (0.3 s), `--align-max-steps` (40). All receiver options,
including `--color`, can be given to `gamepad_sender.py`, which passes them through.

## Live video

`gamepad_sender.py --video` shows the robot camera at `http://localhost:8090/` and opens it in
your browser. The tailnet allows only SSH from this computer to the robot, so the receiver
serves video on the robot's loopback (`127.0.0.1:8090`) and the sender forwards it through its
SSH session (`ssh -L 8090:127.0.0.1:8090`); nobody else on the network can view it. It shares
the camera with recording. The old camera stream (`test_xgo_connectivity`) must not be running.
If port 8090 is busy on this computer, SSH prints a warning and driving still works.

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
