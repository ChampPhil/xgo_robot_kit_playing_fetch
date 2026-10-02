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
