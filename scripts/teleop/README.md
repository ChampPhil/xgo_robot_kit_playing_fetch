# Gamepad teleop over Tailscale SSH

Drive the XGO Lite with a gamepad plugged into your computer. The sender reads the
sticks and pipes one JSON command per tick into an SSH session; the receiver on
the robot turns them into `move_x` / `move_y` / `turn` calls.

```
[gamepad] -> gamepad_sender.py (Mac) --stdin of ssh pi@100.74.30.90--> teleop_receiver.py (robot) -> /dev/ttyAMA0
```

| Control | Robot |
| --- | --- |
| **LB (hold)** | Dead-man switch: the robot only moves while held |
| Left stick up / down | Walk forward / backward |
| Left stick left / right | Strafe left / right |
| Right stick left / right | Turn left / right in place |
| START, or Ctrl+C | Stop and quit |

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

## Speed limits

Full stick deflection maps to the receiver's limits (defaults are about half the
hardware maximum):

| Flag | Default | Max (xgolib) |
| --- | --- | --- |
| `--max-x` | 12 | 25 |
| `--max-y` | 8 | 18 |
| `--max-turn` | 40 | 100 |

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
