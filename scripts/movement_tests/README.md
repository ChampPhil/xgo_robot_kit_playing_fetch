# XGO Lite movement tests

`basic_movement.py` sends a short command for each **selected** movement, then calls `stop()` before the next command. Nothing moves when you run `--help`; a normal run requires at least one `--movement` flag and a confirmation before opening the robot's serial port. Ctrl+C also stops motion. It does not use the camera or screen.

The separate repository-root `xgo_starter.py` is a camera/LCD/button example: A waves, holding C/D turns, and B quits. It needs additional robot-specific camera/GPIO/display libraries and is **not** covered by this movement-test environment.

## Set up on the robot

From the repository root on the **XGO Lite's Raspberry Pi**, with Python 3 (3.8+) and network access:

```bash
./setup.sh
.venv/bin/python scripts/movement_tests/basic_movement.py --help
```

`setup.sh` creates `.venv/` and installs `requirements.txt` (`xgolib` and its `pyserial` dependency). It is safe to rerun. If `python3 -m venv` is unavailable on Raspberry Pi OS, install `python3-venv` first. Alternatively, activate with `source .venv/bin/activate` and use `python3` in place of `.venv/bin/python`. The robot's preinstalled `xgolib` may also work with system Python, but this setup provides a reproducible environment. This script requires access to `/dev/ttyAMA0` (or specify `--port`); if the built-in menu is using the serial port, stop that process before testing.

**Clear a level area** and keep the robot away from edges before proceeding. Example commands from the repository root:

```bash
.venv/bin/python scripts/movement_tests/basic_movement.py --movement forward
.venv/bin/python scripts/movement_tests/basic_movement.py --movement turn-left --duration 0.5 --turn-speed 20
.venv/bin/python scripts/movement_tests/basic_movement.py --movement forward --movement backward --x-step 6
.venv/bin/python scripts/movement_tests/basic_movement.py --movement all
```

`--movement all` runs forward, backward, left, right, turn-left, turn-right in that order. Do not combine `all` with other movement selections. You can repeat `--movement` to choose an ordered subset. Each movement is timed, **not** a command to travel an exact distance or angle. The program stops after each move and on interruption; it does not return the robot to its starting position.

## Flags

| Flag | Meaning | Default / limits |
| --- | --- | --- |
| `--movement NAME` | Required. One or more `forward`, `backward`, `left`, `right`, `turn-left`, `turn-right`, or `all`. Repeat to run several. | No default |
| `--duration SECONDS` | Time each selected movement runs. | `1`; range `0.1–5` |
| `--pause SECONDS` | Rest between selected movements. | `0.5`; range `0–60` |
| `--x-step VALUE` | Forward/backward **stride step size**, not travel distance. | `10` mm; magnitude `1–25` mm |
| `--y-step VALUE` | Left/right sideways stride step size, not travel distance. | `8` mm; magnitude `1–18` mm |
| `--turn-speed VALUE` | Turning rate, not target angle. | `30` degrees/second; magnitude `1–100` degrees/second |
| `--port PATH` | Serial device to communicate with the robot. | `/dev/ttyAMA0` |
| `--yes` | Skip the start confirmation, e.g. for supervised noninteractive use. | Confirmation required |
| `--help` | Show available flags; no robot connection. | — |

Values for step size and turning speed are positive magnitudes; the selected movement determines the sign. The XGO Lite's directions are relative to the robot's body:

| Movement | Axis and direction | Library call |
| --- | --- | --- |
| `forward` / `backward` | +X / −X (forward/back along the floor) | `move_x(+step)` / `move_x(-step)` |
| `left` / `right` | +Y / −Y (sideways translation along the floor; **no turning**) | `move_y(+step)` / `move_y(-step)` |
| `turn-left` / `turn-right` | +yaw / −yaw (rotation about vertical axis, degrees/second) | `turn(+speed)` / `turn(-speed)` |

X and Y are the two horizontal axes; yaw is rotation around the vertical Z axis. There is no vertical/up-down movement in this test. The pinned `xgolib` package accepts x steps in `[-25, 25]`, y steps in `[-18, 18]`, and turning speeds in `[-100, 100]` degrees/second for Lite; the CLI constrains the positive magnitude and applies the correct sign for each direction. Some older hardware documentation lists a 150-degree/second turn limit, but this package clamps at 100. Stride size and time affect the approximate displacement, which varies with surface and gait.

For offline checks (no robot), run `python3 -m unittest discover -s scripts/movement_tests -p 'test_*.py'` from the repository root.
