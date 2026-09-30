# XGO Lite live sensor telemetry

`stream_sensors.py` prints **read-only** sensor snapshots to the terminal until Ctrl+C (or `--count N`). It sends only XGO UART **read** packets, not motion commands. This is deliberate: the robot's preinstalled `xgolib.XGO(...)` constructor automatically sends `reset()` and can make the dog stand up. **The script does not create an XGO object.**

## Run on the robot

From the repository root on the Raspberry Pi, after charging the robot (do **not** operate while charging):

```bash
./setup.sh
sudo fuser -v /dev/ttyAMA0          # check who owns the UART
# If the built-in menu is using the port and you are ready to close its LCD menu:
sudo pkill -f '[m]ain.py'
.venv/bin/python scripts/sensor_monitor/stream_sensors.py --interval 1
```

The built-in `main.py` runs as **root** and normally holds `/dev/ttyAMA0`. The monitor **refuses to start while another process has the port**; it uses `sudo -n id -u` and root-level `fuser` to detect that process. It will not kill the menu itself. If sudo access is unavailable, it fails closed. Stopping `main.py` closes the built-in LCD interface until its normal startup is restored. Use Ctrl+C to end the stream and close the serial port.

For one JSON object per line (useful for logging or plotting), run:

```bash
.venv/bin/python scripts/sensor_monitor/stream_sensors.py --json --interval 2 --count 10
# Or keep the labeled terminal table and also append a JSON log:
.venv/bin/python scripts/sensor_monitor/stream_sensors.py --log sensors.jsonl
```

| Option | Meaning |
| --- | --- |
| `--port PATH` | Serial device; default `/dev/ttyAMA0`. |
| `--interval SECONDS` | Target interval between sample starts, 0.2–60 seconds; default 1. Individual UART timeouts can make sampling slower. |
| `--count N` | Stop after N samples; otherwise stream until Ctrl+C. |
| `--json` | Emit newline-delimited JSON instead of a labeled table per sample. |
| `--log PATH` | Append and flush each sample as JSON to a file. The parent directory must exist. |
| `--servo-profile NAME` | Degree conversion: `lite-2023` (default) or `lite-pypi-1.1.10`. Choose the profile matching the controller/library calibration, not just the Python package you installed. |

## What is actually reported

- **Battery percentage** from the dog controller (`1–100` on valid replies; a reported `0` is treated as invalid, not silently shown as 0%). This is *not* the Raspberry Pi's `vcgencmd measure_volts` core-voltage reading.
- **Roll, pitch, yaw** from the controller's fused attitude registers, in degrees. These are attitude angles, not the MPU6050's raw accelerometer/gyroscope axes.
- **15 labeled servo angle estimates** (12 leg joints, gripper, and two arm joints). Each `servos` entry in JSON contains `label`, `angle_deg`, and the original `raw` byte, keyed by servo ID. The terminal prints the same values as a table. Some older firmware provides only 12 motor bytes: the three arm/gripper entries then show `N/A` / JSON `null`, not zero.
- **Raspberry Pi CPU temperature** (°C) from Linux sysfs when present; this is **not** servo temperature or battery temperature.

### Servo names and angle conversion

Left/right refer to the **robot's** perspective, looking forward, not a person facing it. Joint digits 1, 2, 3 run bottom to top. We use the kit's elbow/shoulder/hip terminology; the position in parentheses avoids ambiguity (the rear-leg middle joint is also labeled shoulder).

| Position | Elbow (lower) | Shoulder (middle) | Hip (upper) |
| --- | --- | --- | --- |
| Front left | 11 | 12 | 13 |
| Front right | 21 | **22** | 23 |
| Rear right | 31 | 32 | 33 |
| Rear left | 41 | 42 | 43 |

| Arm ID | Label | `lite-2023` range | `lite-pypi-1.1.10` range |
| --- | --- | --- | --- |
| 51 | Gripper | −65…65° | −65…65° |
| 52 | Arm distal joint | −70…60° | −115…70° |
| 53 | Arm proximal joint | −90…105° | −85…100° |

Leg lower/middle/upper ranges in both profiles are −70…50°, −70…90°, and −30…30°. Conversion follows the library's `read_motor()` formula: `angle = low + raw / 255 × (high - low)`. Thus these are **quantized firmware readback estimates**, not independently calibrated absolute angles. Gripper degrees are not jaw opening in mm. The default profile comes from the robot's inspected 2023 `xgolib` 1.3.4 source; the alternative comes from this repo's pinned PyPI package. Firmware alone is not used to guess the arm calibration. Confirm the profile before interpreting arm angles. No joints were moved to validate the naming or calibrate the degrees.

### Coverage and failures

The hardware has camera and microphones, and the servos can internally measure voltage/temperature/load, but the robot's **installed 2023 Python API does not expose those as reliable live numeric readings**. Use the separate [camera/colored-box detector](../box_detection/README.md) for image data. Firmware and software versions are not periodically polled sensors.

If a query times out or returns bad data, its field is `null`/`?`/`N/A` and the `errors` field explains why; the script stops after two samples in which *all* five UART queries fail. A count-limited run with no successful UART readings also exits with code 2. Terminal output warns at battery ≤10% (a conservative test threshold, not a manufacturer specification). Never mistake missing data for a true zero angle or a healthy battery. The built-in menu and other UART clients must remain stopped for the whole session, not just at startup.

Offline protocol and formatting tests (no robot needed):

```bash
.venv/bin/python -m unittest discover -s scripts/sensor_monitor -p 'test_*.py'
```

Reference for supported readouts: [XGO Lite Python development documentation](https://wiki.elecfreaks.com/en/pico/cm4-xgo-robot-kit/advanced-development/python-development/). Exact coverage is based on the `xgolib` **1.3.4** source inspected on this robot; the repo's pinned PyPI package is a different release, so this monitor deliberately uses the documented read-only UART protocol instead of invoking either package's constructor.
