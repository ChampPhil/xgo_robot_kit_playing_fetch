# XGO Lite robot experiments

This repository develops an XGO Lite robot that fetches colored blocks and, eventually,
stacks them in a requested order. Single-box teach-and-repeat pickup has worked on hardware.
The new three-box stacking workflow is implemented and tested offline; its carry/place
motions and verification views still need teaching before a physical tower run.
Current evidence and open work are in [docs/PROGRESS.md](docs/PROGRESS.md). A 30 October snapshot of what the robot can do is in [docs/Current Robot 10-30-26.md](docs/Current%20Robot%2010-30-26.md); the quarter recap is [docs/Breakdown of Existing Progress.md](docs/Breakdown%20of%20Existing%20Progress.md).

## Getting started

From the repository root, `./setup.sh` creates `.venv/` and installs `requirements.txt`. See:

- [Colored-box detection](scripts/box_detection/README.md) — camera/photo/video CLI for purple, orange, and light-blue boxes; no motors.
- [Movement tests](scripts/movement_tests/README.md) — explicit, timed movement CLI; commands motors only after confirmation.
- [Sensor monitor](scripts/sensor_monitor/README.md) — live, read-only UART telemetry (battery, attitude, joints) in the terminal; requires the built-in menu to release the serial port.
- [Teleop and pickup](scripts/teleop/README.md) — gamepad/keyboard, recording, alignment and fetch.
- [Voice commands](scripts/voice/README.md) — say **hey jarvis**, then one motion or preset action (openWakeWord + Whisper).
- [Three-box stacking](scripts/stacking/README.md) — multi-box tracking, input color order,
  bounded retries, grip/tower verification, printed marker, simulation and taught-workcell execution.

For example, with a local photo:

```bash
.venv/bin/python scripts/box_detection/detect_boxes.py \
  --order purple orange light-blue --source /path/to/three_boxes.jpg \
  --output annotated.png
```

## Robot hardware (published XGO Lite V2 / CM4 kit specifications)

**Variant note:** The robot's boot software identifies its firmware as XGO-LITE and runs a Raspberry Pi CM4-based kit; we have not verified the exact hardware revision. The specifications below describe the documented **Lite V2 CM4 kit**, not measured characteristics of this individual robot.

| Component | Documented specification |
| --- | --- |
| Size and weight | 250 × 145 × 170 mm in the default stance; **575 g** in the CM4-kit specification. |
| Control | Raspberry Pi CM4 (BCM2711 quad-core Cortex-A72, 1.5 GHz) for Linux/camera applications; ESP32-WROVER board for real-time motion control via UART. |
| Motors/joints | **15 active joints**: three bus servos per leg (12) plus back-mounted arm/gripper actuation (3). Each listed servo: 2.3 kg·cm stall torque, 0.1 s/60° speed, 4.8–7.4 V supply. The servo's 0–300° control range is **not** the safe joint range of an assembled leg. |
| Sensors/feedback | MPU6050 accelerometer/gyroscope for posture; joint-position and current feedback. Servos can report position, speed, voltage, temperature, and load. CM4 head adds a 5 MP OV5647 camera and dual MEMS microphones. |
| Human interface | 2-inch 320×240 color display, four buttons, speaker; micro-HDMI and USB-C on the CM4 module. |
| Battery | Two 18650 cells in a 2S, 2500 mAh pack; charger rated 8.4 V / 1 A. Manufacturer advises **not to operate while charging** or carry more than **20 g**. |

For motion code, X is forward/backward stride command (Lite range ±25), Y is sideways stride (±18), and yaw controls turn rate. **Stride command is not travel distance.** The Python guide lists turn rate ±150°/s; our pinned `xgolib` package clamps at ±100°/s, so the [movement CLI](scripts/movement_tests/README.md) caps it at 100. Body pose control is distinct from walking, and motor servo angles must not be assumed to match the bare servo's 300° capability.

Sources: [CM4 Lite V2 introduction](https://wiki.elecfreaks.com/en/pico/cm4-xgo-robot-kit/product-introduction/xgo-lite-v2-product-instruction/), [CM4 product parameters](https://wiki.elecfreaks.com/en/pico/cm4-xgo-robot-kit/product-parameters/xgo-lite-v2-product-parameters/), [Python API](https://wiki.elecfreaks.com/en/pico/cm4-xgo-robot-kit/advanced-development/python-development/), [safety instructions](https://wiki.elecfreaks.com/en/microbit/robot/xgo-robot-kit-v2/safety-instruction/).

The project's plan, progress and daily log are in [docs/PROGRESS.md](docs/PROGRESS.md).
