# XGO Lite robot experiments

This repository develops camera perception and movement tests toward a fetch-playing robot. The current box detector **only observes** three colored boxes; it does not walk, pick, or stack. Run the tests on a clear, level surface and do not operate the robot while charging.

## Getting started

From the repository root, `./setup.sh` creates `.venv/` and installs `requirements.txt`. See:

- [Colored-box detection](scripts/box_detection/README.md) — camera/photo/video CLI for purple, orange, and light-blue boxes; no motors.
- [Movement tests](scripts/movement_tests/README.md) — explicit, timed movement CLI; commands motors only after confirmation.

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

## Project log

### 9/15

* The robot has a built in QR code scanner which should enable easy connect. We tried this and the the screen would display success; however, the robot never successfully connected. We attempted this for both a hotspot QR and also the school's network QR. We suspect that the school's QR code might not work since the school's wifi also requires interacting with a webpage to login. We also tried the hotspot at both 5 GHz and 2.4 GHz to maximize compatibility, but neither of them worked. 

### 9/16

* Through and HDMI cable, we were able to finally interact with the robot's OS, which was the Raspberry Pi OS. We were able to successfully connect to TrinityGuest from the terminal. One important note is upon boot the network manager has to be re-enabled. Despite our success in connecting, it seems that we cannot ssh into the robot due to per device isolation by the school's network. We then opted to try using a hotspot. We began setting this up; however, the machine lagged and stopped functioning potentially due to the browser being open in the background. We are not exactly sure why this crash occurred, but it could be due to memory issues. 

### 9/17

* Initially, we believed due to admin updates we would be able to ssh into the server. However, upon trying, ssh connection still failed. After trying to connect to the hotspot again, we noticed that the hotspot was not being detected at all by the robot -- in retrospect, this could have been because the hotspot was not on 2.4 GHz. This led us to try and use Tailscale to connect. Tailscale would enable us to the ssh into the robot and get around the school's network restrictions.

### 9/18

Tailscale has been successfully set up as the way to get the robot connected to our laptops. The robot has tailscale login credentials on disk, and on initialization connects to the tailnet. The tailnet has also been scoped for isolation (so the robot can only access allowed devices / ip addresses on the tailnet). Additionally, ssh now works directly between my laptop and pi@[Tailscale-Robot-IP-Address]. A test script was ran (successfully) on the XGO Robot Kit that streamed the camera feed to an .avi on disk
