# Current robot — 30 October 2026

What this XGO Lite can do **right now**, as parked on the Pi at `~/project/xgo_robot_kit_playing_fetch`. Capability sheet, not a history. For the quarter story see `Breakdown of Existing Progress.md`.

---

## Machine

Desktop quadruped, XGO-LITE firmware, Raspberry Pi CM4 in the head, ESP32 motion board on **`/dev/ttyAMA0`**. Front camera (OV5647), dual MEMS mics (WM8960), speaker, 2-inch LCD, four buttons, back-mounted arm and claw.

SSH: `pi@100.74.30.90` over **Tailscale**. Run one motion program at a time so UART and camera stay with that process. After a reboot, quit the vendor menu first so it releases the serial port. Teleop asks for battery ≥ ~20% and a clear floor. Esc / Ctrl+C stops the legs.

How we command it: `move_x` / `move_y` / `turn` set a gait, then `stop()`. Connecting `xgolib` plays a short reset so the body starts from a known pose. Arm targets live in the **80–140 mm** ring in front of the body (measured on this unit). Claw 0 open … 255 closed. Kneel pitches the front down and lowers the body for floor work. Manufacturer payload guidance: about 20 g.

| Axis | Working range we use |
| --- | --- |
| Forward / back | `move_x` ±25 |
| Strafe | `move_y` ±18 |
| Turn | ±100 °/s |
| Arm | 80–140 mm ring, in front |
| Kneel | Front pitch to +10°, height ~70 mm |

---

## Walk on command

`scripts/movement_tests/basic_movement.py` — timed forward, back, strafe, and in-place turn, each finishing with `stop()`. Confirmation before the port opens (`--yes` when you already cleared the floor).

Preset body tricks (also used from voice): lie down, stand, sit, wave, stretch, handshake.

`scripts/sensor_monitor/` — live battery, attitude, and named servo angles on the same UART.

---

## See colour

`scripts/box_detection/` — HSV for purple, orange, and light-blue cubes. Near boxes lock cleanly. Alignment uses a wider purple band so a cube a couple of feet out still counts. The camera waits for a sharp still before it takes a reference photo.

---

## Drive by hand

`scripts/teleop/` — gamepad **or** `--keyboard`. Stick/key ticks go over SSH into `teleop_receiver.py`. Walking is hold-to-move (LB or WASD); release and it stops. If SSH drops, a watchdog halts the gait.

It walks, strafes, turns, moves the arm and claw in the measured ring, kneels, barks, opens the claw and homes the arm, and does a full reset into a kneel. Live video rides inside the SSH session and can draw the recording’s target box.

Keyboard (click the control window, then hold):

| Hold | Does |
| --- | --- |
| W/S | Forward / back |
| A/D | Strafe |
| Q/E | Turn |
| I/K, J/L, U/O | Arm up/down, in/out, claw |
| G/T | Kneel / stand |
| Shift | Full speed |
| Space | Align (hold) |
| X / Y / F / Z / C / B / Esc | Record, replay, fetch, full reset, claw reset, bark, quit |

Start (laptop, as in the teleop README):

```text
.venv/bin/python scripts/teleop/gamepad_sender.py --keyboard --video --align-motion
```

The dog kneels; a few still seconds lets heading hold sample the gyro.

---

## Teach-and-repeat pickup

**Record (X)** while a chosen-colour box is fully in view. Tape goes under `~/xgo_teach/` with the reference photo, walking and arm events, and video.

**Replay (Y)** plays that tape. A kneeled recording from 15 October picked the box up from the recorded spot. That replay runs about 23 s. Drive input aborts.

**Align (hold A / Space, `--align-motion`)** matches the live camera to the recording’s first frame with gyro turns and timed walks. It calibrates on a small first turn, can back up, change kneel height, and keep floor blobs. Lighting-aware purple HSV is in the detector.

**Fetch (F)** chains full reset → settle → align → replay. Cancel with F again, a drive key, Esc, or a dropped SSH.

Heading hold uses fused yaw with drift subtracted so commanded-straight walking can hold a line.

---

## Chase (camera + gait)

A working, filmed capability: the dog looks with the onboard camera, locks a target, and walks toward it. Code in `scripts/xgo_robot/`. Live page at `http://<tailscale-ip>:8080/` while it runs.

- **Person chase** — YOLOv8, person class only. Confirms the same box across frames, then walks and turns toward that lock with conservative speeds. Cyan outline on the chased instance. Browser shows boxes plus battery / IMU / servos.
- **Ball chase** — HSV + roundness for a named colour (orange, red, …). No neural net. Align (spin) until the blob is centered, then walk. Never both at once.
- **Stream** — same camera and page, no walking, for a live look at detections.

Run on the Pi (camera is on the robot). Ctrl+C stops the gait and closes the page. Keep a clear floor; one process on UART.

Example:

```text
python3 scripts/xgo_robot/main.py chase-person
python3 scripts/xgo_robot/main.py chase-ball --color orange
python3 scripts/xgo_robot/main.py stream
```

---

## Voice

`python3 scripts/voice/voice.py listen`

Wake on pretrained **openWakeWord “hey jarvis”** (no enrollment clips). Then a short command clip, transcribed with **Whisper `tiny.en` on the Pi**. Say the command right after `Wake detected`. Wait for `Heard:` (often 10–40 s on this CPU). The mic pauses during Whisper so the next capture stays clean.

| After wake | Robot |
| --- | --- |
| stop | Halt |
| sit / lie down / stand / stand up | Presets |
| wave / stretch / handshake | Presets |
| go forward / forward | 1 s then stop |
| go back / backward | 1 s then stop |
| slide left / slide right | 1 s then stop |
| turn left / turn right | 1 s then stop |

Threshold default 0.35. Walk speeds match the movement CLI (x 10, y 8, turn 30).

---

**Right now this robot can SSH over Tailscale, walk on command, show coloured cubes, be driven by gamepad or keyboard, record and replay a floor pickup, fetch after aligning to a taught view, chase a person or a coloured ball on camera (we have video), and take a spoken command after “hey jarvis.”**
