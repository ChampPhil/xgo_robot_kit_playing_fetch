# Breakdown of existing progress

XGO Lite / Raspberry Pi CM4 kit. Course work from about 14 September 2026 through 30 October 2026. Repository: `xgo_robot_kit_playing_fetch` (GitHub `ChampPhil/xgo_robot_kit_playing_fetch`). Robot checkout: `/home/pi/project/xgo_robot_kit_playing_fetch`. SSH is `pi@100.74.30.90` over Tailscale, not Guest Wi-Fi.

This note is a quarter recap of what we actually got onto the dog, what we measured, and what still does not work. It is not a design spec. Long-term goal written in the repo: find coloured 25 mm blocks, pick a requested colour, stack them in order. What we built this quarter is the infrastructure for a **teach-and-repeat fetch**, plus a separate **chase** stack and a late **voice** demo.

---

## What the robot is

The kit is a small quadruped with a Compute Module 4 in the head, an ESP32 motion board on UART (`/dev/ttyAMA0`), a 5 MP camera, dual MEMS mics, a speaker, a 2-inch LCD, and a three-joint arm/gripper on the back. Firmware reports XGO-LITE. Published Lite V2 numbers (size, 15 active joints, 2S battery) are what we treat as the hardware; we did not re-weigh the unit.

Motion is **not** meters. `move_x` / `move_y` / `turn` are stride and yaw-rate commands. Lite ranges we use: X ±25, Y ±18, turn ±100 °/s in our pinned `xgolib` (some wiki pages say ±150). Sending a speed without `stop()` leaves the gait running. Opening `XGO()` typically runs `reset()` (action 255), so connecting the library twitches the body once.

Manufacturer advice we treat as real: do not operate while charging; do not carry more than about 20 g.

---

## September: getting onto the machine

The first two weeks were not perception. They were “can we talk to it at all.”

The built-in QR Wi-Fi flow showed success on the LCD and then did nothing useful. Trinity Guest and hotspot QRs both failed. Guest is captive-portal plus client isolation: a `10.202.x.x` address on the robot is real, but laptop SSH to it times out.

HDMI plus a keyboard got us a Raspberry Pi OS desktop. TrinityGuest from the terminal worked after re-enabling NetworkManager on boot. SSH still failed because of isolation. A hotspot attempt crashed the machine (browser plus RAM is a plausible cause). A 5 GHz hotspot was invisible; 2.4 GHz is what this radio wants.

Tailscale is what stuck (logged 18 September). The robot joins the tailnet on boot. The tailnet is scoped so the Pi is not an open hop. SSH is `pi@<100.x.x.x>`. First useful onboard test: write the camera to an AVI on disk.

That constraint never went away. Later video had to ride inside the SSH session because the tailnet grant is SSH-shaped, not “every port on the Pi.” A live HTTP page on 8080 only works if the Tailscale ACL actually allows it.

Repo scaffolding in this window: initial commit 14 September; README connection log 16–23 September; `setup.sh` and a pinned `requirements.txt` (`xgolib`, pyserial, headless OpenCV).

---

## Late September: move, see colour, read the bus

28 September: a movement test, then a real CLI (`scripts/movement_tests/basic_movement.py`). You pick `forward` / `backward` / `left` / `right` / `turn-left` / `turn-right` (or `all`). Each burst has a duration cap (0.1–5 s). `stop()` runs in `finally`. Confirmation is required unless `--yes`. That CLI is still the honest way to prove UART motion without the camera.

29 September: camera-only coloured-box detector (`scripts/box_detection/`). Purple, orange, light-blue HSV. It **observes**. It does not walk, pick, or stack. Near boxes are reliable. Far purple (~60 cm) is a few pixels and sits outside the first hue band; that bit us later in alignment.

30 September: read-only sensor logger (`scripts/sensor_monitor/`) for battery, attitude, labeled servos. The vendor boot menu often holds `/dev/ttyAMA0`; you have to kill it before any of our programs. Same day: notes on single-colour selection and visual pickup alignment. First claw open/close tests.

Safety habits from this stretch, still in force: floor clear of stairs; charger unplugged; battery not empty; one owner of the UART and one owner of the camera.

---

## 2 October: drive it by hand

Gamepad teleop over Tailscale SSH (`scripts/teleop/`). Sender on a laptop (F310 in D-mode on macOS, or later a keyboard window). JSON ticks go into SSH stdin. Receiver on the Pi calls `move_x` / `move_y` / `turn`.

Dead-man switch: walking only while a bumper or key is held. Watchdog stops the dog if the pipe dies. Bark on B. Arm/claw mode on RB (or arm keys). Kneel (pitch front down, lower body) so the camera and claw can see a floor box.

The arm “froze” until we measured the real reach. Firmware **silently ignores** targets outside a ring **80–140 mm** from the arm base, in front of the body. We clamp to that ring. Claw is 0 open … 255 closed. Arm pose cannot be read back in a useful way, so the first arm move homes to a known start (default 80, 30 mm) and we integrate from there.

This is milestone 3 in `docs/PROGRESS.md`: drive by hand. Done, and still how we make recordings.

---

## 5 October: teach-and-repeat, not a planner

Design choice: do not solve general pick-and-place this quarter. Record a human pickup once (X), replay it (Y). Store a reference view, motion events, and video so a model could be trained later (LeRobot conversion was not built).

What landed in code that day: background camera reader; X/Y counters on the protocol; recorder; replay; `--video`; then a review pass (non-blocking camera, safer replay start, disk errors, power-cut handling).

A power loss **corrupted the git checkout on the robot**. After that, `git pull --ff-only` plus care around power is part of the ritual.

Live video could not be a random HTTP port on the tailnet. It was moved **into the SSH tunnel**.

Camera-shake: wait for settle; reject blurry reference frames (Laplacian variance; walking frames are mush).

---

## 7–8 October: alignment v1, then walking in the tape

Self-calibrating alignment: get the live camera back to the recording’s first view. First design was stop-and-look with short pulses and a 360° calibration turn.

That design **failed on hardware**. Probe (15 October): a command does **nothing for the first ~0.5–0.7 s**. Pulses of 0.3 s never move the legs. Alignment had to be rebuilt around **gyro turns** and **timed walk segments**, not 0.3 s jabs.

8 October: the box is often only visible from a stand-off, so **walking had to go into the recording and the replay**. BACK/C resets the claw (open, arm home).

First real recording analysis (8–14 October): a long approach with the box **out of view for ~30 s**, and walking stored as **145 tiny speed changes**. Replays did not land in the same place. That is why keyboard control (fixed speeds) exists.

---

## 14–15 October: keyboard, gyro, first real pickup

`--keyboard`: hold W/A/S/D (and arm keys) in a small window. Shift for full speed. Cleaner tapes than analog sticks.

Heading hold: fused yaw (register 0x64) to keep “straight” from curving. Gyro **does** track turns after the program’s startup reset. Drift is not a constant: 0 deg/s in one probe, **+6.9** and **−19** deg/s in others. We subtract a measured drift. Heading hold still curved in the one dedicated test. Alignment was later told **not** to treat alignment turns as drift.

15 October was the pickup day:

- Start **fully kneeled**; full reset (Z/RT) stops, `dog.reset()`, claw open, kneel.
- Recording from that pose. **Replay picked the box up** from the recorded spot.
- F = fetch: reset, align by itself, replay. Built; end-to-end fetch still needed a working aligner on the floor.
- Target box overlay; video inside the keyboard window.
- Far-box HSV widened (purple at ~60 cm: ~11 px tall, hue up to ~175).
- Pulse probe → alignment v2 (gyro + timed walks).
- `scripts/xgo_robot/` detection/chase library pushed (person YOLO, coloured-ball HSV, live page, gait lock). That tree is the class chase homework; it is scheduled for deletion as a package name in later notes, but the chase behaviour is real work this quarter.

Measured that week (use these, do not re-guess):

- Leg start-up delay ~0.5–0.7 s.
- Turn rate kneeled: ~19 °/s at speed 20, ~32 °/s at speed 40.
- Camera FOV ~94° across 640 px.
- Still-frame sharpness ~410; blurry while walking.

---

## 20–23 October: make fetch testable

`teleop_receiver.py` split into protocol / body / actions. `docs/PROGRESS.md` written (plan, milestones, daily log). Session log also written under `~/xgo_teach/session.log`.

Replay of recording `20261015-220825`: 50 s raw, **23.1 s** with pauses capped at 0.5 s.

Alignment v2 deployed; tests pass on the robot. Then a week of **hardware-facing** alignment commits (21–23 October): purple hue starts at 105 because the box reads 103–118 in brighter light; learn slow walk start-up; back up after overshoot; ignore blobs that are not on the floor; back up when the box is cut off at the bottom; search other kneel levels when it leaves view; small turns at half speed; coast learned per speed.

That is the opposite of “alignment never left the laptop.” After 20 October we were tuning it **on the dog**. Fetch (F) is still “reset → align → replay” in software. Grasp success is **not** checked (claw readback / camera confirm not started). Stacking is **not** started. Learned models from the tapes are **not** started.

Disk: the 15 GB root filled with AVIs, pip wheels, and torch. Chase recordings and a failed Whisper venv install pushed us to ~99%. That is an operations problem, not a gait problem.

---

## Late October: chase stack and voice

Parallel to fetch, `scripts/xgo_robot/` (and the Windows homework copy) grew a camera pipeline: 1280×720 capture on the Pi, YOLO person boxes, HSV ball blobs, HTTP live page, telemetry on the same UART, `PersonChaser` / `BallChaser` with lock, confirm-before-walk, and conservative speeds. Guest isolation is why we keep saying “run Python **on the Pi**.” Two `XGO()` objects on `ttyAMA0` corrupt each other; stream and chase must not share the port with teleop.

Voice (28–30 October) lives in `scripts/voice/`. Goal: wake, then one phrase, then a timed move or a preset `action(id)`. Chase-by-voice and “walk N meters” were **explicitly not wired**.

First wake attempt was `local-wake`: three WAV clips of “hey xgo.” That is speaker-specific and a bad live-demo tool. We switched to **openWakeWord**, pretrained **hey jarvis**, no on-robot recordings. Commands go through **Whisper `tiny.en` on CPU**, not a cloud API. `torch` was already on the Pi; a second copy in `.venv` would not fit. Install is `python3 -m pip install --user`.

Command map after wake: stop; sit / lie down / stand / wave / stretch / handshake; go forward / go back; slide left / slide right; turn left / turn right. Speeds match the movement CLI (x=10, y=8, turn=30, 1 s then `stop()`). Gestures use the Lite action table (sit=12, wave=13, …).

Demo bugs we hit in the last days, and what they mean:

- Saying “hey jarvis sit” in one breath lost “sit” because recording started **after** wake. Later builds keep a 2 s pre-roll.
- After “Speak your command…” the process looked dead: either a blocking `stream.read` or Whisper with **no print for 10–40 s**. The Pi is slow. Wait for `Heard:`.
- `Mic status: input overflow` during Whisper: Python’s GIL, callback cannot drain PortAudio. Stuffed queue then dumped **~20 s** into the next clip (the “21 second” recording). Fix: stop the mic during Whisper and drain the queue.
- ONNX “Removing initializer …” lines are unused weights in the jarvis model. Ignore them.
- Wake score 0.35 is the default threshold: weakest legal hit.

Voice is a **demo layer** on top of `xgolib`. It is not fetch. It is not chase. Only one of teleop, chase, movement CLI, or voice may own the UART.

---

## Where we actually are on 30 October

Working, repeatedly:

- SSH over Tailscale; movement CLI; sensor log; HSV boxes (near); gamepad and keyboard teleop; arm inside the measured ring; kneel; bark; record and replay a kneeled pickup that **did pick the box up**; shortened replay; alignment code that has been **tuned on hardware** after the pulse-probe failure; chase library (person / ball); voice listen with hey jarvis + a small English phrase list.

Not done, and we should not pretend otherwise:

- Reliable unattended fetch from arbitrary start poses (depends on alignment success rate, which we were still tuning).
- Heading hold as a solved straight-line walker.
- Grasp verification and retry.
- Stacking, markers, place-at-height.
- Voice chase / metered walking.
- A clean disk and a reproducible venv that can hold both teleop and Whisper without filling the SD card.

How to run the dog in one sentence: Tailscale up, vendor menu off the UART, battery not dead, charger out, `git pull` if the checkout is yours, then either keyboard teleop with `--align-motion` for fetch work, `python3 -m xgo_robot` chase commands for vision homework, or `python3 scripts/voice/voice.py listen` for the jarvis demo — never two of those at once.

---

## Milestone scoreboard (as of 30 October)

These are the ten rows from `docs/PROGRESS.md`, restated against what happened after that file’s 20 October snapshot.

1. Connect and move safely — done. Used every session.
2. Find coloured boxes — done for near; far purple needed extra hue. Alignment still fights lighting.
3. Manual drive (gamepad/keyboard, arm ring, kneel, bark, video, resets) — done.
4. Record and replay a pickup — done. Replay grasped the box on 15 October.
5. Align to the recording’s first view — built, failed as 0.3 s pulses, rebuilt, then tuned on the robot 21–23 October. Not a finished product; it is no longer “never run.”
6. Fetch (F) — software path exists. Not a polished one-button show. Needs 5 to be boringly reliable.
7. Heading hold — built, one curved test, drift inconsistent. Do not demo this as “it walks straight now.”
8. Know the grab worked — not started.
9. Stacking — not started.
10. Learned models from the dataset — not started. Two recordings exist; no converter.

---

## Layout on the robot

`scripts/box_detection/` — HSV three-colour observer.  
`scripts/movement_tests/` — timed gait CLI.  
`scripts/sensor_monitor/` — UART telemetry, no motion.  
`scripts/teleop/` — fetch stack: sender, receiver, camera, heading, align, teach, actions.  
`scripts/xgo_robot/` — chase/stream homework copy (YOLO person, HSV ball, HTTP page). Uncommitted edits on the Pi have collided with laptop copies; treat as messy.  
`scripts/voice/` — hey jarvis + Whisper + small command table. Mostly deployed by copy, not a GitHub commit before 30 October.  
`docs/PROGRESS.md` — living plan (last dated 20 October). This file is the quarter narrative.

Every session still needs: Tailscale actually running on the laptop (it has been found stopped); robot on the floor; charger unplugged; battery ≥ ~20% (teleop refuses below); vendor menu not holding the serial port; nothing else on camera or motors; enough free disk that Whisper and AVI recording do not fall over. Quit teleop with Esc so the dog stands.

That is the quarter: we went from “QR will not join Wi-Fi” to a dog we can SSH, walk, see, drive, tape a pickup, replay a grasp, chase a person or ball, and (slowly) take a spoken command after “hey jarvis.” Fetch as a robust one-press behaviour and stacking are next quarter’s problem, not this one’s leftover checkbox.

GitHub `main` for this repo runs from the 14 September initial commit through 23 October (`da99946` alignment coast/speed). Voice and the last disk/wake-word fight are on the Pi after that tip. If a date in this file disagrees with `git log`, trust the log. If it disagrees with `docs/PROGRESS.md` after 20 October, trust the later alignment commits and this recap.
