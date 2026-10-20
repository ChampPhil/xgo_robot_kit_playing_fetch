# XGO Lite project: plan, progress and daily log

Last updated 2026-10-20. Code: `scripts/teleop/` (see its README "Code map"); robot checkout
`~/project/xgo_robot_kit_playing_fetch` at `dd2b98e`.

## Goal

**Long term:** the robot finds coloured 25 mm blocks anywhere in view, picks the requested
colour and stacks them at a marked spot in a requested bottom-to-top order.

**Current target:** **fetch** — one key press (F) and the robot gets itself to where a recorded
pickup started, then replays that pickup. Target time: under ~70 s.

**Approach:** teach and repeat. A person drives a pickup once (recorded with X); the robot later
re-creates the starting view with the camera (alignment) and replays the recorded arm/claw
motion. Recordings also store video + commands for training a model later.

## Milestones

| # | Milestone | Status | Evidence / what's missing |
| --- | --- | --- | --- |
| 1 | Connect and move safely (SSH over Tailscale, movement CLI, read-only sensor logger) | ✅ Done | Used in every session |
| 2 | Find coloured boxes in the camera image | ✅ Done | Near boxes reliable; far (~60 cm) purple needs the wider range used by alignment |
| 3 | Drive it by hand: gamepad and keyboard, arm/claw inside the measured reach ring, kneel, bark, live video with target box, claw reset, full reset | ✅ Done | All used on the robot |
| 4 | Record a pickup and replay it (X / Y) | ✅ Done | **Replay picked the box up** (2026-10-15) from the recorded spot; replays now shortened 50 s → 23 s |
| 5 | Align: get back to the recording's first view by itself (hold A) | 🔄 Built, **not yet run on the robot** | v1 (stop-and-look, 0.3 s pulses) failed on hardware: pulses that short never move the legs. v2 (gyro turns + timed walk segments) passes a simulator built from measured numbers; deployed 2026-10-20 |
| 6 | Fetch: one press = reset → align → replay (F) | 🔄 Built, not yet run end to end | Depends on 5 |
| 7 | Walk straight (heading hold) | ⚠️ Built, unproven | It still curved in the one test; gyro drift readings were inconsistent (+6.9, −19, 0 deg/s in different conditions). Alignment no longer depends on it |
| 8 | Know the grab worked (claw readback / camera check) | ⏳ Not started | Needed before the robot retries on its own |
| 9 | Stacking: separate pick and place recordings, place height +25 mm per layer, align to the stack spot (marker), sequence the colours | ⏳ Not started | Needs 5–8 first; a printed ArUco marker for the stack spot is recommended |
| 10 | Learned models (alignment / pickup) from the recorded dataset | ⏳ Later | Two recordings exist; LeRobot conversion not built |

## Measured on this robot (use these, don't re-guess)

| Fact | Value | When |
| --- | --- | --- |
| Arm reach | A ring 80–140 mm from the arm base, in front; targets outside are silently ignored | 2026-10-02 |
| Leg command start-up delay | Nothing moves for the first ~0.5–0.7 s of a command | 2026-10-15 probe |
| Turn rate (kneeled) | ~19 deg/s at speed 20, ~32 deg/s at speed 40 | 2026-10-15 probe |
| Camera field of view | ~94° across the 640 px width | 2026-10-15 probe |
| Gyro (fused yaw, reg 0x64) | Tracks turns correctly after the program's startup reset; drift 0 then, but 6.9 and −19 deg/s seen in other conditions; never wrapped to ±180 | 2026-10-14/15 |
| Still-frame sharpness | ~410 (Laplacian variance); blurry while walking | 2026-10-07 |
| Purple box at ~60 cm | ~11 px tall, S≈62, V≈40, hue up to ~175 (outside the normal purple range) | 2026-10-15 |
| Replay length | Recording 20261015-220825: 50 s raw, 23.1 s with pauses capped at 0.5 s | 2026-10-20 |

## Open problems

1. **Alignment has never run with motion on the robot.** Next session's main job.
2. **Heading hold** still curved; gyro drift is inconsistent. Possibly fine after the startup
   reset (drift 0 in the probe) — needs one straight-walk comparison with and without it.
3. **Two people work on the robot checkout.** `scripts/xgo_robot/` has uncommitted edits on the
   robot (5 files as of 2026-10-20); commit or push them so updates don't collide.
4. **Disk** is ~97% full (523 MB free); camera-stream AVIs fill it quickly. Run streams with
   `--no-record` or clear old AVIs.
5. **Calibration/reference are tied to the recording's pose** (kneeled, arm at default). A new
   recording made standing or with the arm moved needs a new calibration (automatic).

## Next steps (in order)

1. First real fetch: box ~2 ft in front, slightly to one side, on the floor. Watch the
   calibration turn, alignment moves and ALIGNED, then the replay. Tune speeds/tolerances from
   the log.
2. Repeat fetch from a few starting positions; note success rate and time.
3. Decide on heading hold (keep, fix, or remove) from one straight-walk comparison.
4. Grasp check from the claw readback (closing on a box stops short of fully closed).
5. Stacking design: named recordings (pick / place), per-layer place height, stack-spot marker.

## Session checklist (every time the robot is used)

1. **Tailscale on** on the Mac (it has been found stopped several times; SSH then times out).
2. Robot **on the floor**, charger **unplugged**, battery **≥ 20%** (the program refuses below).
3. After a reboot the vendor **boot menu holds the motor port** — stop it (or ask Claude to).
4. Nothing else using the camera/motors (camera stream, chase program).
5. Robot code up to date: `git pull --ff-only` in the robot checkout, then `sync`
   (a power cut once corrupted the checkout right after a pull).
6. Start: `.venv/bin/python scripts/teleop/gamepad_sender.py --keyboard --video --align-motion`
   — robot kneels; stand still ~3 s before walking.
7. Afterwards: check free disk; quit with Esc (the robot stands up).

## Daily log

| Date | What happened |
| --- | --- |
| 09-14 → 09-23 | Repository set up; connection attempts; Tailscale access for SSH documented |
| 09-28 | Basic movement test and movement CLI |
| 09-29 | Camera colour-box detection; robot specs gathered |
| 09-30 | Read-only sensor logger (battery, attitude, servo angles); single-colour detection; visual-alignment research; first claw open/close tests |
| 10-02 | **Gamepad teleop** over SSH (F310 must be in D mode on macOS); bark, arm/claw mode, speed ramp; **arm reach measured** (ring 80–140 mm) after the arm seemed to freeze; kneel |
| 10-05 | Teach-and-repeat design and plan; **recording (X) / replay (Y)**, live video; independent review fixed 4 safety bugs; robot checkout repaired after a **power-loss corruption**; video moved into the SSH tunnel (tailnet only allows SSH) |
| 10-07 | Camera-shake handling (settle + sharpness); first alignment design and implementation (step-and-look, 360° calibration) |
| 10-08 | Box only visible from a stand-off distance → **walking added to recordings and replay**; BACK resets the claw |
| 10-08 → 10-14 | First real recording analysed: the approach was blind (box out of view 30 s) and walking was recorded as 145 tiny speed changes, so replays landed differently each time |
| 10-14 | **Keyboard control** (`--keyboard`, fixed speeds for cleaner recordings); **heading hold** with gyro; gyro drift found (6.9 deg/s) and compensated |
| 10-15 | Full reset (Z/RT) and start kneeled; recording made from the kneeled pose — **replay picked the box up**; F = fetch; red target box and video in the keyboard window; **pulse probe** showed 0.3 s commands never move the legs → alignment rebuilt around gyro turns and timed walks; far-box detection widened; Philip pushed the `scripts/xgo_robot/` detection/chase library |
| 10-20 | New alignment deployed to the robot (tests pass there); replay now 23 s; **code cleanup**: `teleop_receiver.py` split into protocol/body/actions; this progress document |
