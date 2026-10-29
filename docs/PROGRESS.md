# XGO Lite project: plan, progress and daily log

Latest continuation handoff: [HANDOFF-2026-10-29.md](HANDOFF-2026-10-29.md). Read it first for
the interrupted setup work, newer uncommitted edits and latest disk/connectivity observations.

Last updated 2026-10-29. Code: `scripts/teleop/` and `scripts/stacking/`; robot checkout
`~/project/xgo_robot_kit_playing_fetch` last verified at `da99946` on 2026-10-26.
Precision changes described below were deployed as a patch over that commit on 2026-10-26;
240 teleop tests passed on the Pi, followed by a user-confirmed pickup with the new version.

## Goal

**Long term:** the robot finds coloured 25 mm blocks anywhere in view, picks the requested
colour and stacks them at a marked spot in a requested bottom-to-top order.

**Current target:** three-color pickup, verification and tower sequencing. The new
`scripts/stacking/stack_boxes.py` supports multi-box observation/tracks, user-selected
bottom-to-top order, bounded miss/drop retries, marker alignment, separate taught layer
placements and visual tower verification. It runs end to end on synthetic images and was
deployed on 2026-10-28, preserving unrelated robot edits. **No live stacking test:**
carry/place/recovery routes and held/floor/stack verification views still need teaching.
The user selected a printed marker; an ID-0 PNG is included.

**Existing hardware evidence:** Fetch (F: reset → align → replay)
completed a user-confirmed pickup on 2026-10-23 in 54 s. An initial 2026-10-26 trial grasped
the box partially; after precision changes, a requested retry **held and lifted the box** in
about 55 s. The robot is stopped after that one successful trial; further runs require a new
request. The end-to-end time target remains under ~70 s.

**Approach:** teach and repeat. A person drives a pickup once (recorded with X); the robot later
re-creates the starting view with the camera (alignment) and replays the recorded arm/claw
motion. Recordings also store video + commands for training a model later.

## Milestones

| # | Milestone | Status | Evidence / what's missing |
| --- | --- | --- | --- |
| 1 | Connect and move safely (SSH over Tailscale, movement CLI, read-only sensor logger) | ✅ Done | Used in every session |
| 2 | Find coloured boxes in the camera image | ⚠️ Implemented; scene-dependent | Wider purple range helps at distance but also accepts blue chairs / floor patches; crouched view identified the real box on 2026-10-26 |
| 3 | Drive it by hand: gamepad and keyboard, arm/claw inside the measured reach ring, kneel, bark, live video with target box, claw reset, full reset | ✅ Done | All used on the robot |
| 4 | Record a pickup and replay it (X / Y) | ✅ Done | **Replay picked the box up** (2026-10-15) from the recorded spot; replays now shortened 50 s → 23 s |
| 5 | Align: get back to the recording's first view by itself (hold A / Space) | 🔄 Tighter gate passed one hardware trial | 2026-10-26 new version: eu +0.017 / eh −0.017 / ev +0.010 on three settled observations, followed by a confirmed pickup. More positions need testing |
| 6 | Fetch: one press = reset → align → replay (F) | 🔄 Two user-confirmed successes across versions; reliability unmeasured | 54 s on 2026-10-23; ~55 s with new precision version on 2026-10-26. Other trials failed or partially grasped. Sequence completion alone is not grasp success |
| 7 | Walk straight (heading hold) | ⚠️ Built, unproven | It still curved in the one test; gyro drift readings were inconsistent (+6.9, −19, 0 deg/s in different conditions). Alignment no longer depends on it |
| 8 | Know the grab worked (claw readback / camera check) | 🔄 Implemented offline; calibration pending | Positive held-region box plus no floor-region box; ambiguous/missing evidence stops. Optional independently calibrated readback bands. No physical validation yet |
| 9 | Stacking: separate pick/place recordings, marked spot, requested color sequence | 🔄 Implemented offline; taught assets pending | Multi-box tracks, bounded retries, marker approach, 0/25/50 mm layer profiles, verified color/geometry order before next layer. Simulation is not a physical tower |
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

1. **Final alignment is too loose for repeatable pickup.** The partial-grasp trial ended at
   eu −0.03 (about 19 px left at 640 px width) / eh −0.07 (7% smaller than reference).
   That passed the deployed 4% horizontal / 8% size limits. It is evidence of residual
   mismatch, not proof of the sole cause of the miss; grasp geometry is not calibrated.
   Deployed changes halve the tolerances, require three separated observations with both raw
   frames inside tolerance, damp turns after crossing the target, and use smaller/slower
   corrections. If minimum turns still overshoot, alignment stops. One tighter-gate hardware
   trial succeeded; repeatability across positions, lighting and orientations remains unmeasured.
2. **Heading hold** still curved; gyro drift is inconsistent. Possibly fine after the startup
   reset (drift 0 in the probe) — needs one straight-walk comparison with and without it.
3. **Two people work on the robot checkout.** `scripts/xgo_robot/` has uncommitted edits on the
   robot (5 modified tracked files rechecked on 2026-10-26); preserve them during deployment.
4. **Disk** was 284 MB free before the 2026-10-26 trial and 492 MB afterwards; cause of the
   increase was not investigated. Both readings are historical. Use `--no-record`; do not
   delete recordings without authorization.
5. **Calibration/reference are tied to the recording's pose** (kneeled, arm at default). A new
   recording made standing or with the arm moved needs a new calibration (automatic).
6. **Command interruption:** the first 2026-10-26 attempt stopped on the 0.5 s watchdog.
   A subsequent 10 s transport probe received all 200 messages with max arrival gap 0.154 s;
   the retry completed. The original pause's cause remains unknown; the watchdog stays enabled.

## Next steps (in order)

1. Precision version is deployed: **265 offline tests** passed locally, 240 teleop tests
   passed on the Pi, and one hardware fetch held and lifted the box. Preserve both this
   uncommitted deployment and unrelated robot edits when making future updates.
2. On the next requested hardware session, test multiple starting positions and record
   alignment residuals, elapsed time, and the user's grasp outcome for each. Prefer
   alignment-only (Space) when investigating a new position before attempting pickup.
3. Investigate distant-box detection: a visible box just above the fixed 40% floor cutoff
   was rejected. The user moved it closer before the successful precision-version trial;
   that run does not validate far-box fetching.
4. Decide on heading hold (keep, fix, or remove) from one straight-walk comparison.
5. New stacking setup: print the marker, teach a carry/check view exposing the held box and
   floor separately, then named carry/place/inspect/return/recovery motions. Use the new
   `--record-motion-only` option to teach routes without a target box or video dataset.
6. Validate one layer before adding the 25 and 50 mm support-surface placements. No guessed
   Cartesian heights, automatic reverse-walking or disappearance-only grasp success.

## Stacking software (2026-10-28, deployed; offline validation only)

- Entry point and instructions: [scripts/stacking/README.md](../scripts/stacking/README.md).
- `--observe` reports all colors and continuous IDs for unique detections; `--simulate`
  exercises actual color/marker detectors on synthetic floor/held/tower images.
- `--execute` is a Mac-to-Pi 20 Hz heartbeat launcher. The Pi checks every taught asset
  before XGO construction and shares camera/UART across alignment, replay and telemetry.
- A confirmed floor miss/drop can retry, at most three attempts per color by default.
  Unknown evidence stops. A placement is committed only after marker-anchored inspection
  finds the requested visible bottom-to-top order and adjacent aligned faces.
- The example workcell profile is disabled and has no guessed arm poses or image ROIs.
  SSH was used to deploy and run synthetic/mock tests; no real camera, XGO constructor,
  UART or motors were initialized. Existing voice/chase edits were preserved.
- 308 offline tests pass on Mac and Pi (242 teleop, 41 stacking, 6 movement, 8 detection,
  11 telemetry), plus local pyflakes and Python 3.9 grammar checks. A Pi OpenCV 4.7 marker
  grouping difference was reproduced and fixed; marker-size and separate-print ambiguity
  regressions pass. Existing test resource warnings remain.
- After deployment/tests completed, SSH timed out on both attempts at the final hash audit
  and stacking-guide refresh. The local guide is current; the Pi may retain the earlier
  "not deployed" wording. No physical test or further hardware commands were attempted.

## Session checklist (every time the robot is used)

1. **Tailscale on** on the Mac (it has been found stopped several times; SSH then times out).
2. Robot **on the floor**, charger **unplugged**, battery **≥ 20%** (the program refuses below).
3. After a reboot the vendor **boot menu holds the motor port** — stop it (or ask Claude to).
4. Nothing else using the camera/motors (camera stream, chase program).
5. Check local and robot Git status; preserve edits. For an authorized update, use
   `git pull --ff-only` in the robot checkout, then `sync`
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
| 10-21 | Review fixes; learned slow walk start-up, backup after overshoot / clipped bottom, and posture search. Low battery halted further trials |
| 10-23 | **54 s fetch with user-confirmed pickup.** Turn coasting fix deployed (`da99946`). Follow-up stopped during alignment; floor / chair false detections investigated but no detector fix committed before the Mac slept |
| 10-26 | Battery 89% before startup; first attempt cancelled by command watchdog. Retry began at 86%, aligned and completed replay in ~51 s from F; user reported **partial grasp / slight miss** and stopped fetching. Receiver exited cleanly, camera/UART free, boot menu remains stopped. Local precision changes developed afterwards; 265 offline tests, pyflakes and Python 3.9 grammar checks pass. Not deployed or hardware-tested |
| 10-26, precision retry | User requested testing the new version; patch deployed while preserving unrelated robot edits, 240 teleop tests passed on Pi. Initial far box was below detection limits / above floor cutoff, so no fetch ran; user moved it closer. Trial began at 80% battery: 14 alignment moves, eu +0.017 / eh −0.017 / ev +0.010 confirmed across three settled looks, then replay. About 55 s from F; user confirmed **held and lifted**. Receiver stopped cleanly; camera/UART free; boot menu stays stopped. Disk 276 MB free after trial. No repeat trial started |
| 10-28 | Implemented separate multi-box tower script, fresh-frame tracks, bounded pickup/drop retries, positive visual grip and tower checks, ArUco marker matching/generation, strict named-motion profile validation, one-owner live backend with SSH watchdog, and simulation. Added command-only route teaching. Deployed code with hashes checked and unrelated robot work preserved. Fixed Pi OpenCV 4.7 candidate grouping; all 308 offline tests pass on Pi and Mac. No real camera/UART/motor initialization or physical stacking attempted. Printed marker selected by user; live teaching remains next |
| 10-29 | Read-only continuation: SSH restored; root disk only 73 MB free. Inspected two existing purple recordings and newer uncommitted teleop edits; 41 local stacking tests passed. Proposed setup report/importer not yet written. User redirected to handoff documentation; see HANDOFF-2026-10-29.md. No physical camera/UART/motor work |
| 10-29 | Fixes from the 10-23 failed fetch and a review of the 10-26 changes (offline only): bright floor (H106 S48 V135) no longer passes as a "too close" purple box — pale purple now counts only when dark (detector accepts several ranges per colour); quarter-speed final steps no longer count as "walk did not start" or as walking-speed samples; fetch aligns to and replays the newest *box* recording, so a newer route recording cannot hide it (Y still replays the newest). 247 teleop, 41 stacking, 9 detection tests pass. Not yet deployed: the Pi checkout carries copied-in files, so a plain `git pull` would conflict |
