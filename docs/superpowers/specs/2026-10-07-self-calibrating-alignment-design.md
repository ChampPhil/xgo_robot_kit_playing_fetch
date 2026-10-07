# Self-calibrating alignment (hold A): design

Date: 2026-10-07. Status: approved direction (user chose option 2 + 360° calibration turn);
supersedes the "Alignment (hold A)" section of `2026-10-05-teach-and-repeat-design.md`.

## Goal

Holding **A** moves the robot until the box looks the way it did in the latest recording's
reference photo, then stops and says **ALIGNED — press Y**, so the recorded pickup can be
replayed. The robot learns how its own steps move the box in the image (a calibration "wiggle"
that includes a full 360° turn), then uses and keeps refining that model while aligning.

## Facts this design is built on

- The camera is on the head; **it shakes heavily while walking** and for a moment after. Only
  frames captured after the legs have been still for `--settle` seconds (0.7) are used, two in a
  row must agree, and each must pass the sharpness check (`--min-sharpness`, Laplacian variance,
  default 50; a still frame on the robot measured ~410).
- Step commands (`move_x`, `move_y`, `turn`) are speeds, not distances; strides vary. So the
  effect of a step is **measured**, never assumed.
- Arm and kneel change the camera view (kneel pitches the head down), so alignment first puts
  the arm and kneel exactly where they were when the reference was taken.

## Image features and error

From the target-colour detection (the existing `box_features`): centre `u`, `v` and height `h`,
normalised by frame size. Against the reference (`reference.json`):

- `e = (eu, eh)` with `eu = u − u_ref` and `eh = h / h_ref − 1` — what the steps drive to zero;
- `ev = v − v_ref` — a **consistency check** only: on a flat floor size and vertical position
  both track distance, so `|eh|` in tolerance with `|ev|` out of tolerance means the camera pose
  differs from the reference (kneel/arm/floor/box), which walking cannot fix → abort with
  "view mismatch".

## Step primitives and the motion model

Three primitives, each "drive at a small fixed speed for `--align-step` seconds (0.3), then
stop": **turn** (`--align-turn`, default 20), **forward** (`--align-walk`, default 8) and
**strafe** (`--align-strafe`, default 6). A primitive may be run with a fraction `a ∈ [−1, 1]` of
a full step (duration `|a| ×` step, minimum 0.12 s; sign = direction).

The motion model is one column per primitive, `J_p = (Δu, Δh/h_ref)` per full step (positive
direction). Predicted effect of running `p` with amount `a`: `a · J_p`.

## Calibration ("the wiggle")

Runs the first time A is held in a receiver session if there is no saved calibration, or always
with `--recalibrate`. It needs the box in view at the start. Every observation below is a
settled, sharp, two-frame-agreeing observation.

1. **360° turn, as stop-and-look steps.** Repeat full positive turn steps (turn left), observing
   after each. While the box is still in view the per-step change gives `J_turn`. The box leaves
   the view, later re-enters from the other side; the revolution is complete when its `u` crosses
   the starting `u` again. The step count `N` (with the last step interpolated linearly in `u`)
   gives **degrees per turn step = 360 / N**. Safety cap: 90 steps (abort "box not seen again").
   If the box is lost and found more than once (another object, a duplicate), abort.
2. **Forward/back probe:** one forward step, observe, one back step, observe.
   `J_forward` = average of the forward change and the negated back change.
3. **Strafe probe:** one left step, observe, one right step, observe → `J_strafe` likewise.

A primitive whose measured effect is below the noise floor (`|J_p| < 0.005`) is marked unusable
(e.g. strafe that barely moves the image) and never chosen.

The result is saved atomically to `~/xgo_teach/calibration.json`
(`{"version": 1, "created", "deg_per_turn_step", "columns": {"turn": [du, dh], "forward": …,
"strafe": …}, "speeds": {...}, "step_seconds", "kneel", "arm"}`) and reused next session unless
`--recalibrate` is given or the saved speeds/step time/kneel/arm differ from the current ones.

## Aligning

Each cycle (state machine ticked by the 20 Hz receiver loop, never blocking):

1. **Observe** a settled, sharp, agreeing pair → `e`, `ev`.
2. **Done?** `|eu| ≤ --tol-u` (0.04) and `|eh| ≤ --tol-h` (0.08) on **3 consecutive** observations
   → stop, log `ALIGNED — press Y`, and hold (no motion) until A is released.
   If in tolerance but `|ev| > --tol-v` (0.05) → abort "view mismatch".
3. **Choose a step:** for each usable primitive `p`, `a_p = clip(−(J_p · e) / (J_p · J_p), −1, 1)`
   and predicted residual `|e + a_p J_p|`; run the primitive with the smallest residual.
   (Far away this naturally picks turning for `eu`; close up, strafe if it is the better lever.)
4. **Learn:** after observing the result, update that primitive's column toward what actually
   happened (Broyden-style rank-1 update, rate 0.3): `J_p ← J_p + 0.3 (Δe / a_p − J_p)` when
   `|a_p| ≥ 0.25`. The in-memory model improves during the session; the saved file is not
   overwritten by these updates.

## Safety and aborts

Alignment only moves while **A is held** and only with `--align-motion`; without it, holding A is
**display-only**: it observes and logs `ALIGN eu=+0.06 eh=-0.12 ev=+0.01 → would turn left 0.4`
(using the saved calibration if any, otherwise plain sign rules) and never drives the legs.
Calibration needs `--align-motion` too.

Stop the legs and abort with a logged reason when: A is released; the receiver's watchdog fires
(no commands for 0.5 s); any stick, LB/RB or kneel input (manual); the target is missing,
ambiguous or touching the image edge on 3 consecutive observations (calibration's 360° turn
excepted, where losing the box is expected); no fresh settled frame within 2 s (camera stalled);
total error grows on 3 consecutive steps; more than `--align-max-steps` (40) steps or 60 s of
aligning, or 90 turn steps / 120 s of calibration; no recording/reference exists; a recording or
replay is in progress. A, X and Y exclude each other: A is ignored while recording or replaying,
and X/Y are ignored while aligning.

The robot walks on its own here, so this is for a **clear floor only**, supervised, within reach
of the robot. The 360° turn is in place but turning drifts; keep about half a metre clear around
the robot.

## Status

Log lines (stderr → Mac terminal): `CALIBRATE turn 7/~30`, `CALIBRATE done: 12.1°/turn step,
turn du=-0.071, forward dh=+0.064, strafe du=+0.032`, `ALIGN 5: eu=+0.06 eh=-0.12 → turn left
0.40`, `ALIGNED — press Y`, and every abort reason. The sender adds an `ALIGN` indicator while A is
held.

## Components

| Unit | Responsibility |
| --- | --- |
| `align.py` (new) | Pure: `errors()`, `choose_step()`, `broyden_update()`, `aligned()`, `Calibration` load/save/compatible; `Observer` (settled, sharp, two-agreeing frames); `Aligner` state machine (prepare → calibrate → align) driving a `legs` interface (`step(primitive, amount)` / `stop()`) and reading the camera. |
| `teleop_receiver.py` | Flags, `Message.align` (A held), routing: alignment owns the legs/arm/kneel while active; `LegStepper` adapter turning a primitive + amount into a timed `Driver` command. |
| `gamepad_sender.py` | Sends `"align": true` while A is held; status shows `ALIGN`. |
| `teach.py` | Expose the latest session's reference (`load_reference`). |

## Testing

Offline, with a simulated robot: a fake camera whose box features move by a known (hidden)
linear model per step plus noise, and frames during motion marked shaky. Tests: error/feature
maths; choose-step picks the best lever and clips; Broyden update converges on a wrong initial
model; observer rejects frames before settle, blurry frames and disagreeing pairs; calibration
recovers degrees per step from a simulated 360° (box leaves and re-enters) and the probe columns,
and aborts when the box never returns or returns twice; aligning converges in the simulator and
reports ALIGNED after 3 in-tolerance observations; every abort condition; display-only never
calls the legs; A/X/Y exclusion; calibration save/load/compatibility; Python 3.9 grammar.

Hardware, supervised, floor only, in this order: (1) display-only — move the box by hand and
check the error signs; (2) `--align-motion` calibration alone, watching the 360° turn;
(3) aligning from slightly off (rotated, too far, too close), then Y.

## Out of scope

Box orientation (yaw) and arcing around a rotated box; aligning without a recording; automatic
replay after ALIGNED; obstacle/edge detection; learned (neural) alignment.
