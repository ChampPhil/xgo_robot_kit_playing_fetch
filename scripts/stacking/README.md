# Three-box tower workflow

User order is **bottom to top**. The three colors are purple, orange and light-blue,
each exactly once. New code was **deployed on 2026-10-28; 308 offline tests pass on Mac/Pi**,
but no live stacking or verification calibration has run. The successful single-box
pickup is reused through the existing gyro aligner / teach replayer; transport, placement,
verification poses and layer recordings still need teaching on the actual workcell.

## Try it without a robot

From the repository root:

```bash
.venv/bin/python scripts/stacking/stack_boxes.py --simulate \
  --order orange light-blue purple --fail-first orange --drop-first light-blue
.venv/bin/python scripts/stacking/stack_boxes.py --simulate \
  --order purple orange light-blue --uncertain purple
.venv/bin/python scripts/stacking/stack_boxes.py --observe \
  --source scripts/stacking/assets/three-boxes.png --output /tmp/boxes-marked.png \
  --order light-blue purple orange
```

Simulation uses the actual color and ArUco detectors on synthetic images. It prints JSON
phase/evidence events and labels every result as simulation (`physical_success: false`).
The uncertain-grip example exits 2 before any simulated placement. `--attempts 3` includes
the first attempt (two retries); allowed range is 1–5. A repeated failure exhausts this limit.

`--observe --source 0 --frames 60` reads a real camera **without initializing XGO or UART**.
Stop any competing camera owner first. A stream URL or video file also works. All color
candidates are reported; unique colors get track IDs only while position/size remain
consistent across fresh frames. Duplicate candidates fail ambiguous; disappearance or a
jump resets continuity. A single image is labeled as such and cannot establish live stability.
This is HSV rectangle detection, not semantic box recognition or a calibrated floor plane.
The multi-box mode deliberately uses the existing disjoint color ranges; the wider
single-purple alignment range overlaps light-blue and is unsuitable for selecting three colors.

## Workflow

1. Survey: find all three colors with stable, unique detections in the taught survey view.
2. Select the next unplaced color, align with **that color's** pickup reference, replay pickup.
3. Move to a taught, stationary check pose with disjoint **held** and **floor** image regions.
   One box in held and none on floor is a positive visual grip observation. A floor box and
   no held box is a confirmed miss and may retry. Neither, both, duplicates, stale frames or
   occlusion are unknown: stop. Disappearance alone never authorizes carrying or placement.
4. Carry along a taught route with the claw command preserved. Stop and check the grip again,
   align to the printed stack marker, check its corner geometry, then recheck the grip.
   A confirmed dropped floor box uses a separately taught recovery route and consumes an attempt.
5. Replay the layer's stationary place motion. It must contain exactly one explicit claw-open
   event; no walking or turning is allowed in a placement motion.
6. Inspect from a taught view. Require the correct marker/corners, an empty held region and
   unique, vertically adjacent colored faces in the requested bottom-to-top order. Only then
   commit the layer and return to survey for the next color. Uncertain placement stops the job;
   the script never piles a new box onto an unverified layer.

No full reset, automatic claw-open, or automatic stand command is issued on cancellation.
The legs stop and the current arm/claw setpoints remain. This avoids dropping a box on exit.

## Printed target

Print [assets/stack-marker-0.png](assets/stack-marker-0.png), including its white margin,
and fix it beside the tower so it stays visible for all three layers. It is **DICT_4X4_50,
ID 0**, generated with [OpenCV's ArUco API](https://docs.opencv.org/4.10.0/de/d67/group__objdetect__aruco.html).
Do not cover it with the first block or move it between teaching and execution.
The detector includes a tested OpenCV 4.7 compatibility setting for the Pi: retain candidates
that its default grouping misses, then merge nested candidates with matching corners from
the same marker. Distinct prints of the same ID remain ambiguous; error correction is unchanged.

Generate another ID with:

```bash
.venv/bin/python scripts/stacking/stack_boxes.py --make-marker /tmp/marker-3.png --marker-id 3
```

Marker matching uses normalized image corners, position and size to return to a taught view.
It does not infer metric arm coordinates, a collision-free route or a camera-to-arm transform.
Marker pixel size on paper need not be hand-measured for this image-matching workflow; keep
the same print and placement for teaching and use. A missing marker stops the approach.

## Teach the workcell

Copy `stack_config.example.json` to a new workcell configuration. Paths are relative to
that configuration file. The example is deliberately disabled, with no invented arm poses
or guessed held/floor/stack ROIs. `validated` means the corresponding actual motion/view
has been supervised and verified; changing a Boolean does not perform calibration.

Use the existing keyboard/gamepad to teach explicit, named motions with X:

```bash
.venv/bin/python scripts/teleop/gamepad_sender.py --keyboard --video \
  --record-motion-only --teach-dir '~/xgo_teach/stacking/taught/routes'
```

`--record-motion-only` allows X even without a box, saves commands and one reference image,
and skips the video dataset. Set the arm explicitly with its controls before X; unknown arm
starts are refused. This flag never creates an automatic F/Space alignment reference.
Normal recording/fetch behavior is unchanged without it. Keep route sessions separate from
ordinary pickup sessions. Copy the chosen session's `motion.json` to its named profile path;
preserve original recordings. Release walking keys before stopping X so the route contains
its explicit leg-stop event. No recording is reversed automatically.

Teach these assets (paths are listed in the example):

- A survey motion ending at the camera pose used to see the remaining floor boxes.
- A pickup reference + pickup motion for each color, and an empty-gripper retry/retreat motion.
  Pickup starts explicitly open and ends closed. References must share the pickup start pose.
- A stationary carry/check motion that exposes the held box and floor in disjoint ROIs.
- For **each layer**: a carry-to-stack route; a marker approach reference; a stationary
  place/release motion; a stationary inspection motion and marker reference; a return route;
  and an empty-gripper recovery route after a transport drop. Assets may share a validated
  route where physically appropriate; do not assume a blind reverse walk retraces the path.

The known block height is **25 mm**: placement support surfaces are nominally 0, 25 and 50 mm.
Each layer needs its own physically taught arm trajectory. Adding 25 to arm `z` is not a
validated vertical shift when the body pitches; the code checks reach limits instead of
silently clamping an unreachable placement. Routes preserve the current claw setpoint even
if a recording's start includes an old claw value. Claw events in carry/check/return/recovery
routes are rejected. All event times/values and route limits are checked before motor startup.

Create a camera-only reference while the teaching controller holds the desired pose:

```bash
.venv/bin/python scripts/stacking/stack_boxes.py --capture-reference /tmp/stack-approach \
  --source http://127.0.0.1:8090/stream --target marker --pose-file /path/to/pose.json
```

`pose.json` is `{"arm": [x,z], "kneel": level}` with the **already commanded** values,
or an existing recording `reference.json` whose `state` contains them. It sends no pose
commands. Use the controller's SSH video tunnel so both programs share the same camera;
do not open camera 0 twice. Capture each pickup with `--target purple` etc. and each marker
approach/inspection view with `--target marker`. Keep the sharp reference image next to its JSON.

Fill normalized ROIs `[left,top,right,bottom]` from images of the actual taught views:
survey floor and excluded stack area; disjoint check-pose held/floor regions; and per-layer
inspection stack/held regions. This uses camera observation, not hand-measured distances.
The marker-approach pose, place-start pose and carry-check final pose must match. The
inspection final pose must match its inspection marker reference. The loader enforces these
relationships so a check cannot unknowingly change the camera pose after alignment.

An optional `verification.grip_raw` can add **independently calibrated**, disjoint raw
bands `{"empty": [low,high], "occupied": [low,high]}`. These are servo readback bytes,
not force. No ranges are supplied because they have not been measured on this robot.
When enabled, readbacks must agree with visual evidence; missing/contradictory data stops.

## Later live use (not run during this implementation)

While teaching, list everything still missing, invalid or unreviewed in one pass (read-only,
no camera or motors; exit 0 only when the workcell is complete):

```bash
python3 scripts/stacking/stack_boxes.py --setup-report \
  --config ~/xgo_teach/stacking/workcell.json
```

Each line is `missing` (file or value not set), `invalid` (with the reason) or `unreviewed`
(`validated` still false). Cross-checks such as matching poses or disjoint ROIs are reported
once both sides load. The last stdout line is the full report as JSON.

Validate all files on the Pi first:

```bash
python3 scripts/stacking/stack_boxes.py --check-config \
  --config ~/xgo_teach/stacking/workcell.json --order purple orange light-blue
```

After deploying and teaching/validating the actual workcell, launch **from the Mac**:

```bash
.venv/bin/python scripts/stacking/stack_boxes.py --execute \
  --config '~/xgo_teach/stacking/workcell.json' --order light-blue purple orange
```

The Mac first performs remote configuration validation, then prints the concrete motion and
floor/unplugged/empty-gripper checklist. It sends 20 Hz heartbeats through SSH. The Pi owns
one UART and one background camera; there is no competing telemetry process. Camera failure,
heartbeat loss/EOF, low/invalid battery, control-loop stalls, Ctrl+C/SIGHUP/SIGTERM, per-phase
timeouts and the 600 s session limit stop the run. Battery must be valid and >=20% before
XGO startup and during the run. Initial XGO construction resets/stands the robot. Boot menu
and camera owners must be released as in AGENTS.md. No files/recordings are deleted.

Phase/evidence JSONL is written under `~/xgo_teach/stacking/`; this workflow does not record
continuous video. Logs distinguish detection, grip verification, committed placement and job
completion. Stops do not attempt to restore the boot menu or move a held box. Software stops
do not cover SIGKILL, process deadlock or a motion MCU that ignores commands; stay supervised.

Taught routes support a bounded workcell with a visible marker, not navigation anywhere on
the floor. Color rectangles and a 2-D tower view can be confused by lighting, occlusion or
background objects; the inspection view must actually expose the colored faces. Validation
and a printed target are still required before claiming a physical tower was built.

## Code and tests

| File | Job |
| --- | --- |
| `stack_boxes.py` | CLI, camera-only inspection/capture, Mac heartbeat launcher |
| `vision.py` | Three-color detection/tracks, marker, grip/tower evidence |
| `workflow.py` | Color order, bounded retries, layer commits, stop handling |
| `profiles.py` | Validate every taught file, pose, layer and claw rule before startup |
| `robot.py` | Single camera/UART owner; reuse teleop aligner and replayer |
| `simulation.py` | Actual detectors on synthetic floor, held-box and tower images |

```bash
.venv/bin/python -m unittest discover -s scripts/stacking -p 'test_*.py'
```
