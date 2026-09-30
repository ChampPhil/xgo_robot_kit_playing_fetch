# Colored-box camera test (no robot motion)

`detect_boxes.py` is the first, **perception-only** step toward sorting and stacking the purple, orange, and light-blue boxes. It uses OpenCV HSV color masks, morphology, and rectangular-contour filtering; it does **not** use YOLO, the serial port, the arm, or the gripper. Use `--order` to report all three boxes in bottom-to-top order, or `--color` to locate just one box. Pixels are **not** calibrated grasp coordinates; selecting a color does **not** pick it up.

## Setup and examples

From the repository root, run `./setup.sh` to create `.venv/` and install the dependencies. Run the detector on a laptop photo first (no robot needed):

```bash
.venv/bin/python scripts/box_detection/detect_boxes.py \
  --order purple orange light-blue --source /path/to/three_boxes.jpg \
  --output annotated.png
```

On the **charged** robot, place three separate, clearly visible boxes on a neutral background in good light. For a camera exposed as OpenCV device 0:

```bash
.venv/bin/python scripts/box_detection/detect_boxes.py \
  --order orange purple light-blue --source 0 --frames 90 \
  --output camera_result.png
```

To select one target color (even if the other boxes are not in view):

```bash
.venv/bin/python scripts/box_detection/detect_boxes.py \
  --color purple --source 0 --output purple_target.png
```

You can also use `--source /path/to/video.mp4` to analyze recorded footage. Camera/video requires three **consecutive** unambiguous frames by default; a photo needs only one frame. If the camera cannot open, check whether another process owns it. This script uses OpenCV `VideoCapture`; some Raspberry Pi camera/OS configurations instead require Picamera2. Camera 0 has been confirmed to return 640×480 frames on this robot. Purple was tuned against one robot-camera frame and detected successfully; orange and light-blue thresholds have **not** yet been checked against the actual boxes.

## Flags and output

| Flag | Meaning |
| --- | --- |
| `--order C1 C2 C3` | Select all three colors exactly once, **bottom to top**. Reporting only. Cannot combine with `--color`. |
| `--color NAME` | Select one purple, orange, or light-blue target. Reporting only. Cannot combine with `--order`. |
| `--source 0` | Camera index (default 0), image path, or video path. |
| `--frames N` | Maximum frames read from a camera/video (default 60). |
| `--stable-frames N` | Consecutive valid frames needed for camera/video success (default 3). |
| `--min-area N` | Reject colored regions smaller than N pixels (default 150). |
| `--output PATH` | Save the final frame with labeled rectangles; no graphical desktop required. |
| `--help` | Show CLI usage. |

The script prints JSON. `--order` reports `status`, `detections`, `missing`, `ambiguous`, and `observed_order`; `--color` reports `status`, `detections`, and a single `target` with pixel bounding box/center/area. Exit code **0** means a stable complete observation. Exit code **2** means missing, ambiguous, unstable, or an input/camera error. On failure `observed_order` is empty or `target` is null: **never** use a missing or uncertain detection as a pickup instruction. Saving an annotated image does not mean detection succeeded; check the status/exit code.

## Tuning and limits

The approximate initial HSV bounds are in `HSV_RANGES` at the top of `detect_boxes.py`: purple H 115–150 (S ≥75, V ≥35), orange H 5–25, light-blue H 85–110. OpenCV hue uses **0–179**, saturation/value **0–255**. Adjust the bounds using images from the *actual* camera and lighting. `--min-area` controls small-noise rejection; the detector also rejects very thin/wide or poorly filled blobs. It can confuse colored background objects with boxes and cannot separate touching or occluded boxes reliably. Use a plain mat, space the boxes apart, and inspect `--output` before trusting an observation. The method is a baseline to benchmark against a custom YOLO box detector later; the built-in `yoloFast()` model does not label these colors or a generic box class.

**No physical pickup or stacking is implemented.** In particular, boxes may start anywhere in view, but the arm only accepts physical X/Z coordinates; there is no camera-to-arm calibration or verified way to align the robot laterally/depth-wise yet. Pickup/placement requires a charged robot, exact box size/weight (about 25 mm high per the initial setup), a measured reachable workspace, camera-to-arm/ground calibration, safe navigation or manual alignment, and separate guarded motion tests. Never run movement tests while charging the robot.

For a way to avoid measuring the distance on every attempt, see [visual alignment research and proposed tests](VISUAL_ALIGNMENT.md). A taught, known-good pickup image can serve as an approach reference, but this is not yet an autonomous controller. Do not test automatic walking on a tabletop.

Offline synthetic tests:

```bash
.venv/bin/python -m unittest discover -s scripts/box_detection -p 'test_*.py'
```

References: [OpenCV HSV/inRange](https://docs.opencv.org/4.x/da/d97/tutorial_threshold_inRange.html), [XGO Python motion/arm API](https://wiki.elecfreaks.com/en/pico/cm4-xgo-robot-kit/advanced-development/python-development/).
