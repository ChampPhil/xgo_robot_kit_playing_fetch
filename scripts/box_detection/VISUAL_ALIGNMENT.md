# Repositioning for pickup without measuring each distance

## Finding

**Yes, image-based alignment is a plausible next step, but the current script does not move or pick up boxes.** We can avoid measuring the robot-to-box distance on each attempt by teaching a known-good approach image once, then moving until the current box looks like that reference. This is a simplified visual-servoing strategy, not a claim that one bounding box determines a unique 3D grasp pose.

The camera is mounted on the robot's head/body, not its wrist. Body movement changes the image, while moving the arm alone may not. Leg stance, body pitch/roll, camera crop and resolution must match the taught setup. An arbitrary camera image cannot by itself establish a safe gripper pose.

## Recommended first experiment: teach and approach

1. On a clear **floor**, manually place one 25 mm-high box in a position where a supervised, stationary arm pickup has already worked. Do not force powered joints by hand. Save the commanded arm approach/grasp/lift positions and claw setting from that successful test.
2. With the same robot stance, camera resolution, and box orientation, save a reference image before the arm occludes the box. Record normalized box center `(u/W, v/H)`, width/height `(w/W, h/H)`, and contour/face orientation if reliable. This teaches the desired visual appearance rather than a measured distance.
3. First run a **no-motion overlay** comparing the live detection to the reference. The gripper's desired location need not be the center of the camera image. Normalize against the actual frame dimensions; do not hard-code 320/240 for a 640×480 frame.
4. After validating movement direction empirically, use small bounded **move → stop → settle → observe** increments to reduce lateral image error. Then approach in short increments while monitoring apparent box size and vertical image position. Translation/turning are coupled; recheck all features after every increment. XGO stride commands are not metric distance commands.
5. Only after all reference features agree over multiple fresh frames, stop the base and consider replaying the previously verified grasp. Stop/hold on lost, clipped, merged, or duplicate detections. Stop if image error grows or no progress occurs. Image agreement alone must not be reported as a confirmed successful pickup.
6. Verify pickup separately: the object should move with the gripper/lift, rather than simply disappear behind it. There is no verified force/contact signal in our current sensor logger, so a failed grasp must not trigger an unattended repeated squeeze.

This replaces measuring each approach distance with **one-time teaching and camera/control validation**. The approach is local and conditional, not guaranteed for arbitrary box positions or orientations. Apparent size changes with pose, lighting masks, and partial occlusion as well as depth. Similar image features can correspond to physically different grasp conditions.

## Changes needed in the current identifier

`detect_boxes.py` currently outputs a color blob's axis-aligned rectangle, center and area, then exits once detections are present in several frames. Before using it in a controller:

- Keep acquiring fresh frames in a separate perception loop; timestamp observations and reject stale frames. Presence in consecutive frames is not identity tracking: add position/scale continuity and reject jumps between objects.
- Add normalized features and a persisted reference image/config. Reset that reference when camera pose/resolution, box geometry, robot stance or arm settings change.
- Distinguish `detected`, `aligned`, and `grasp_verified`. Today's `ready` only means a unique detection; it is **not** safe-to-grasp.
- Reject boxes touching the image border, ambiguous colors, merged regions, and strong tilt/occlusion. The existing HSV filter cannot detect table edges or obstacles.
- Design one serial owner for movement and telemetry. The standalone logger deliberately excludes other UART clients; do not run it alongside a controller using a second serial connection.
- Implement time/iteration limits, an explicit motion-enable flag, battery/attitude gates, emergency stop, and a stop mechanism independent of a stalled camera/inference call. `finally: stop()` alone cannot handle a hung process, SIGKILL or lost serial connection.

**Do not test automatic walking on the tabletop seen in the last camera capture.** The camera does not reliably detect edges; tabletop visibility does not imply traversable space. Start on a clear, level floor with a nearby human stop control.

## Alternative: estimate pose from known geometry

If visual teaching is not robust enough, use camera intrinsics/distortion calibration plus known-size features (a square ArUco tag or reliably ordered box corners) and OpenCV `solvePnP`. That can estimate camera-relative pose without measuring each robot-to-box distance, but still requires a camera-to-robot/arm transform. Box height alone (25 mm) and an axis-aligned rectangle are insufficient to supply reliable corner correspondences, face orientation and all dimensions. A tag is often easier to localize than plain uniformly colored faces.

A fixed ground-plane homography is another option, but it maps only points on that plane. Applying it to the center/top of a 25 mm-high box produces parallax errors; camera pose also changes when the dog walks or tilts. Do not apply a floor-plane mapping unchanged to subsequent stack levels.

## Validation before enabling motion

- Match targets offline on independent camera frames (including empty scene, occlusion, colored distractors and duplicate boxes).
- Prove reference matching rejects wrong scale, lateral offset, clipping, stale frames, and camera changes.
- Supervise one small movement at a time on the floor, recording fresh before/after images and IMU attitude. Establish control direction and whether error decreases.
- Demonstrate repeated stationary grasps before connecting approach to pickup. Measure grasp success independently from detection success.

## References

- [ViSP four-point IBVS tutorial](https://github.com/lagadic/visp/blob/master/tutorial/visual-servo/ibvs/tutorial-ibvs-4pts-wireframe-robot-viper.cpp): uses desired/current image features, an interaction matrix and robot velocity control. It is a simulated arm example, **not** a ready-made XGO controller.
- [Visual servoing geometry tutorial](https://sir.upc.edu/projects/ris_tutorials/advanced/visual_servoing/visual_servoing.html): camera projection, intrinsic parameters, feature coordinates and depth dependence.
- [OpenCV PnP documentation](https://docs.opencv.org/4.11.0/d5/d1f/calib3d_solvePnP.html): pose from calibrated 2D–3D correspondences and known object geometry.

No automatic base/arm motion was implemented or run during this investigation. The robot was unreachable over SSH during the latest logger upgrade; earlier purple detection was confirmed live, but a grasp-ready reference has not been taught.
