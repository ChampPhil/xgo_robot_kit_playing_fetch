"""Synthetic, offline checks for the color detector (no camera or robot required)."""

import contextlib
import io
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import cv2  # type: ignore[import-not-found]
import detect_boxes
import numpy as np  # type: ignore[import-not-found]

ORDER = ["purple", "orange", "light-blue"]


def scene(colors=ORDER, duplicate_purple=False):
    hsv = np.zeros((240, 320, 3), dtype=np.uint8)
    hsv[:] = (0, 0, 35)
    hues = {"purple": 140, "orange": 15, "light-blue": 100}
    for index, color in enumerate(colors):
        x = 20 + 95 * index
        cv2.rectangle(hsv, (x, 80), (x + 40, 120), (hues[color], 200, 210), -1)
    if duplicate_purple:
        cv2.rectangle(hsv, (25, 155), (65, 195), (140, 200, 210), -1)
    cv2.rectangle(hsv, (300, 15), (302, 17), (140, 200, 210), -1)  # noise
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


class BoxDetectionTests(unittest.TestCase):
    def test_three_boxes_in_requested_order(self):
        detections = detect_boxes.detect_boxes(scene())
        result = detect_boxes.evaluate(detections, ["light-blue", "purple", "orange"])
        self.assertEqual(result["status"], "ready")
        self.assertEqual([item["color"] for item in result["observed_order"]],
                         ["light-blue", "purple", "orange"])
        self.assertTrue(all(len(detections[color]) == 1 for color in ORDER))
        self.assertTrue(all(item["area_px"] >= 150 for item in result["observed_order"]))

    def test_a_colour_can_combine_several_ranges(self):
        hsv = np.full((240, 320, 3), (0, 0, 35), dtype=np.uint8)
        cv2.rectangle(hsv, (20, 80), (60, 120), (140, 200, 210), -1)  # vivid
        cv2.rectangle(hsv, (200, 80), (240, 120), (140, 50, 40), -1)  # pale but dark
        frame = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        vivid = ((130, 100, 25), (150, 255, 255))
        pale_dark = ((130, 40, 25), (150, 255, 90))
        found = detect_boxes.detect_boxes(frame, ranges={"purple": [vivid, pale_dark]})
        self.assertEqual(sorted(box["bbox"][0] for box in found["purple"]), [20, 200])
        self.assertEqual(len(detect_boxes.detect_boxes(frame, ranges={"purple": vivid})["purple"]),
                         1)

    def test_missing_or_duplicate_box_never_produces_an_order(self):
        missing = detect_boxes.evaluate(detect_boxes.detect_boxes(scene(ORDER[:2])), ORDER)
        self.assertEqual(missing["status"], "missing")
        self.assertEqual(missing["missing"], ["light-blue"])
        self.assertEqual(missing["observed_order"], [])
        ambiguous = detect_boxes.evaluate(detect_boxes.detect_boxes(scene(duplicate_purple=True)), ORDER)
        self.assertEqual(ambiguous["status"], "ambiguous")
        self.assertEqual(ambiguous["ambiguous"], ["purple"])
        self.assertEqual(ambiguous["observed_order"], [])

    def test_image_cli_and_annotation(self):
        with tempfile.TemporaryDirectory() as folder:
            image = Path(folder) / "boxes.png"
            annotated = Path(folder) / "annotated.png"
            self.assertTrue(cv2.imwrite(str(image), scene()))
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                code = detect_boxes.main([
                    "--order", *ORDER, "--source", str(image), "--output", str(annotated),
                ])
            result = json.loads(output.getvalue())
            self.assertEqual(code, 0)
            self.assertEqual(result["status"], "ready")
            self.assertEqual(result["stable_frames"], 1)
            self.assertIsNotNone(cv2.imread(str(annotated)))

    def test_camera_requires_consecutive_ready_frames(self):
        camera = Mock()
        camera.isOpened.return_value = True
        camera.read.side_effect = [(True, scene()), (True, scene())]
        output = io.StringIO()
        with patch.object(detect_boxes.cv2, "VideoCapture", return_value=camera), contextlib.redirect_stdout(output):
            code = detect_boxes.main(["--order", *ORDER, "--frames", "2"])
        self.assertEqual(code, 2)
        self.assertEqual(json.loads(output.getvalue())["status"], "unstable")
        self.assertEqual(json.loads(output.getvalue())["observed_order"], [])
        camera.release.assert_called_once()

    def test_single_color_flag_selects_target_without_other_boxes(self):
        with tempfile.TemporaryDirectory() as folder:
            image = Path(folder) / "orange.png"
            self.assertTrue(cv2.imwrite(str(image), scene(["orange"])))
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                code = detect_boxes.main(["--color", "orange", "--source", str(image)])
            report = json.loads(output.getvalue())
            self.assertEqual(code, 0)
            self.assertEqual(report["status"], "ready")
            self.assertEqual(report["target"]["color"], "orange")
            self.assertIn("no arm", report["note"])

    def test_duplicate_color_is_ambiguous_in_single_color_mode(self):
        with tempfile.TemporaryDirectory() as folder:
            image = Path(folder) / "duplicate.png"
            self.assertTrue(cv2.imwrite(str(image), scene(duplicate_purple=True)))
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                code = detect_boxes.main(["--color", "purple", "--source", str(image)])
            report = json.loads(output.getvalue())
            self.assertEqual(code, 2)
            self.assertEqual(report["status"], "ambiguous")
            self.assertIsNone(report["target"])

    def test_custom_ranges_and_min_area(self):
        hsv = np.zeros((240, 320, 3), dtype=np.uint8)
        cv2.rectangle(hsv, (100, 100), (110, 110), (165, 50, 40), -1)  # dark, desaturated, far
        frame = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        self.assertEqual(detect_boxes.detect_boxes(frame)["purple"], [])
        wide = {"purple": ((115, 40, 25), (178, 255, 255))}
        found = detect_boxes.detect_boxes(frame, min_area=40, ranges=wide)
        self.assertEqual(len(found["purple"]), 1)
        self.assertEqual(set(found), {"purple"})

    def test_duplicate_order_is_rejected(self):
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
            detect_boxes.main(["--order", "purple", "purple", "orange", "--source", "missing.png"])
        self.assertEqual(error.exception.code, 2)


if __name__ == "__main__":
    unittest.main()
