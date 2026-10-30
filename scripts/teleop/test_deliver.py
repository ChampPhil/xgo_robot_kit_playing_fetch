"""Delivery after fetch: the marker view and the lower+place motion from a stacking workcell."""

import contextlib
import io
import json
import sys
import tempfile
import unittest
from pathlib import Path

import cv2  # type: ignore[import-not-found]
import numpy as np  # type: ignore[import-not-found]

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "stacking"))
import deliver  # noqa: E402
import stack_boxes  # noqa: E402
from simulation import scene  # noqa: E402

PICKUP = {"version": 1, "duration": 30.0, "start": {"arm": None, "claw": None, "kneel": 1.0},
          "events": [{"t": 5.0, "arm": [80, 30]}, {"t": 8.0, "arm": [110, 0]},
                     {"t": 11.0, "arm": [137, -30]},
                     {"t": 14.0, "claw": 128}, {"t": 15.0, "claw": 255},
                     {"t": 20.0, "arm": [100, 40]}, {"t": 25.0, "arm": [0, 80]}]}


class DeliveryTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        root = Path(temp.name)
        session = root / "20261015-220825"
        session.mkdir()
        (session / "motion.json").write_text(json.dumps(PICKUP))
        self.config = root / "cell" / "workcell.json"
        image = root / "marker.png"
        cv2.imwrite(str(image), scene())
        with contextlib.redirect_stdout(io.StringIO()):
            assert stack_boxes.main(["--generate-placements", str(session),
                                     "--config", str(self.config)]) == 0
            assert stack_boxes.main(["--capture-approach", "--source", str(image),
                                     "--config", str(self.config)]) == 0

    def test_motion_lowers_then_places_then_retreats(self):
        motion = deliver.load(self.config)["motion"]
        self.assertEqual(motion["start"]["arm"], [0, 80])  # no jump from the carry pose
        self.assertIsNone(motion["start"]["claw"])  # keep holding
        times = [event["t"] for event in motion["events"]]
        self.assertEqual(times, sorted(times))
        self.assertLessEqual(times[-1], motion["duration"])
        claws = [event["claw"] for event in motion["events"] if "claw" in event]
        self.assertEqual(claws, [0])  # one release
        arms = [event["arm"] for event in motion["events"] if "arm" in event]
        self.assertIn([137, -30], arms)  # the pickup's grab point, layer 1
        self.assertEqual(arms[-1], [80, 30])

    def test_marker_reference_keeps_the_arm_where_it_is(self):
        reference = deliver.load(self.config)["reference"]
        self.assertIsNone(reference["state"]["arm"])  # never move the held block to align
        self.assertEqual(reference["state"]["kneel"], 1.0)
        self.assertTrue({"u", "v", "h"} <= set(reference["features"]))

    def test_marker_measure(self):
        measure = deliver.marker_measure(0)
        features, reason = measure(scene())
        self.assertIsNone(reason)
        self.assertTrue({"u", "v", "h"} <= set(features))
        features, reason = measure(np.full((480, 640, 3), 128, np.uint8))
        self.assertIsNone(features)
        self.assertIn("marker", reason)

    def test_a_workcell_without_placements_is_refused(self):
        self.config.write_text(json.dumps({"version": 1, "layers": [{}]}))
        with self.assertRaisesRegex(ValueError, "generate"):
            deliver.load(self.config)


if __name__ == "__main__":
    unittest.main()
