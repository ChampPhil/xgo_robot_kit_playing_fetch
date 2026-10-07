"""Offline checks for self-calibrating alignment; uses a simulated robot, no hardware."""

import json
import math
import tempfile
import unittest
from pathlib import Path

import align

REF = {"u": 0.5, "v": 0.6, "h": 0.2}


def feat(u=0.5, v=0.6, h=0.2):
    return {"u": u, "v": v, "h": h}


class MathTests(unittest.TestCase):
    def test_errors_relative_to_reference(self):
        eu, eh, ev = align.errors(feat(u=0.56, v=0.58, h=0.18), REF)
        self.assertAlmostEqual(eu, 0.06)
        self.assertAlmostEqual(eh, -0.1)
        self.assertAlmostEqual(ev, -0.02)

    def test_choose_step_picks_the_best_lever_and_clips(self):
        columns = {"turn": [-0.08, 0.0], "forward": [0.0, 0.06], "strafe": [0.03, 0.0]}
        # box right of target (eu>0): turning right (negative turn) fixes it best
        primitive, amount = align.choose_step((0.04, 0.0), columns)
        self.assertEqual(primitive, "turn")
        self.assertAlmostEqual(amount, 0.5)
        # box too small (far): walk forward, clipped to one full step
        primitive, amount = align.choose_step((0.0, -0.2), columns)
        self.assertEqual((primitive, amount), ("forward", 1.0))

    def test_unusable_columns_are_never_chosen(self):
        columns = {"turn": [-0.08, 0.0], "strafe": [0.001, 0.0], "forward": None}
        self.assertEqual(align.choose_step((0.0, -0.2), columns), (None, 0.0))
        self.assertEqual(align.choose_step((0.04, 0.0), columns)[0], "turn")

    def test_broyden_update_moves_the_column_toward_what_happened(self):
        column = align.broyden_update([-0.08, 0.0], observed=(-0.05, 0.01), amount=0.5)
        # per full step it actually did (-0.1, 0.02); move 30% of the way
        self.assertAlmostEqual(column[0], -0.08 + 0.3 * (-0.1 + 0.08))
        self.assertAlmostEqual(column[1], 0.3 * 0.02)
        self.assertEqual(align.broyden_update([-0.08, 0.0], (-0.01, 0.0), 0.1), [-0.08, 0.0])

    def test_broyden_learns_a_wrong_model(self):
        truth, model = [-0.1, 0.0], [-0.03, 0.02]
        for _ in range(15):
            model = align.broyden_update(model, (truth[0] * 0.5, truth[1] * 0.5), 0.5)
        self.assertAlmostEqual(model[0], -0.1, places=2)
        self.assertAlmostEqual(model[1], 0.0, places=2)

    def test_within_tolerance(self):
        tol = {"u": 0.04, "h": 0.08, "v": 0.05}
        self.assertTrue(align.within((0.03, -0.05, 0.01), tol))
        self.assertFalse(align.within((0.05, 0.0, 0.0), tol))
        self.assertFalse(align.within((0.0, 0.09, 0.0), tol))


class CalibrationFileTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = Path(self.tmp.name) / "calibration.json"
        self.setup = {"speeds": {"turn": 20, "forward": 8, "strafe": 6}, "step_seconds": 0.3,
                      "kneel": 0.0, "arm": [80, 30]}

    def tearDown(self):
        self.tmp.cleanup()

    def test_save_load_and_compatibility(self):
        calibration = dict(self.setup, deg_per_turn_step=12.0,
                           columns={"turn": [-0.07, 0.0], "forward": [0.0, 0.06], "strafe": None})
        align.save_calibration(self.path, calibration)
        loaded = align.load_calibration(self.path)
        self.assertEqual(loaded["columns"]["turn"], [-0.07, 0.0])
        self.assertTrue(align.compatible(loaded, self.setup))
        self.assertFalse(align.compatible(loaded, dict(self.setup, step_seconds=0.4)))
        self.assertFalse(align.compatible(loaded, dict(self.setup, kneel=0.5)))

    def test_missing_or_damaged_calibration_loads_as_none(self):
        self.assertIsNone(align.load_calibration(self.path))
        self.path.write_text('{"version": 1, "colu')
        self.assertIsNone(align.load_calibration(self.path))
        self.path.write_text(json.dumps({"version": 1, "columns": {"turn": ["x", 0]}}))
        self.assertIsNone(align.load_calibration(self.path))


if __name__ == "__main__":
    unittest.main()
