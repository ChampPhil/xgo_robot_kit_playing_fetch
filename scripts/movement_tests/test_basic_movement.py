"""Offline checks for the movement CLI; no robot or serial port required."""

import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import basic_movement


class FakeDog:
    def __init__(self):
        self.calls = []

    def move_x(self, value):
        self.calls.append(("x", value))

    def move_y(self, value):
        self.calls.append(("y", value))

    def turn(self, value):
        self.calls.append(("turn", value))

    def stop(self):
        self.calls.append(("stop",))


class MovementTests(unittest.TestCase):
    def setUp(self):
        self.parser = basic_movement.build_parser()
        self.dog = FakeDog()

    def test_all_movements_and_stop_between_commands(self):
        args = self.parser.parse_args(["--movement", "all", "--pause", "0"])
        with patch.object(basic_movement.time, "sleep") as sleep:
            basic_movement.run_test(self.dog, args)
        self.assertEqual(self.dog.calls, [
            ("x", 10), ("stop",), ("x", -10), ("stop",),
            ("y", 8), ("stop",), ("y", -8), ("stop",),
            ("turn", 30), ("stop",), ("turn", -30), ("stop",), ("stop",),
        ])
        self.assertEqual(sleep.call_count, 11)  # six moves, five pauses

    def test_selected_movements_and_custom_values(self):
        args = self.parser.parse_args([
            "--movement", "turn-right", "--movement", "forward",
            "--turn-speed", "15", "--x-step", "5", "--duration", "0.5",
        ])
        with patch.object(basic_movement.time, "sleep"):
            basic_movement.run_test(self.dog, args)
        self.assertEqual(self.dog.calls, [
            ("turn", -15.0), ("stop",), ("x", 5.0), ("stop",), ("stop",),
        ])

    def test_interrupt_stops_robot(self):
        args = self.parser.parse_args(["--movement", "forward"])
        with patch.object(basic_movement.time, "sleep", side_effect=KeyboardInterrupt), self.assertRaises(KeyboardInterrupt):
            basic_movement.run_test(self.dog, args)
        self.assertEqual(self.dog.calls, [("x", 10), ("stop",), ("stop",)])

    def test_invalid_input_rejected_before_connecting(self):
        for arguments in (
            [], ["--movement", "diagonal"],
            ["--movement", "forward", "--x-step", "26"],
            ["--movement", "left", "--y-step", "0"],
            ["--movement", "turn-left", "--turn-speed", "nan"],
            ["--movement", "forward", "--duration", "6"],
        ):
            with self.subTest(arguments=arguments), patch.object(self.parser, "error", side_effect=ValueError), self.assertRaises(ValueError):
                self.parser.parse_args(arguments)

    def test_main_passes_port_and_skips_prompt_with_yes(self):
        factory = Mock(return_value=self.dog)
        with patch.object(basic_movement, "import_module", return_value=SimpleNamespace(XGO=factory)), \
                patch("builtins.input") as prompt, \
                patch.object(basic_movement.time, "sleep"):
            basic_movement.main(["--movement", "right", "--port", "/dev/test", "--yes"])
        factory.assert_called_once_with(port="/dev/test", version="xgolite")
        prompt.assert_not_called()
        self.assertEqual(self.dog.calls, [("y", -8), ("stop",), ("stop",)])

    def test_all_cannot_be_combined_with_other_movements(self):
        with patch.object(basic_movement, "import_module") as import_module, self.assertRaises(SystemExit) as error:
            basic_movement.main(["--movement", "all", "--movement", "forward", "--yes"])
        self.assertEqual(error.exception.code, 2)
        import_module.assert_not_called()


if __name__ == "__main__":
    unittest.main()
