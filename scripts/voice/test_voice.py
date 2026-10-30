"""Offline checks for voice phrase parsing and command execution."""

import unittest
from unittest.mock import patch

import voice


class FakeDog:
    """Records xgolib calls without a serial port."""

    def __init__(self) -> None:
        self.calls: list[tuple] = []

    def stop(self) -> None:
        self.calls.append(("stop",))

    def move_x(self, value: float) -> None:
        self.calls.append(("move_x", value))

    def move_y(self, value: float) -> None:
        self.calls.append(("move_y", value))

    def turn(self, value: float) -> None:
        self.calls.append(("turn", value))

    def action(self, action_id: int) -> None:
        self.calls.append(("action", action_id))


class ParseCommandTests(unittest.TestCase):
    def test_turn_left_beats_left(self) -> None:
        self.assertEqual(voice.parse_command("please turn left now"), "turn_left")

    def test_go_forward(self) -> None:
        self.assertEqual(voice.parse_command("Go forward!"), "go_forward")

    def test_sit(self) -> None:
        self.assertEqual(voice.parse_command("sit down please"), "sit")

    def test_unknown_returns_none(self) -> None:
        self.assertIsNone(voice.parse_command("make me a sandwich"))


class ExecuteCommandTests(unittest.TestCase):
    def setUp(self) -> None:
        self.dog = FakeDog()

    def test_stop(self) -> None:
        voice.execute_command(self.dog, "stop")
        self.assertEqual(self.dog.calls, [("stop",)])

    def test_timed_forward_stops(self) -> None:
        with patch.object(voice.time, "sleep"):
            voice.execute_command(self.dog, "go_forward")
        self.assertEqual(self.dog.calls, [("move_x", 10.0), ("stop",)])

    def test_gesture_action(self) -> None:
        with patch.object(voice.time, "sleep"):
            voice.execute_command(self.dog, "sit")
        self.assertEqual(self.dog.calls, [("action", 12)])


class WakeScoreTests(unittest.TestCase):
    def test_picks_jarvis_key(self) -> None:
        self.assertEqual(voice._wake_score({"hey_jarvis_v0.1": 0.81, "other": 0.99}), 0.81)


class ExplainTests(unittest.TestCase):
    def test_chase_message(self) -> None:
        with patch("builtins.print") as printed:
            voice.explain_no_command("chase the person")
        self.assertIn("Chase is not wired", printed.call_args[0][0])

    def test_walk_meters_message(self) -> None:
        with patch("builtins.print") as printed:
            voice.explain_no_command("walk 2 meters")
        self.assertIn("Walk N meters", printed.call_args[0][0])


if __name__ == "__main__":
    unittest.main()
