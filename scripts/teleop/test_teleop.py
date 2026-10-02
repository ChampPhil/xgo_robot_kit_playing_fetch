"""Offline checks for gamepad teleop; no gamepad, SSH, robot or serial port required."""

import io
import json
import queue
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import gamepad_sender as sender
import teleop_receiver as receiver


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


def line(x=0.0, y=0.0, yaw=0.0, enable=True):
    return json.dumps({"x": x, "y": y, "yaw": yaw, "enable": enable}) + "\n"


QUIET = object()  # a get() that times out


class Paced:
    """Commands arriving live: one per get(), never a backlog."""

    def __init__(self, *items):
        self.items = list(items)

    def get(self, timeout=None):
        item = self.items.pop(0)
        if item is QUIET:
            raise queue.Empty
        if isinstance(item, BaseException):
            raise item
        return item

    def get_nowait(self):
        raise queue.Empty


class SenderTests(unittest.TestCase):
    def test_stick_directions_map_to_robot_axes(self):
        # SDL: left stick up is -y, right is +x. Robot: +x forward, +y left, +yaw turn left.
        self.assertEqual(sender.make_command(0, -1, 0, True, 0.1)["x"], 1.0)
        self.assertEqual(sender.make_command(0, 1, 0, True, 0.1)["x"], -1.0)
        self.assertEqual(sender.make_command(-1, 0, 0, True, 0.1)["y"], 1.0)
        self.assertEqual(sender.make_command(1, 0, 0, True, 0.1)["y"], -1.0)
        self.assertEqual(sender.make_command(0, 0, 1, True, 0.1)["yaw"], -1.0)
        self.assertEqual(sender.make_command(0, 0, -1, True, 0.1)["yaw"], 1.0)

    def test_deadzone_zeroes_drift_and_rescales(self):
        self.assertEqual(sender.make_command(0.1, -0.12, 0.05, True, 0.15),
                         {"x": 0.0, "y": 0.0, "yaw": 0.0, "enable": True})
        self.assertAlmostEqual(sender.apply_deadzone(0.575, 0.15), 0.5)

    def test_deadman_requires_lb(self):
        arming = sender.Arming(deadman=True)
        command = sender.make_command(0, -1, 0, False, 0.1)
        self.assertFalse(arming.enable(command, lb_held=False))
        self.assertTrue(arming.enable(command, lb_held=True))

    def test_without_deadman_sticks_must_centre_first(self):
        arming = sender.Arming(deadman=False)
        pushed = sender.make_command(0, -1, 0, False, 0.1)
        centred = sender.make_command(0, 0, 0, False, 0.1)
        self.assertFalse(arming.enable(pushed, lb_held=False))
        self.assertTrue(arming.enable(centred, lb_held=False))
        self.assertTrue(arming.enable(pushed, lb_held=False))

    def test_remote_command_passes_speed_limits(self):
        args = sender.build_parser().parse_args(["--max-x", "10", "--max-turn", "30"])
        command = sender.remote_command(args)
        self.assertTrue(command.startswith("cd ~/project/xgo_robot_kit_playing_fetch && exec python3"))
        self.assertIn("--max-x 10 --max-turn 30", command)
        self.assertNotIn("--max-y", command)


class ReceiverTests(unittest.TestCase):
    def setUp(self):
        self.args = receiver.build_parser().parse_args([])  # max x12, y8, turn40
        self.dog = FakeDog()
        self.driver = receiver.Driver(self.dog, self.args)
        self.log = []

    def run_lines(self, *items, first=None):
        receiver.run(self.driver, Paced(*items), 0.5, self.log.append, first=first)

    def test_full_sticks_scale_to_limits_and_stop_on_eof(self):
        self.run_lines(line(x=1, y=-1, yaw=0.5), receiver.EOF)
        self.assertEqual(self.dog.calls, [("x", 12), ("y", -8), ("turn", 20), ("stop",)])

    def test_only_changed_axes_are_resent(self):
        self.run_lines(line(x=1), receiver.EOF, first=line(x=0.5))
        self.assertEqual(self.dog.calls[:2], [("x", 6), ("x", 12)])

    def test_centred_sticks_send_one_stop(self):
        self.run_lines(line(x=1), receiver.EOF)
        self.dog.calls.clear()
        driver_calls = []
        self.driver.dog = SimpleNamespace(stop=lambda: driver_calls.append("stop"))
        self.driver.current = {"x": 12, "y": 0, "yaw": 0}
        self.driver.apply({"x": 0, "y": 0, "yaw": 0})
        self.driver.apply({"x": 0, "y": 0, "yaw": 0})
        self.assertEqual(driver_calls, ["stop"])

    def test_released_deadman_stops(self):
        self.run_lines(line(x=1), line(x=1, enable=False), receiver.EOF, first=line(x=1))
        self.assertEqual(self.dog.calls[:2], [("x", 12), ("stop",)])

    def test_bad_input_stops(self):
        for bad in ("not json\n", "[1]\n", line(x=float("nan")), line(x="fast"), line(x=True)):
            self.dog.calls.clear()
            self.run_lines(bad, receiver.EOF, first=line(x=1))
            self.assertEqual(self.dog.calls[:2], [("x", 12), ("stop",)], bad)

    def test_out_of_range_is_clamped(self):
        self.run_lines(line(x=5, yaw=-9), receiver.EOF)
        self.assertEqual(self.dog.calls[:2], [("x", 12), ("turn", -40)])

    def test_watchdog_stops_when_commands_go_quiet(self):
        self.run_lines(line(x=1), QUIET, QUIET, receiver.EOF)
        self.assertEqual(self.dog.calls, [("x", 12), ("stop",), ("stop",)])
        self.assertEqual(self.log.count("No command for 0.5s; stopping."), 1)

    def test_stale_backlog_is_skipped_but_eof_is_not(self):
        lines = queue.Queue()
        for item in (line(x=0.5), line(x=1)):
            lines.put(item)
        self.assertEqual(receiver.latest(lines, line(x=0.25)), line(x=1))
        lines.put(line(x=1))
        lines.put(receiver.EOF)
        self.assertIs(receiver.latest(lines, line(x=0.25)), receiver.EOF)

    def test_interrupt_still_stops(self):
        with self.assertRaises(KeyboardInterrupt):
            self.run_lines(KeyboardInterrupt(), first=line(x=1))
        self.assertEqual(self.dog.calls, [("x", 12), ("stop",)])

    def test_speed_flags_are_bounded(self):
        parser = receiver.build_parser()
        with patch("sys.stderr", io.StringIO()):
            for argv in (["--max-x", "26"], ["--max-y", "0"], ["--max-turn", "nan"]):
                with self.assertRaises(SystemExit):
                    parser.parse_args(argv)


if __name__ == "__main__":
    unittest.main()
