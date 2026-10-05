"""Offline checks for gamepad teleop; no gamepad, SSH, robot or serial port required."""

import io
import json
import tempfile
import math
import queue
import subprocess
import sys
import unittest
from pathlib import Path
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

    def attitude(self, direction, value):
        self.calls.append(("attitude", direction, value))

    def translation(self, direction, value):
        self.calls.append(("translation", direction, value))

    def arm(self, x, z):
        self.calls.append(("arm", x, z))

    def claw(self, value):
        self.calls.append(("claw", value))


def line(x=0.0, y=0.0, yaw=0.0, enable=True, **extra):
    message = {"x": x, "y": y, "yaw": yaw, "enable": enable}
    message.update(extra)
    return json.dumps(message) + "\n"


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


    def test_stream_writes_through_ssh_pipe(self):
        real_popen = subprocess.Popen
        echo = [sys.executable, "-c", "import sys; sys.stdout.write(sys.stdin.read())"]

        def fake_ssh(command, **kwargs):  # same pipe options as ssh, local echo process
            self.assertEqual(command[0], "ssh")
            return real_popen(echo, stdout=subprocess.PIPE, **kwargs)

        args = sender.build_parser().parse_args([])
        pad = SimpleNamespace(get_axis=lambda code: {1: -32768}.get(code, 0),
                              get_button=lambda code: code == 9)
        pygame = SimpleNamespace(event=SimpleNamespace(pump=lambda: None),
                                 CONTROLLER_AXIS_LEFTX=0, CONTROLLER_AXIS_LEFTY=1,
                                 CONTROLLER_AXIS_RIGHTX=2, CONTROLLER_BUTTON_LEFTSHOULDER=9,
                                 CONTROLLER_BUTTON_RIGHTSHOULDER=10, CONTROLLER_BUTTON_B=1,
                                 CONTROLLER_BUTTON_START=6, CONTROLLER_AXIS_TRIGGERLEFT=4,
                                 CONTROLLER_BUTTON_DPAD_UP=11, CONTROLLER_BUTTON_DPAD_DOWN=12,
                                 CONTROLLER_BUTTON_X=2, CONTROLLER_BUTTON_Y=3)
        reads = {"n": 0}
        real_read = sender.read_sticks

        def read_twice(pg, pd):  # one command, then START
            reads["n"] += 1
            sticks = real_read(pg, pd)
            return sticks if reads["n"] == 1 else dict(sticks, start=True)

        with patch.object(sender.subprocess, "Popen", side_effect=fake_ssh), \
                patch.object(sender, "read_sticks", side_effect=read_twice), \
                patch("sys.stderr", io.StringIO()):
            ssh = sender.start_ssh(args)
            sender.stream(pygame, pad, ssh.stdin, args, ssh)
            ssh.stdin.close()
            output = ssh.stdout.read()
            ssh.wait()
        self.assertEqual(json.loads(output), {"x": 1.0, "y": 0.0, "yaw": 0.0, "enable": True,
                                              "arm": False, "ax": 0.0, "az": 0.0, "claw": 0.0,
                                              "bark": 0, "kneel": 0, "record": 0, "replay": 0})

    def state(self, **overrides):
        state = dict(left_x=0.0, left_y=0.0, right_x=0.0, lb=False, rb=False, b=False, start=False,
                     lt=False, dpad_up=False, dpad_down=False, x=False, y=False)
        state.update(overrides)
        return state

    def test_rb_switches_sticks_to_arm_and_disables_walking(self):
        arming = sender.Arming(deadman=True)
        message = sender.make_message(self.state(left_x=1, left_y=-1, right_x=1, lb=True, rb=True),
                                      arming, 0.1, barks=2)
        self.assertEqual(message, {"x": 1.0, "y": -1.0, "yaw": -1.0, "enable": False,
                                   "arm": True, "ax": 1.0, "az": 1.0, "claw": 1.0, "bark": 2,
                                   "kneel": 0, "record": 0, "replay": 0})
        message = sender.make_message(self.state(left_x=-1, left_y=1, right_x=-1, rb=True),
                                      arming, 0.1, barks=2)
        self.assertEqual((message["ax"], message["az"], message["claw"]), (-1.0, -1.0, -1.0))

    def test_x_and_y_press_counters_are_sent(self):
        message = sender.make_message(self.state(), sender.Arming(deadman=True), 0.1, 0,
                                      records=3, replays=2)
        self.assertEqual((message["record"], message["replay"]), (3, 2))

    def test_video_flag_passes_port_and_tunnels_through_ssh(self):
        # The tailnet only allows SSH to the robot, so video rides inside the SSH session.
        args = sender.build_parser().parse_args(["--video"])
        self.assertIn("--video-port 8090", sender.remote_command(args))
        self.assertEqual(sender.video_url(args), "http://localhost:8090/")
        with patch.object(sender.subprocess, "Popen") as popen:
            sender.start_ssh(args)
            command = popen.call_args[0][0]
        self.assertIn("8090:127.0.0.1:8090", command)
        self.assertEqual(command[command.index("8090:127.0.0.1:8090") - 1], "-L")
        plain = sender.build_parser().parse_args([])
        self.assertNotIn("--video-port", sender.remote_command(plain))
        with patch.object(sender.subprocess, "Popen") as popen:
            sender.start_ssh(plain)
            self.assertNotIn("-L", popen.call_args[0][0])

    def test_lt_with_dpad_kneels_and_stands(self):
        arming = sender.Arming(deadman=True)
        kneel = lambda **s: sender.make_message(self.state(**s), arming, 0.1, 0)["kneel"]  # noqa: E731
        self.assertEqual(kneel(lt=True, dpad_down=True), 1)
        self.assertEqual(kneel(lt=True, dpad_up=True), -1)
        self.assertEqual(kneel(dpad_down=True), 0)  # D-pad alone does nothing
        self.assertEqual(kneel(lt=True), 0)
        self.assertEqual(kneel(lt=True, dpad_up=True, dpad_down=True), 0)

    def test_kneel_works_alongside_arm_mode(self):
        message = sender.make_message(self.state(rb=True, lt=True, dpad_down=True, left_y=-1),
                                      sender.Arming(deadman=True), 0.1, 0)
        self.assertEqual((message["arm"], message["az"], message["kneel"]), (True, 1.0, 1))

    def test_arm_axes_are_zero_outside_arm_mode(self):
        message = sender.make_message(self.state(left_x=1, left_y=-1, right_x=1, lb=True),
                                      sender.Arming(deadman=True), 0.1, barks=0)
        self.assertTrue(message["enable"])
        self.assertEqual((message["arm"], message["ax"], message["az"], message["claw"]),
                         (False, 0.0, 0.0, 0.0))

    def test_new_receiver_flags_pass_through(self):
        args = sender.build_parser().parse_args(["--ramp", "2", "--arm-home-z", "50"])
        self.assertIn("--ramp 2 --arm-home-z 50", sender.remote_command(args))


class FakeClock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


class FakeSound:
    def __init__(self):
        self.done = False

    def poll(self):
        return 0 if self.done else None


def arm_line(ax=0.0, az=0.0, claw=0.0, **extra):
    message = {"x": 0, "y": 0, "yaw": 0, "enable": True, "arm": True,
               "ax": ax, "az": az, "claw": claw}
    message.update(extra)
    return json.dumps(message) + "\n"


class ArmBarkRampTests(unittest.TestCase):
    def setUp(self):
        self.args = receiver.build_parser().parse_args([])  # arm 60 mm/s, claw 170/s, home 40,30
        self.dog = FakeDog()
        self.clock = FakeClock()
        self.arm = receiver.ArmController(self.dog, self.args, clock=self.clock)

    def tick(self, seconds, **command):
        self.clock.now += seconds
        self.arm.update(dict({"ax": 0.0, "az": 0.0, "claw": 0.0}, **command))

    def test_arm_mode_overrides_walking(self):
        message = receiver.parse_message(arm_line(ax=0.5, x=1))
        walk, arm = message.walk, message.arm
        self.assertEqual(walk, {"x": 0.0, "y": 0.0, "yaw": 0.0})
        self.assertEqual(arm, {"ax": 0.5, "az": 0.0, "claw": 0.0})

    def test_nothing_sent_until_a_stick_moves_then_starts_from_home(self):
        self.tick(0.05)
        self.tick(0.05)
        self.assertEqual(self.dog.calls, [])
        self.tick(0.05, az=1)  # first moving tick: no elapsed time yet counted from idle
        self.tick(0.05, az=1)  # +3 mm at 60 mm/s
        self.assertEqual(self.dog.calls, [("arm", 80, 30), ("arm", 80, 33)])

    def test_arm_rate_is_capped_after_a_stall_and_clamped_to_limits(self):
        self.tick(0, ax=1)
        self.tick(5.0, ax=1)  # a 5 s gap counts as at most 0.1 s: +6 mm, not +300
        self.assertEqual(self.dog.calls[-1], ("arm", 86, 30))
        for _ in range(100):
            self.tick(0.1, ax=1, az=-1)
        x, z = self.dog.calls[-1][1:]
        self.assertAlmostEqual(math.hypot(x, z), receiver.ARM_REACH[1], delta=1)
        self.assertGreaterEqual(z, receiver.ARM_LIMITS["z"][0])

    def test_target_never_leaves_reach_so_reversing_moves_at_once(self):
        # Measured on the robot: (140, 30) followed, (150, 30) ignored; (80, 115) followed,
        # (80, 130) ignored. Out-of-reach targets are silently ignored by the firmware.
        self.tick(0, ax=1)
        for _ in range(50):  # hold "reach out" far longer than needed
            self.tick(0.1, ax=1)
        x, z = self.dog.calls[-1][1:]
        self.assertLessEqual(math.hypot(x, z), receiver.ARM_REACH[1] + 0.5)
        self.tick(0.1, ax=-1)  # pull in: the very next command must be reachable and different
        x2, z2 = self.dog.calls[-1][1:]
        self.assertLess(x2, x)

    def test_diagonal_windup_cannot_strand_the_arm(self):
        self.tick(0, ax=1, az=1)
        for _ in range(50):
            self.tick(0.1, ax=1, az=1)
        for _ in range(3):  # only pull in; height was pushed out of reach too
            self.tick(0.1, ax=-1)
        for call in self.dog.calls:
            self.assertLessEqual(math.hypot(*call[1:]), receiver.ARM_REACH[1] + 0.5, call)
        self.assertNotEqual(self.dog.calls[-1], self.dog.calls[-4])

    def test_target_stays_out_of_the_unreachable_core_and_in_front(self):
        # (70, 30) followed but (55, 30) was ignored: too close to the arm base.
        self.tick(0, ax=-1)
        for _ in range(50):
            self.tick(0.1, ax=-1)
        for call in self.dog.calls:
            x, z = call[1:]
            self.assertGreaterEqual(math.hypot(x, z), receiver.ARM_REACH[0] - 1, call)  # int rounding
            self.assertGreaterEqual(x, 0, call)

    def test_stops_at_the_edge_instead_of_sliding_along_it(self):
        self.tick(0, ax=1)
        for _ in range(50):  # hold "reach out" at z=30 well past the edge
            self.tick(0.1, ax=1)
        self.assertEqual(self.dog.calls[-1][2], 30)  # height unchanged: no drift downwards
        start = len(self.dog.calls)
        for _ in range(10):  # at full reach, raising is out of reach too: nothing is sent
            self.tick(0.1, az=1)
        self.assertEqual(len(self.dog.calls), start)

    def test_default_home_is_reachable(self):
        self.assertTrue(receiver.ARM_REACH[0] <= math.hypot(self.args.arm_home_x,
                                                            self.args.arm_home_z)
                        <= receiver.ARM_REACH[1])

    def test_set_pose_and_claw_send_and_notify(self):
        heard = []
        self.arm.listener = lambda kind, value: heard.append((kind, value))
        self.arm.set_pose(100, 20)
        self.arm.set_claw(200)
        self.arm.set_pose(100, 20)  # unchanged: nothing resent
        self.assertEqual(self.dog.calls, [("arm", 100, 20), ("claw", 200)])
        self.assertEqual(heard, [("arm", [100, 20]), ("claw", 200)])
        self.tick(0, az=1)
        self.tick(0.1, az=1)  # manual control continues from the set pose
        self.assertEqual(self.dog.calls[-1], ("arm", 100, 26))

    def test_manual_moves_notify_listener(self):
        heard = []
        self.arm.listener = lambda kind, value: heard.append((kind, value))
        self.tick(0, claw=1)
        self.assertEqual(heard, [("claw", 128)])

    def test_claw_moves_without_touching_arm(self):
        self.tick(0, claw=1)
        self.tick(0.1, claw=1)  # 128 + 17
        self.tick(0.1, claw=-1)
        self.assertEqual(self.dog.calls, [("claw", 128), ("claw", 145), ("claw", 128)])

    def test_idle_resets_elapsed_time(self):
        self.tick(0, az=1)
        self.arm.idle()
        self.tick(3.0, az=1)
        self.assertEqual(self.dog.calls, [("arm", 80, 30)])

    def test_run_routes_arm_messages(self):
        driver = receiver.Driver(self.dog, self.args)
        receiver.run(driver, Paced(arm_line(az=1), arm_line(az=1), receiver.EOF), 0.5,
                     lambda text: None, arm=self.arm)
        self.assertEqual(self.dog.calls[0], ("arm", 80, 30))
        self.assertNotIn(("x", 12), self.dog.calls)

    def test_bark_plays_once_per_press(self):
        sounds = []
        barker = receiver.Barker(play=lambda: sounds.append(FakeSound()) or sounds[-1])
        barker.update(0)
        self.assertEqual(len(sounds), 0)
        barker.update(1)  # a press in the very first processed line still barks
        self.assertEqual(len(sounds), 1)
        sounds[0].done = True
        barker.update(1)
        self.assertEqual(len(sounds), 1)
        barker.update(6)
        self.assertEqual(len(sounds), 2)
        barker.update(7)  # still playing: ignored
        self.assertEqual(len(sounds), 2)
        sounds[1].done = True
        barker.update(9)  # two presses lost in skipped lines still give one bark
        self.assertEqual(len(sounds), 3)

    def test_bad_bark_counter_is_rejected(self):
        for bad in (-1, 1.5, True, "1"):
            with self.assertRaises(ValueError):
                receiver.parse_message(line(bark=bad))

    def test_ramp_grows_stride_and_resets(self):
        ramp = receiver.Ramp(2.0, 0.4, clock=self.clock)
        walk = {"x": 1.0, "y": -0.5, "yaw": 1.0}
        self.assertEqual(ramp.apply(walk), {"x": 0.4, "y": -0.2, "yaw": 1.0})
        self.clock.now += 1.0
        self.assertAlmostEqual(ramp.apply(walk)["x"], 0.7)
        self.clock.now += 5.0
        self.assertEqual(ramp.apply(walk)["x"], 1.0)
        self.assertAlmostEqual(ramp.apply({"x": -1.0, "y": -0.5, "yaw": 0})["x"], -0.4)  # reversed
        self.assertEqual(ramp.apply(walk)["y"], -0.5)  # y kept its direction the whole time
        ramp.apply({"x": 0.0, "y": 0.0, "yaw": 0})
        self.assertEqual(ramp.apply(walk)["x"], 0.4)

    def test_ramp_off_by_default(self):
        ramp = receiver.Ramp(self.args.ramp, self.args.ramp_start, clock=self.clock)
        self.assertEqual(ramp.apply({"x": 1.0, "y": 1.0, "yaw": 0.0})["x"], 1.0)

    def test_watchdog_resets_ramp(self):
        ramp = receiver.Ramp(2.0, 0.4, clock=self.clock)
        ramp.apply({"x": 1.0, "y": 0.0, "yaw": 0.0})
        driver = receiver.Driver(self.dog, self.args)
        receiver.run(driver, Paced(QUIET, receiver.EOF), 0.5, lambda text: None, ramp=ramp)
        self.assertEqual(ramp.since, {})


class KneelTests(unittest.TestCase):
    def setUp(self):
        self.args = receiver.build_parser().parse_args([])  # 2 s to full kneel: pitch 10, z 85->70
        self.dog = FakeDog()
        self.clock = FakeClock()
        self.posture = receiver.Posture(self.dog, self.args, clock=self.clock)

    def tick(self, seconds, direction):
        self.clock.now += seconds
        self.posture.update(direction)

    def test_kneel_parses_and_rejects_bad_values(self):
        self.assertEqual(receiver.parse_message(line(kneel=1)).kneel, 1)
        self.assertEqual(receiver.parse_message(line()).kneel, 0)
        for bad in (2, 0.5, True, "1"):
            with self.assertRaises(ValueError):
                receiver.parse_message(line(kneel=bad))

    def test_nothing_sent_until_kneel_is_pressed(self):
        self.tick(0.1, 0)
        self.tick(0.1, -1)  # already standing: "up" does nothing
        self.assertEqual(self.dog.calls, [])

    def test_kneel_down_ramps_pitch_and_height_then_holds(self):
        self.tick(0, 1)
        self.tick(0.1, 1)  # 5% of the way
        self.assertEqual(self.dog.calls, [("attitude", "p", 0), ("translation", "z", 85),
                                          ("attitude", "p", 0.5), ("translation", "z", 84.25)])
        for _ in range(40):
            self.tick(0.1, 1)
        self.assertEqual(self.dog.calls[-2:], [("attitude", "p", 10), ("translation", "z", 70)])
        sent = len(self.dog.calls)
        self.tick(0.1, 1)  # fully knelt: nothing more
        self.tick(0.1, 0)  # released: holds
        self.assertEqual(len(self.dog.calls), sent)

    def test_stand_up_and_exit_restores_standing(self):
        self.tick(0, 1)
        for _ in range(10):
            self.tick(0.1, 1)
        self.tick(0.1, -1)
        self.tick(0.1, -1)
        self.assertEqual(self.dog.calls[-2:], [("attitude", "p", 4.0), ("translation", "z", 79.0)])
        self.posture.stand()
        self.assertEqual(self.dog.calls[-2:], [("attitude", "p", 0), ("translation", "z", 85)])
        sent = len(self.dog.calls)
        self.posture.stand()  # already standing
        self.assertEqual(len(self.dog.calls), sent)

    def test_a_stall_does_not_jump(self):
        self.tick(0, 1)
        self.tick(5.0, 1)  # counts as at most 0.1 s
        self.assertEqual(self.dog.calls[-2:], [("attitude", "p", 0.5), ("translation", "z", 84.25)])

    def test_run_routes_kneel_and_stands_on_exit(self):
        driver = receiver.Driver(self.dog, self.args)
        self.posture.update(1)  # first press sends the standing pose
        self.posture.level = 0.5
        receiver.run(driver, Paced(line(kneel=1), line(kneel=1), receiver.EOF), 0.5,
                     lambda text: None, posture=self.posture)
        self.assertIn(("attitude", "p", 5.0), self.dog.calls)  # routed: level 0.5 was sent
        self.assertEqual(self.dog.calls[-2:], [("attitude", "p", 0), ("translation", "z", 85)])

    def test_set_level_sends_and_notifies(self):
        heard = []
        self.posture.listener = lambda kind, value: heard.append((kind, value))
        self.posture.set_level(0.5)
        self.assertEqual(self.dog.calls, [("attitude", "p", 5.0), ("translation", "z", 77.5)])
        self.assertEqual(heard, [("kneel", 0.5)])
        self.posture.set_level(7)  # clamped
        self.assertEqual(self.dog.calls[-2:], [("attitude", "p", 10), ("translation", "z", 70)])

    def test_robot_state_snapshot(self):
        arm = receiver.ArmController(self.dog, self.args, clock=self.clock)
        driver = receiver.Driver(self.dog, self.args)
        self.assertEqual(receiver.robot_state(driver, arm, self.posture),
                         {"arm": None, "claw": None, "kneel": 0.0,
                          "walk": {"x": 0, "y": 0, "yaw": 0}})
        arm.set_pose(90, 10)
        self.posture.set_level(0.25)
        state = receiver.robot_state(driver, arm, self.posture)
        self.assertEqual((state["arm"], state["kneel"]), ([90, 10], 0.25))

    def test_kneel_flags_are_bounded(self):
        parser = receiver.build_parser()
        with patch("sys.stderr", io.StringIO()):
            for argv in (["--kneel-pitch", "11"], ["--kneel-height", "59"],
                         ["--stand-height", "111"], ["--kneel-time", "0"]):
                with self.assertRaises(SystemExit):
                    parser.parse_args(argv)


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

    def test_message_carries_counters_and_raw(self):
        message = receiver.parse_message(line(record=2, replay=1))
        self.assertEqual((message.record, message.replay), (2, 1))
        self.assertEqual(message.raw["record"], 2)
        self.assertEqual(receiver.parse_message(line()).record, 0)

    def test_manual_means_real_input(self):
        self.assertFalse(receiver.parse_message(line(enable=True)).manual)  # LB, sticks centred
        self.assertTrue(receiver.parse_message(line(enable=False, x=0.5)).manual)
        self.assertTrue(receiver.parse_message(line(enable=False, yaw=-1)).manual)
        self.assertTrue(receiver.parse_message(arm_line()).manual)  # RB held
        self.assertTrue(receiver.parse_message(line(enable=False, kneel=1)).manual)
        with self.assertRaises(ValueError):
            receiver.parse_message(line(enable=False, x="fast"))

    def test_stand_still_runs_when_teach_close_fails(self):
        events = []

        def broken_close():
            raise OSError(28, "No space left on device")

        teach_stub = SimpleNamespace(update=lambda message, manual: False,
                                     watchdog=lambda: None, close=broken_close)
        posture = receiver.Posture(self.dog, self.args, clock=FakeClock())
        posture.stand = lambda: events.append("stand")
        receiver.run(self.driver, Paced(line(), receiver.EOF), 0.5, self.log.append,
                     posture=posture, teach=teach_stub)
        self.assertEqual(events, ["stand"])
        self.assertTrue(any("No space left" in entry for entry in self.log))

    def test_bad_counters_are_rejected(self):
        for key in ("record", "replay"):
            for bad in (-1, 1.5, True, "1"):
                with self.assertRaises(ValueError):
                    receiver.parse_message(line(**{key: bad}))

    def test_speed_flags_are_bounded(self):
        parser = receiver.build_parser()
        with patch("sys.stderr", io.StringIO()):
            for argv in (["--max-x", "26"], ["--max-y", "0"], ["--max-turn", "nan"]):
                with self.assertRaises(SystemExit):
                    parser.parse_args(argv)


    def test_run_records_motion_and_replays_through_teach(self):
        import teach

        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        clock = FakeClock()
        args = receiver.build_parser().parse_args([])
        dog = FakeDog()
        arm = receiver.ArmController(dog, args, clock=clock)
        posture = receiver.Posture(dog, args, clock=clock)
        driver = receiver.Driver(dog, args)
        frame = __import__("numpy").zeros((240, 320, 3), dtype="uint8")
        camera = SimpleNamespace(start_async=lambda: None, close=lambda: None, error=None,
                                 latest=lambda: (frame, clock.now))
        box = {"bbox": [140, 100, 40, 40], "center": [160, 120], "area_px": 1600}
        recorder = teach.Recorder(Path(tmp.name), camera, lambda f: {"purple": [box]}, "purple",
                                  clock=clock, free_mb=lambda p: 10 ** 6, encode=lambda f: b"j")
        arm.listener = posture.listener = recorder.on_send
        replayer = teach.Replayer(arm, posture, clock=clock, settle=0)
        controller = teach.TeachController(
            camera, recorder, replayer, Path(tmp.name),
            lambda: receiver.robot_state(driver, arm, posture), lambda text: None)
        idle = dict(enable=False, record=2, replay=1)  # no LB: not manual input
        lines = [line(record=1),
                 arm_line(az=1, record=1), arm_line(az=1, record=1),
                 line(record=2),
                 line(**idle), line(**idle), line(**idle), receiver.EOF]
        clock_lines = Paced(*lines)
        real_get = clock_lines.get

        def get(timeout=None):
            clock.now += 0.1
            return real_get(timeout)

        clock_lines.get = get
        receiver.run(driver, clock_lines, 0.5, lambda text: None, arm=arm, posture=posture,
                     teach=controller)
        session = next(Path(tmp.name).iterdir())
        motion = json.loads((session / "motion.json").read_text())
        self.assertEqual(motion["events"][0]["arm"], [80, 30])
        replayed = [call for call in dog.calls if call[0] == "arm"]
        self.assertEqual(replayed[-1], ("arm", 80, 36))  # the replay ends at the last pose

    def test_run_finalises_recording_on_exit(self):
        events = []
        teach_stub = SimpleNamespace(
            update=lambda message, manual: False, watchdog=lambda: None,
            close=lambda: events.append("close"))
        posture = receiver.Posture(self.dog, self.args, clock=FakeClock())
        posture.stand = lambda: events.append("stand")
        receiver.run(self.driver, Paced(line(), receiver.EOF), 0.5, self.log.append,
                     posture=posture, teach=teach_stub)
        self.assertEqual(events, ["close", "stand"])

    def test_viewer_serves_only_on_the_robot_itself(self):
        frame = __import__("numpy").zeros((4, 4, 3), dtype="uint8")
        camera = SimpleNamespace(start=lambda: None,
                                 latest=lambda: (frame, __import__("time").monotonic()))
        viewer = receiver.start_viewer(camera, 0, self.log.append)
        try:
            self.assertEqual(viewer.host, "127.0.0.1")
            self.assertTrue(any("Live video" in entry for entry in self.log))
        finally:
            viewer.close()

    def test_viewer_failure_is_reported_not_raised(self):
        def broken():
            raise RuntimeError("cannot open camera 0")

        self.assertIsNone(receiver.start_viewer(SimpleNamespace(start=broken), 0,
                                                self.log.append))
        self.assertTrue(any("cannot open camera" in entry for entry in self.log))

    def test_teach_flags(self):
        args = receiver.build_parser().parse_args([])
        self.assertEqual((args.color, args.record_fps, args.min_free_mb, args.replay_settle),
                         ("purple", 10, 200, 1.0))
        self.assertTrue(args.teach_dir.endswith("xgo_teach"))
        with patch("sys.stderr", io.StringIO()):
            for argv in (["--color", "red"], ["--record-fps", "0"], ["--min-free-mb", "10"]):
                with self.assertRaises(SystemExit):
                    receiver.build_parser().parse_args(argv)


if __name__ == "__main__":
    unittest.main()
