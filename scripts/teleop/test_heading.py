"""Offline checks for heading hold; a scripted gyro stands in for the robot's IMU."""

import json
import struct
import sys
import tempfile
import unittest
from pathlib import Path

import heading

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "sensor_monitor"))

STRAIGHT = {"x": 12, "y": 0, "yaw": 0}
STOPPED = {"x": 0, "y": 0, "yaw": 0}


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


class Gyro:
    """read() returns the true heading `yaw` plus a constant drift of `bias` deg/s (the real
    IMU drifted 6.9 deg/s standing still); set `fail` to raise like a missed UART reply."""

    def __init__(self, yaw=0.0, clock=None, bias=0.0):
        self.yaw, self.fail, self.reads, self.clock, self.bias = yaw, False, 0, clock, bias

    def read(self):
        self.reads += 1
        if self.fail:
            raise TimeoutError("no valid reply from register 0x64")
        drift = self.bias * (self.clock.now - 100.0) if self.clock else 0.0
        return self.yaw + drift


class HeadingHoldTests(unittest.TestCase):
    def make(self, sign=None, bias=0.0, gyro_bias=0.0, **kwargs):
        self.clock, self.log = Clock(), []
        self.gyro = Gyro(clock=self.clock, bias=gyro_bias)
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.hold = heading.HeadingHold(self.gyro.read, self.clock, log=self.log.append,
                                        sign=sign, sign_path=Path(self.tmp.name) / "heading.json",
                                        bias=bias, **kwargs)
        return self.hold

    def stand_still(self, seconds):
        for _ in range(int(seconds / 0.05)):
            self.clock.now += 0.05
            self.hold.idle()

    def tick(self, steps, seconds=0.2):
        self.clock.now += seconds
        return self.hold.adjust(steps)

    def test_angle_difference_wraps(self):
        self.assertEqual(heading.angle_diff(170, -170), -20)
        self.assertEqual(heading.angle_diff(-170, 170), 20)
        self.assertEqual(heading.angle_diff(10, 5), 5)

    def test_corrects_drift_back_toward_the_starting_heading(self):
        self.make(sign=1)
        self.assertEqual(self.tick(STRAIGHT)["yaw"], 0)  # locks the heading
        self.gyro.yaw = 5.0  # drifted 5 deg to the left
        self.assertEqual(self.tick(STRAIGHT), {"x": 12, "y": 0, "yaw": -10})  # turn right
        self.gyro.yaw = -30.0
        self.assertEqual(self.tick(STRAIGHT)["yaw"], 25)  # clamped to the maximum
        self.gyro.yaw = 1.0
        self.assertEqual(self.tick(STRAIGHT)["yaw"], 0)  # inside the 2 deg deadband

    def test_opposite_gyro_direction_flips_the_correction(self):
        self.make(sign=-1)
        self.tick(STRAIGHT)
        self.gyro.yaw = 5.0
        self.assertEqual(self.tick(STRAIGHT)["yaw"], 10)

    def test_no_correction_until_the_gyro_direction_is_known(self):
        self.make()
        self.tick(STRAIGHT)
        self.gyro.yaw = 8.0
        self.assertEqual(self.tick(STRAIGHT)["yaw"], 0)
        self.assertTrue(any("turn once" in line for line in self.log))

    def test_learns_and_saves_the_gyro_direction_from_a_manual_turn(self):
        self.make()
        left = {"x": 0, "y": 0, "yaw": 25}
        self.tick(left)
        self.gyro.yaw = 3.0
        self.tick(left)
        self.assertIsNone(self.hold.sign)  # not enough rotation yet
        self.gyro.yaw = 9.0
        self.tick(left)
        self.assertEqual(self.hold.sign, 1)  # turning left made the reading go up
        saved = json.loads((Path(self.tmp.name) / "heading.json").read_text())
        self.assertEqual(saved["sign"], 1)
        again = heading.HeadingHold(self.gyro.read, self.clock, log=self.log.append,
                                    sign_path=Path(self.tmp.name) / "heading.json")
        self.assertEqual(again.sign, 1)

    def test_learns_a_negative_direction_too(self):
        self.make()
        right = {"x": 0, "y": 0, "yaw": -25}
        self.tick(right)
        self.gyro.yaw = 7.0  # turning right made the reading go up: the gyro counts clockwise
        self.tick(right)
        self.assertEqual(self.hold.sign, -1)

    def test_turning_or_stopping_releases_the_hold_and_relocks_afterwards(self):
        self.make(sign=1)
        self.tick(STRAIGHT)
        turn = {"x": 12, "y": 0, "yaw": 20}
        self.gyro.yaw = 40.0
        self.assertEqual(self.tick(turn), turn)  # the operator's turn wins, unchanged
        self.assertEqual(self.tick(STRAIGHT)["yaw"], 0)  # new heading (40) locked
        self.gyro.yaw = 45.0
        self.assertEqual(self.tick(STRAIGHT)["yaw"], -10)
        self.assertEqual(self.tick(STOPPED), STOPPED)
        self.gyro.yaw = 90.0
        self.assertEqual(self.tick(STRAIGHT)["yaw"], 0)  # re-locked after stopping

    def test_failed_reads_never_cause_a_correction(self):
        self.make(sign=1)
        self.tick(STRAIGHT)
        self.gyro.yaw = 10.0
        self.gyro.fail = True
        for _ in range(4):
            self.assertEqual(self.tick(STRAIGHT)["yaw"], 0)
        self.assertEqual(sum("gyro" in line for line in self.log), 1)  # reported once

    def test_reads_are_rate_limited(self):
        self.make(sign=1)
        for _ in range(10):
            self.tick(STRAIGHT, seconds=0.05)  # 20 Hz ticks for 0.5 s
        self.assertLessEqual(self.gyro.reads, 3)

    def test_runaway_disables_the_hold(self):
        self.make(sign=1)
        self.tick(STRAIGHT)
        for yaw in (10, 20, 31, 40):  # keeps turning away despite corrections
            self.gyro.yaw = yaw
            self.tick(STRAIGHT)
        self.assertEqual(self.tick(STRAIGHT)["yaw"], 0)
        self.assertFalse(self.hold.enabled)
        self.assertTrue(any("disabled" in line for line in self.log))

    def test_no_correction_until_the_drift_has_been_measured(self):
        self.make(sign=1, bias=None, gyro_bias=6.9)
        self.tick(STRAIGHT)
        self.gyro.yaw = 10.0
        self.assertEqual(self.tick(STRAIGHT)["yaw"], 0)
        self.assertTrue(any("stand still" in line for line in self.log))

    def test_measures_the_drift_while_standing_still(self):
        self.make(sign=1, bias=None, gyro_bias=6.9)
        self.stand_still(1.0)
        self.assertIsNone(self.hold.bias)  # not enough steady data yet (0.5 s settle + 2 s)
        self.stand_still(2.5)
        self.assertAlmostEqual(self.hold.bias, 6.9, places=2)
        self.assertTrue(any("6.9" in line for line in self.log))

    def test_steady_drift_is_not_mistaken_for_turning(self):
        self.make(sign=1, bias=None, gyro_bias=6.9)
        self.stand_still(3.0)
        corrections = [self.tick(STRAIGHT)["yaw"] for _ in range(15)]  # 3 s walking straight
        self.assertEqual(corrections, [0] * 15)

    def test_real_turning_on_top_of_drift_is_corrected(self):
        self.make(sign=1, bias=None, gyro_bias=6.9)
        self.stand_still(3.0)
        self.tick(STRAIGHT)
        self.gyro.yaw = 5.0  # really turned 5 deg left, on top of the drift
        self.assertEqual(self.tick(STRAIGHT)["yaw"], -10)

    def test_direction_learning_subtracts_the_drift(self):
        # a slow right turn (-5 deg/s) plus +6.9 deg/s drift reads as turning *left*
        self.make(bias=None, gyro_bias=6.9)
        self.stand_still(3.0)
        right = {"x": 0, "y": 0, "yaw": -25}
        for _ in range(8):
            self.clock.now += 0.2
            self.gyro.yaw -= 1.0  # -5 deg/s true rotation
            self.hold.adjust(right)
        self.assertEqual(self.hold.sign, 1)  # counter-clockwise positive, not fooled

    def test_moving_restarts_the_still_period(self):
        self.make(sign=1, bias=None, gyro_bias=6.9)
        self.stand_still(1.5)
        self.tick(STRAIGHT)  # walked: the earlier still samples no longer count
        self.stand_still(1.5)
        self.assertIsNone(self.hold.bias)

    def test_forgetting_the_drift_requires_a_new_measurement(self):
        self.make(sign=1, bias=None, gyro_bias=6.9)
        self.stand_still(3.0)
        self.assertIsNotNone(self.hold.bias)
        self.hold.forget_drift()
        self.assertIsNone(self.hold.bias)
        self.assertEqual(self.tick(STRAIGHT)["yaw"], 0)
        self.assertEqual(sum("stand still" in line for line in self.log), 1)
        self.stand_still(3.0)
        self.assertIsNotNone(self.hold.bias)

    def test_disabled_hold_passes_commands_through(self):
        self.make(sign=1, enabled=False)
        self.tick(STRAIGHT)
        self.gyro.yaw = 20.0
        self.assertEqual(self.tick(STRAIGHT), STRAIGHT)
        self.assertEqual(self.gyro.reads, 0)


class FakeSerial:
    """Replies to one read request with a framed 4-byte float, like the motion controller."""

    def __init__(self, value):
        payload = struct.pack("<f", value)
        length = 5 + len(payload) + 3
        body = [0x55, 0x00, length, 0x12, heading.YAW_REGISTER, *payload]
        check = 255 - (length + 0x12 + heading.YAW_REGISTER + sum(payload)) % 256
        self.reply = bytes(body + [check, 0x00, 0xAA])
        self.written = []

    def reset_input_buffer(self):
        pass

    def write(self, packet):
        self.written.append(bytes(packet))
        self.pending = bytearray(self.reply)

    def read(self, size=1):
        if not getattr(self, "pending", None):
            return b""
        byte = bytes(self.pending[:1])
        del self.pending[:1]
        return byte


class YawReaderTests(unittest.TestCase):
    def test_reads_and_decodes_yaw_with_a_read_request(self):
        connection = FakeSerial(-12.5)
        read = heading.make_yaw_reader(connection, timeout=0.05)
        self.assertEqual(read(), -12.5)
        packet = connection.written[0]
        self.assertEqual((packet[3], packet[4], packet[5]), (0x02, 0x64, 4))  # READ mode only

    def test_garbage_and_silence_raise_instead_of_returning_zero(self):
        self.assertRaises(ValueError, heading.make_yaw_reader(FakeSerial(float("nan"))))
        silent = FakeSerial(1.0)
        silent.reply = b""
        self.assertRaises(TimeoutError, heading.make_yaw_reader(silent, timeout=0.05))


class InnerDriver:
    def __init__(self):
        self.calls, self.current = [], dict(STOPPED)

    def apply(self, steps):
        self.calls.append(dict(steps))
        self.current = dict(steps)

    def stop(self):
        self.calls.append("stop")
        self.current = dict(STOPPED)


class HeadingDriverTests(unittest.TestCase):
    def setUp(self):
        self.clock, self.gyro = Clock(), Gyro()
        self.hold = heading.HeadingHold(self.gyro.read, self.clock, log=lambda text: None, sign=1,
                                        bias=0.0)
        self.inner = InnerDriver()
        self.driver = heading.HeadingDriver(self.inner, self.hold)
        self.heard = []
        self.driver.listener = lambda kind, value: self.heard.append((kind, value))

    def test_applies_corrections_but_reports_and_records_what_was_asked(self):
        self.driver.apply(STRAIGHT)
        self.clock.now += 0.2
        self.gyro.yaw = 5.0
        self.driver.apply(STRAIGHT)
        self.assertEqual(self.inner.calls[-1], {"x": 12, "y": 0, "yaw": -10})
        self.assertEqual(self.driver.current, STRAIGHT)  # the operator's request
        self.assertEqual(self.heard, [("walk", STRAIGHT)])  # recorded once, uncorrected

    def test_refresh_keeps_correcting_a_replayed_walk(self):
        self.driver.apply(STRAIGHT)  # a replay sends this once
        self.clock.now += 0.2
        self.gyro.yaw = -5.0
        self.driver.refresh()  # every loop tick
        self.assertEqual(self.inner.calls[-1], {"x": 12, "y": 0, "yaw": 10})

    def test_stop_stops_and_resets(self):
        self.driver.apply(STRAIGHT)
        self.driver.stop()
        self.assertEqual(self.inner.calls[-1], "stop")
        self.assertEqual(self.driver.current, STOPPED)
        self.assertEqual(self.heard[-1], ("walk", STOPPED))
        self.driver.stop()  # already stopped: nothing new to record
        self.assertEqual(len(self.heard), 2)


if __name__ == "__main__":
    unittest.main()
