"""Protocol tests using fake UART replies; never access robot hardware."""

import contextlib
import io
import json
import struct
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import stream_sensors


class FakeSerial:
    def __init__(self, replies, corrupt=False):
        self.replies = replies
        self.corrupt = corrupt
        self.pending = bytearray()
        self.writes = []

    def reset_input_buffer(self):
        self.pending.clear()

    def write(self, packet):
        self.writes.append(packet)
        assert packet[:4] == b"\x55\x00\x09\x02"  # only READ packets
        address, size = packet[4:6]
        assert packet[6] == 255 - (9 + 2 + address + size) % 256
        payload = self.replies.get(address)
        if payload is None:
            return len(packet)
        length = 8 + len(payload)
        typ = 0x12  # reply type need not equal request type
        checksum = 255 - (length + typ + address + sum(payload)) % 256
        if self.corrupt:
            checksum ^= 1
        self.pending.extend(bytes((0x55, 0, length, typ, address)) + payload
                            + bytes((checksum, 0, 0xAA)))
        return len(packet)

    def read(self, amount):
        if not self.pending:
            return b""
        value = self.pending[:amount]
        del self.pending[:amount]
        return bytes(value)


def all_replies():
    return {
        stream_sensors.BATTERY: bytes((76,)),
        stream_sensors.ROLL: struct.pack("<f", 3.5),
        stream_sensors.PITCH: struct.pack("<f", -4.25),
        stream_sensors.YAW: struct.pack("<f", 88.0),
        stream_sensors.MOTOR_ANGLES: bytes((128,)) * 15,
    }


class SensorTests(unittest.TestCase):
    def test_read_only_packets_and_telemetry(self):
        fake = FakeSerial(all_replies())
        reader = stream_sensors.XGOReader(fake, timeout=0.02)
        with patch.object(stream_sensors, "read_cpu_temp", return_value=42.0):
            report = stream_sensors.sample(reader)
        self.assertEqual(report["battery_pct"], 76)
        self.assertEqual(report["attitude_deg"], {"roll": 3.5, "pitch": -4.25, "yaw": 88.0})
        self.assertEqual(len(report["leg_angles_deg"]), 12)
        self.assertEqual(report["arm_motor_raw"], [128, 128, 128])
        self.assertEqual(report["servos"]["22"]["label"], "front right shoulder (middle)")
        self.assertEqual(report["servos"]["32"]["label"], "rear right shoulder (middle)")
        self.assertEqual(report["servos"]["51"]["label"], "gripper")
        self.assertEqual(report["servos"]["52"]["angle_deg"], round(128 / 255 * 130 - 70, 2))
        self.assertEqual(report["servos"]["53"]["angle_deg"], round(128 / 255 * 195 - 90, 2))
        self.assertEqual(report["pi_cpu_temp_c"], 42.0)
        self.assertEqual(report["errors"], {})
        self.assertEqual([packet[4] for packet in fake.writes], [1, 0x62, 0x63, 0x64, 0x50])
        self.assertIn("battery=76%", stream_sensors.format_sample(report))

    def test_calibration_endpoints_and_missing_arm(self):
        for profile, limits in stream_sensors.SERVO_PROFILES.items():
            for raw, end in ((0, 0), (255, 1)):
                decoded = stream_sensors.decode_servos(bytes((raw,)) * 15, profile)
                self.assertEqual(len(decoded), 15)
                for i, motor_id in enumerate(stream_sensors.SERVO_IDS):
                    self.assertEqual(decoded[motor_id]["angle_deg"], limits[i][end])
        decoded = stream_sensors.decode_servos(bytes((128,)) * 12, "lite-2023")
        for motor_id in ("51", "52", "53"):
            self.assertIsNone(decoded[motor_id]["angle_deg"])
            self.assertIsNone(decoded[motor_id]["raw"])
        with self.assertRaises(ValueError):
            stream_sensors.decode_servos(bytes(14), "lite-2023")

    def test_failed_reads_do_not_reuse_old_joint_values(self):
        reader = stream_sensors.XGOReader(FakeSerial({}), timeout=0.002)
        with patch.object(stream_sensors, "read_cpu_temp", return_value=None):
            report = stream_sensors.sample(reader)
        self.assertEqual(len(report["errors"]), 5)
        self.assertTrue(all(s["angle_deg"] is None for s in report["servos"].values()))
        self.assertIn("N/A", stream_sensors.format_sample(report))

    def test_twelve_motor_reply_is_supported(self):
        fake = FakeSerial({**all_replies(), stream_sensors.MOTOR_ANGLES: bytes((128,)) * 12})
        report = stream_sensors.sample(stream_sensors.XGOReader(fake, timeout=0.01))
        self.assertEqual(report["errors"], {})
        self.assertEqual(len(report["leg_angles_deg"]), 12)
        self.assertIsNone(report["servos"]["51"]["angle_deg"])

    def test_log_appends_json_even_with_human_output(self):
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / "readings.jsonl"
            log.write_text('{"previous":true}\n', encoding="utf-8")
            output = io.StringIO()
            with patch.object(stream_sensors, "require_free_port"), \
                    patch.object(stream_sensors.serial, "Serial", return_value=contextlib.nullcontext(FakeSerial(all_replies()))), \
                    contextlib.redirect_stdout(output):
                self.assertEqual(stream_sensors.main(["--count", "1", "--log", str(log)]), 0)
            rows = [json.loads(line) for line in log.read_text().splitlines()]
            self.assertEqual(len(rows), 2)
            self.assertEqual(rows[1]["servos"]["22"]["label"], "front right shoulder (middle)")
            self.assertIn("front right shoulder", output.getvalue())

    def test_single_all_failed_sample_exits_with_error(self):
        with patch.object(stream_sensors, "require_free_port"), \
                patch.object(stream_sensors.serial, "Serial", return_value=contextlib.nullcontext(FakeSerial({}))), \
                patch.object(stream_sensors, "sample", return_value={"errors": dict.fromkeys(range(5), "timeout")}), \
                contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(stream_sensors.main(["--json", "--count", "1"]), 2)

    def test_bad_checksum_and_wrong_address_timeout(self):
        fake = FakeSerial(all_replies(), corrupt=True)
        reader = stream_sensors.XGOReader(fake, timeout=0.01)
        with self.assertRaises(TimeoutError):
            reader.read_register(stream_sensors.BATTERY, 1)
        fake = FakeSerial({stream_sensors.BATTERY: bytes((50,))})
        reader = stream_sensors.XGOReader(fake, timeout=0.01)
        with self.assertRaises(TimeoutError):
            reader.read_register(stream_sensors.ROLL, 4)

    def test_failed_battery_is_not_reported_as_zero_percent(self):
        fake = FakeSerial({**all_replies(), stream_sensors.BATTERY: bytes((0,))})
        with patch.object(stream_sensors, "read_cpu_temp", return_value=None):
            report = stream_sensors.sample(stream_sensors.XGOReader(fake, timeout=0.01))
        self.assertIsNone(report["battery_pct"])
        self.assertIn("battery", report["errors"])
        self.assertEqual(report["leg_angles_deg"]["11"], round(128 / 255 * 120 - 70, 2))

    def test_busy_port_fails_before_opening_uart(self):
        with patch.object(stream_sensors.subprocess, "run") as fuser, \
                patch.object(stream_sensors.serial, "Serial") as uart, \
                patch.object(stream_sensors.os, "geteuid", return_value=1000), \
                contextlib.redirect_stderr(io.StringIO()):
            fuser.return_value.returncode = 0
            fuser.return_value.stdout = b"0\n"
            self.assertEqual(stream_sensors.main(["--count", "1"]), 2)
        uart.assert_not_called()
        self.assertEqual(fuser.call_args_list[0].args[0], ["sudo", "-n", "id", "-u"])
        self.assertEqual(fuser.call_args_list[1].args[0],
                         ["sudo", "-n", "fuser", "-s", "/dev/ttyAMA0"])

    def test_missing_sudo_access_fails_closed(self):
        with patch.object(stream_sensors.subprocess, "run") as process, \
                patch.object(stream_sensors.os, "geteuid", return_value=1000):
            process.return_value.returncode = 1
            with self.assertRaises(RuntimeError):
                stream_sensors.require_free_port("/dev/ttyAMA0")
        self.assertEqual(process.call_count, 1)

    def test_json_output_one_sample(self):
        fake = FakeSerial(all_replies())
        port = contextlib.nullcontext(fake)
        output = io.StringIO()
        with patch.object(stream_sensors, "require_free_port"), \
                patch.object(stream_sensors.serial, "Serial", return_value=port), \
                patch.object(stream_sensors, "read_cpu_temp", return_value=None), \
                contextlib.redirect_stdout(output):
            self.assertEqual(stream_sensors.main(["--json", "--count", "1"]), 0)
        self.assertEqual(json.loads(output.getvalue())["battery_pct"], 76)


if __name__ == "__main__":
    unittest.main()
