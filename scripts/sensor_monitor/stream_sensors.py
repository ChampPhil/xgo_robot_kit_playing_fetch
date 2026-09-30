"""Stream read-only XGO Lite telemetry over UART; no motor commands are sent."""

import argparse
import json
import math
import os
import struct
import subprocess
import sys
import time
from contextlib import ExitStack
from datetime import datetime
from pathlib import Path

import serial  # type: ignore[import-not-found]

BAUD = 115200
BATTERY = 0x01
MOTOR_ANGLES = 0x50
ROLL, PITCH, YAW = 0x62, 0x63, 0x64
LEG_IDS = tuple(f"{leg}{joint}" for leg in range(1, 5) for joint in range(1, 4))
SERVO_IDS = (*LEG_IDS, "51", "52", "53")
# Leg digits: 1 LF, 2 RF, 3 RR, 4 LR; joint digits: lower, middle, upper.
SERVO_LABELS = {
    f"{leg}{joint}": f"{location} {part}"
    for leg, location in ((1, "front left"), (2, "front right"),
                          (3, "rear right"), (4, "rear left"))
    for joint, part in ((1, "elbow (lower)"), (2, "shoulder (middle)"), (3, "hip (upper)"))
}
SERVO_LABELS.update({"51": "gripper", "52": "arm distal joint", "53": "arm proximal joint"})
LEG_LIMITS = ((-70, 50), (-70, 90), (-30, 30))
# Register bytes are normalized joint coordinates, not universal absolute servo angles.
SERVO_PROFILES = {
    "lite-2023": (*LEG_LIMITS * 4, (-65, 65), (-70, 60), (-90, 105)),
    "lite-pypi-1.1.10": (*LEG_LIMITS * 4, (-65, 65), (-115, 70), (-85, 100)),
}


def positive_int(value):
    try:
        result = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("expected a positive integer") from exc
    if result < 1:
        raise argparse.ArgumentTypeError("expected a positive integer")
    return result


def bounded_float(value):
    try:
        result = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("expected a number of seconds") from exc
    if not math.isfinite(result) or not 0.2 <= result <= 60:
        raise argparse.ArgumentTypeError("interval must be between 0.2 and 60 seconds")
    return result


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyAMA0", help="serial port (default: /dev/ttyAMA0)")
    parser.add_argument("--interval", type=bounded_float, default=1.0, help="seconds between samples (0.2–60; default 1)")
    parser.add_argument("--count", type=positive_int, help="stop after N samples; otherwise run until Ctrl+C")
    parser.add_argument("--json", action="store_true", help="print one JSON object per sample")
    parser.add_argument("--log", type=Path, help="append timestamped JSON samples to this file")
    parser.add_argument("--servo-profile", choices=tuple(SERVO_PROFILES), default="lite-2023",
                        help="readback conversion profile (default: lite-2023, installed robot library)")
    return parser


class XGOReader:
    """Minimal XGO read protocol; avoids xgolib.XGO(), whose constructor resets the dog."""

    def __init__(self, connection, timeout=0.4):
        self.connection = connection
        self.timeout = timeout

    def read_register(self, address, count, allowed_lengths=None):
        if not 0 <= address <= 255 or not 1 <= count <= 32:
            raise ValueError("invalid query address or length")
        check = 255 - (9 + 2 + address + count) % 256
        packet = bytes((0x55, 0x00, 9, 2, address, count, check, 0, 0xAA))
        self.connection.reset_input_buffer()
        self.connection.write(packet)  # protocol mode 2 = READ (never actuator mode 1)

        lengths = (count,) if allowed_lengths is None else allowed_lengths
        deadline = time.monotonic() + self.timeout
        frame = bytearray()
        while time.monotonic() < deadline:
            byte = self.connection.read(1)
            if not byte:
                continue
            value = byte[0]
            if not frame:
                if value == 0x55:
                    frame.append(value)
            elif len(frame) == 1:
                if value == 0:
                    frame.append(value)
                else:
                    frame = bytearray((0x55,)) if value == 0x55 else bytearray()
            else:
                frame.append(value)
                if len(frame) == 3 and not 9 <= frame[2] <= 48:
                    frame.clear()
                elif len(frame) >= 3 and len(frame) == frame[2]:
                    payload = frame[5:-3]
                    checksum = 255 - (frame[2] + frame[3] + frame[4] + sum(payload)) % 256
                    if (frame[4] == address and len(payload) in lengths
                            and frame[-3:] == bytes((checksum, 0, 0xAA))):
                        return bytes(payload)
                    frame.clear()
        raise TimeoutError(f"no valid reply from register 0x{address:02x}")


def read_cpu_temp():
    try:
        return round(int(Path("/sys/class/thermal/thermal_zone0/temp").read_text().strip()) / 1000, 1)
    except (OSError, ValueError):
        return None


def decode_servos(data, profile):
    """Return all named joints, explicitly marking missing arm telemetry as unavailable."""
    if len(data) not in (12, 15):
        raise ValueError("expected 12 or 15 servo readback bytes")
    readings = {}
    for i, motor_id in enumerate(SERVO_IDS):
        raw = data[i] if i < len(data) else None
        lo, hi = SERVO_PROFILES[profile][i]
        readings[motor_id] = {
            "label": SERVO_LABELS[motor_id],
            "angle_deg": round(raw / 255 * (hi - lo) + lo, 2) if raw is not None else None,
            "raw": raw,
        }
    return readings


def sample(reader, profile="lite-2023"):
    """Read the available telemetry; never retain values from a previous sample."""
    result = {
        "timestamp": datetime.now().astimezone().isoformat(timespec="seconds"),
        "battery_pct": None,
        "attitude_deg": {"roll": None, "pitch": None, "yaw": None},
        "leg_angles_deg": None,
        "arm_motor_raw": None,
        "servo_profile": profile,
        "servos": {motor_id: {"label": SERVO_LABELS[motor_id], "angle_deg": None, "raw": None}
                   for motor_id in SERVO_IDS},
        "pi_cpu_temp_c": read_cpu_temp(),
        "errors": {},
    }
    for name, address, count in (
        ("battery", BATTERY, 1),
        ("roll", ROLL, 4),
        ("pitch", PITCH, 4),
        ("yaw", YAW, 4),
        ("motors", MOTOR_ANGLES, 15),
    ):
        try:
            data = reader.read_register(address, count, allowed_lengths=(12, 15) if name == "motors" else None)
            if name == "battery":
                if not 1 <= data[0] <= 100:
                    raise ValueError("battery returned 0 or an invalid percentage")
                result["battery_pct"] = data[0]
            elif name == "motors":
                result["servos"] = decode_servos(data, profile)
                result["leg_angles_deg"] = {
                    motor_id: result["servos"][motor_id]["angle_deg"] for motor_id in LEG_IDS
                }
                result["arm_motor_raw"] = list(data[12:]) if len(data) == 15 else None
            else:
                angle = struct.unpack("<f", data)[0]
                if not math.isfinite(angle):
                    raise ValueError("non-finite angle")
                result["attitude_deg"][name] = round(angle, 2)
        except (TimeoutError, ValueError, struct.error, serial.SerialException) as exc:
            result["errors"][name] = str(exc)
    return result


def format_sample(reading):
    angles = reading["attitude_deg"]
    attitude = " ".join(f"{name}={angles[name]:.1f}°" if angles[name] is not None else f"{name}=?"
                        for name in ("roll", "pitch", "yaw"))
    battery = f"{reading['battery_pct']}%" if reading["battery_pct"] is not None else "?"
    cpu = f"{reading['pi_cpu_temp_c']}°C" if reading["pi_cpu_temp_c"] is not None else "?"
    lines = [f"\n{reading['timestamp']} battery={battery} {attitude} Pi_CPU={cpu}",
             f"Servo readback estimates ({reading['servo_profile']}):",
             "  ID  Joint                           Degrees   Raw"]
    for motor_id, servo in reading["servos"].items():
        angle = f"{servo['angle_deg']:.2f}" if servo["angle_deg"] is not None else "N/A"
        raw = str(servo["raw"]) if servo["raw"] is not None else "N/A"
        lines.append(f"  {motor_id}  {servo['label']:<30} {angle:>8} {raw:>5}")
    if reading["battery_pct"] is not None and reading["battery_pct"] <= 10:
        lines.append("  LOW BATTERY: do not start motor tests; charge before operating.")
    if reading["errors"]:
        lines.append(f"  Read errors: {reading['errors']}")
    return "\n".join(lines)


def require_free_port(port):
    """Check as root: the built-in menu runs as root and is hidden from pi's fuser."""
    try:
        command = ["fuser", "-s", port]
        if os.geteuid() != 0:
            authorized = subprocess.run(["sudo", "-n", "id", "-u"], check=False,
                                        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
            if authorized.returncode != 0 or authorized.stdout.strip() != b"0":
                raise RuntimeError("sudo access is needed to inspect root-owned UART clients")
            command = ["sudo", "-n", *command]
        status = subprocess.run(command, check=False, stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL).returncode
    except FileNotFoundError as exc:
        raise RuntimeError("sudo and fuser are required to check serial-port ownership") from exc
    if status == 0:
        raise RuntimeError(f"{port} is already in use; stop the built-in menu/other UART clients first")
    if status != 1:
        raise RuntimeError(f"cannot verify whether {port} is in use (fuser exit {status})")


def main(argv=None):
    args = build_parser().parse_args(argv)
    try:
        require_free_port(args.port)
        # Open the log before touching UART; close both on errors or interruption.
        with ExitStack() as stack:
            log = stack.enter_context(args.log.open("a", encoding="utf-8")) if args.log else None
            connection = stack.enter_context(serial.Serial(args.port, BAUD, timeout=0.1, exclusive=True))
            reader = XGOReader(connection)
            failures = 0
            count = 0
            any_uart_data = False
            while args.count is None or count < args.count:
                start = time.monotonic()
                reading = sample(reader, args.servo_profile)
                serialized = json.dumps(reading, allow_nan=False)
                print(serialized if args.json else format_sample(reading), flush=True)
                if log is not None:
                    log.write(serialized + "\n")
                    log.flush()
                count += 1
                all_failed = len(reading["errors"]) == 5
                any_uart_data = any_uart_data or not all_failed
                failures = failures + 1 if all_failed else 0
                if failures >= 2:
                    print("All UART queries failed twice; stopping to avoid stale telemetry.", file=sys.stderr)
                    return 2
                if args.count is not None and count >= args.count:
                    break
                time.sleep(max(0, args.interval - (time.monotonic() - start)))
            if not any_uart_data:
                return 2
    except KeyboardInterrupt:
        print("\nStopped (serial port closed).", file=sys.stderr)
    except (OSError, RuntimeError, serial.SerialException) as exc:
        print(f"Sensor stream unavailable: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
