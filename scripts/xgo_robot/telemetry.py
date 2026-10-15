"""
Battery, servo angles, and IMU over the XGO serial port.

Purpose:
    One background thread owns ``/dev/ttyAMA0``. The stats panel polls
    ``snapshot()``. Chase sends walk commands through ``command()`` on the
    same lock so two ``XGO()`` objects never share the UART.

Key terms:
    - UART / serial: The wire from the Pi to the motion controller.
    - IMU: Inertial measurement unit. Acceleration, gyro, roll/pitch/yaw.
    - command(): Run a callable with the live dog while holding the serial lock.

Exports:
    - XgoTelemetryPoller.
"""

from __future__ import annotations

import json
import threading
import time
from datetime import datetime

from .models import (
    DEFAULT_TELEMETRY_HZ,
    DEFAULT_XGO_PORT,
    SERVO_IDS,
    SERVO_LABELS,
)


def _xgo_read(dog: object, method_name: str) -> object:
    """Call an XGO read method whether xgolib exposes it as a function or property.

    :param dog: Connected ``XGO`` instance from xgolib.
    :param method_name: Name such as ``read_battery`` or ``read_motor``.
    :returns: Whatever the library returns for that reading.
    """

    attr = getattr(dog, method_name)
    if callable(attr):
        return attr()
    return attr


def _imu_from_dog(dog: object) -> dict[str, dict[str, float]]:
    """Read acceleration, gyroscope, and Euler rotation from the onboard IMU.

    :param dog: Connected ``XGO`` instance.
    :returns: Nested dict with ``acceleration``, ``gyroscope``, and ``rotation``.
    """

    acceleration = {"x": 0.0, "y": 0.0, "z": 0.0}
    gyroscope = {"x": 0.0, "y": 0.0, "z": 0.0}
    rotation = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
    have_rotation = False
    try:
        raw_imu = _xgo_read(dog, "read_imu")
        if isinstance(raw_imu, dict):
            acc = raw_imu.get("acceleration", (0, 0, 0))
            gyro = raw_imu.get("gyroscope", (0, 0, 0))
            euler = raw_imu.get("euler")
        else:
            acc = getattr(raw_imu, "acceleration", (0, 0, 0))
            gyro = getattr(raw_imu, "gyroscope", (0, 0, 0))
            euler = getattr(raw_imu, "euler", None)
        acceleration = {"x": float(acc[0]), "y": float(acc[1]), "z": float(acc[2])}
        gyroscope = {"x": float(gyro[0]), "y": float(gyro[1]), "z": float(gyro[2])}
        if euler is not None:
            roll, pitch, yaw = euler
            if max(abs(float(roll)), abs(float(pitch)), abs(float(yaw))) <= 6.5:
                roll = float(roll) * 57.2957795
                pitch = float(pitch) * 57.2957795
                yaw = float(yaw) * 57.2957795
            rotation = {"roll": float(roll), "pitch": float(pitch), "yaw": float(yaw)}
            have_rotation = True
    except (AttributeError, IndexError, TypeError, ValueError):
        pass

    if not have_rotation:
        rotation = {
            "roll": float(_xgo_read(dog, "read_roll")),
            "pitch": float(_xgo_read(dog, "read_pitch")),
            "yaw": float(_xgo_read(dog, "read_yaw")),
        }
    return {
        "acceleration": acceleration,
        "gyroscope": gyroscope,
        "rotation": rotation,
    }


def _servos_from_motor_list(motor_angles: list[float] | tuple[float, ...]) -> list[dict[str, object]]:
    """Pair raw ``read_motor()`` values with human-readable joint names.

    :param motor_angles: List returned by ``read_motor()`` (12 or 15 values).
    :returns: One dict per joint with ``id``, ``name``, and ``angle_deg``.
    """

    servos: list[dict[str, object]] = []
    for index, angle in enumerate(motor_angles):
        if index >= len(SERVO_IDS):
            break
        servo_id = SERVO_IDS[index]
        servos.append(
            {
                "id": servo_id,
                "name": SERVO_LABELS.get(servo_id, f"Servo {servo_id}"),
                "angle_deg": float(angle),
            }
        )
    return servos


class XgoTelemetryPoller:
    """Poll battery, servos, and IMU over serial without blocking the camera loop.

    :param port: Serial device for the XGO controller.
    :param poll_hz: How many telemetry samples to attempt per second.
    """

    def __init__(self, port: str = DEFAULT_XGO_PORT, poll_hz: float = DEFAULT_TELEMETRY_HZ) -> None:
        self._port = port
        self._interval = 1.0 / max(poll_hz, 0.5)
        self._lock = threading.Lock()
        self._serial_lock = threading.Lock()
        self._dog: object | None = None
        self._stop = threading.Event()
        self._snapshot: dict[str, object] = {
            "ok": False,
            "error": "Connecting to robot…",
            "battery_percent": 0,
            "servos": [],
            "imu": {
                "acceleration": {"x": 0.0, "y": 0.0, "z": 0.0},
                "gyroscope": {"x": 0.0, "y": 0.0, "z": 0.0},
                "rotation": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
            },
        }
        self._thread = threading.Thread(target=self._loop, name="xgo-telemetry", daemon=True)
        self._thread.start()

    def snapshot(self) -> dict[str, object]:
        """Return a JSON-serializable copy of the latest robot readings.

        :returns: Dict consumed by the browser panel and ``GET /stats``.
        """

        with self._lock:
            return json.loads(json.dumps(self._snapshot))

    def wait_until_connected(self, timeout: float = 8.0) -> bool:
        """Block until xgolib opens, or until ``timeout`` seconds elapse.

        :param timeout: Seconds to wait for a live ``XGO`` handle.
        :returns: True if the dog handle is set.
        """

        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._dog is not None:
                return True
            snap = self.snapshot()
            error = str(snap.get("error") or "")
            if "not installed" in error.lower() or "could not open" in error.lower():
                return False
            time.sleep(0.1)
        return self._dog is not None

    def command(self, fn: object) -> object:
        """Run ``fn(dog)`` while holding the serial lock.

        :param fn: Callable that takes the ``XGO`` instance.
        :returns: Whatever ``fn`` returns, or None if the dog is not connected.
        """

        with self._serial_lock:
            if self._dog is None:
                return None
            return fn(self._dog)

    def close(self) -> None:
        """Stop the polling thread.

        :returns: None.
        """

        self._stop.set()
        self._thread.join(timeout=2.0)

    def _loop(self) -> None:
        """Connect to xgolib once, then refresh readings until ``close``.

        :returns: None.
        """

        try:
            from xgolib import XGO
        except ImportError:
            self._set_error("xgolib is not installed. On the Pi run: pip3 install xgolib")
            return

        try:
            dog = XGO("xgolite", port=self._port)
        except TypeError:
            dog = XGO(port=self._port, version="xgolite")
        except OSError as exc:
            self._set_error(f"Could not open {self._port}: {exc}")
            return

        self._dog = dog
        print(f"Telemetry connected on {self._port}.")

        while not self._stop.is_set():
            try:
                with self._serial_lock:
                    battery_raw = _xgo_read(dog, "read_battery")
                    battery = int(battery_raw) if battery_raw is not None else 0
                    motor_raw = _xgo_read(dog, "read_motor")
                    imu = _imu_from_dog(dog)
                servos: list[dict[str, object]] = (
                    [] if not motor_raw else _servos_from_motor_list(list(motor_raw))
                )
                payload = {
                    "ok": True,
                    "battery_percent": max(0, min(100, battery)),
                    "servos": servos,
                    "imu": imu,
                    "updated_at": datetime.now().isoformat(timespec="seconds"),
                }
                with self._lock:
                    self._snapshot = payload
            except Exception as exc:
                self._set_error(f"Telemetry read failed: {exc}")
            time.sleep(self._interval)

    def _set_error(self, message: str) -> None:
        """Store a failure message for the stats panel.

        :param message: Plain-English explanation shown in the browser.
        :returns: None.
        """

        with self._lock:
            self._snapshot = {
                "ok": False,
                "error": message,
                "battery_percent": 0,
                "servos": [],
                "imu": {
                    "acceleration": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "gyroscope": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "rotation": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
                },
            }
