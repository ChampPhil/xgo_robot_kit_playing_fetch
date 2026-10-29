"""One camera + one UART owner. Used only after a fully taught profile passes preflight."""

import hashlib
import json
import sys
import threading
import time
from pathlib import Path

from vision import (InventoryTracker, Stable, features, marker_matches,
                    pickup_verdict, tower_verdict)
from workflow import Stopped

sys.path[:0] = [str(Path(__file__).resolve().parents[1] / name)
                for name in ("teleop", "sensor_monitor")]
from align import GyroAligner, Observer, alignment_legs_driver  # noqa: E402
from body import ArmController, Driver, LegStepper, PosePreparer, Posture  # noqa: E402
from camera import Camera  # noqa: E402
from heading import HeadingDriver, HeadingHold, make_yaw_reader  # noqa: E402
from stream_sensors import BATTERY, MOTOR_ANGLES, XGOReader, require_free_port  # noqa: E402
from teach import Replayer, laplacian_sharpness  # noqa: E402
from teleop_receiver import build_parser  # noqa: E402


class Heartbeat:
    def __init__(self, stream, clock=time.monotonic):
        self.clock, self.last, self.error = clock, None, None
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self.read, args=(stream,), daemon=True)
        self.thread.start()

    def read(self, stream):
        try:
            for line in stream:
                message = json.loads(line)
                if not isinstance(message, dict) or set(message) != {"heartbeat"} or message["heartbeat"] is not True:
                    raise ValueError("invalid heartbeat")
                with self.lock:
                    self.last = self.clock()
        except (OSError, ValueError) as exc:
            self.error = str(exc)
        finally:
            self.error = self.error or "heartbeat stream ended"

    def check(self):
        with self.lock:
            if self.error or self.last is None or self.clock() - self.last > 0.5:
                raise Stopped(self.error or "no heartbeat for 0.5 s")

    def wait(self):
        deadline = self.clock() + 8
        while self.last is None and not self.error and self.clock() < deadline:
            time.sleep(0.02)
        self.check()  # before creating XGO, whose constructor moves the dog


class Robot:
    def __init__(self, plan, perception, heartbeat, log, workdir):
        self.plan, self.perception, self.heartbeat, self.log = plan, perception, heartbeat, log
        self.workdir = Path(workdir).expanduser()
        self.workdir.mkdir(parents=True, exist_ok=True)
        self.driver = self.dog = self.camera = None
        heartbeat.wait()
        require_free_port("/dev/ttyAMA0")
        require_free_port("/dev/video0")
        import serial

        with serial.Serial('/dev/ttyAMA0', 115200, timeout=0.01, exclusive=True) as connection:
            battery = XGOReader(connection, timeout=0.15).read_register(BATTERY, 1)[0]
        if not 20 <= battery <= 100:
            raise Stopped("battery invalid or below 20%; constructor not called")
        self.camera = Camera(0)
        self.camera.start()  # validate camera before any motor constructor/reset
        try:
            heartbeat.check()
            require_free_port("/dev/ttyAMA0")  # recheck after camera startup / battery read
            import xgolib

            self.log({"state": "startup", "battery_pct": battery,
                      "note": "XGO constructor resets and stands the dog"})
            self.dog = xgolib.XGO(port="/dev/ttyAMA0", version="xgolite")
            self.args = build_parser().parse_args([])
            base = Driver(self.dog, self.args)
            self.hold = HeadingHold(make_yaw_reader(self.dog.ser), time.monotonic, log=self.say,
                                    sign_path=self.workdir / "heading.json")
            self.driver = HeadingDriver(base, self.hold)
            self.arm = ArmController(self.dog, self.args)
            self.posture = Posture(self.dog, self.args)
            self.reader = XGOReader(self.dog.ser, timeout=0.1)
            self.started = self.last_service = time.monotonic()
            self.battery_at = self.started
            self.not_before = self.started
            self.replayer = Replayer(self.arm, self.posture, driver=self.driver,
                                     max_gap=0)  # preserve validated route timing exactly
        except BaseException:
            self.close()
            raise

    def say(self, message):
        self.log({"state": "controller", "message": message})

    def read_register(self, address, count):
        saved = self.dog.ser.timeout
        try:
            self.dog.ser.timeout = 0.01
            return self.reader.read_register(address, count)
        finally:
            self.dog.ser.timeout = saved

    def service(self, external=False):
        now = time.monotonic()
        self.heartbeat.check()
        if now - self.last_service > 0.5:
            raise Stopped("control loop stalled for >0.5 s")
        self.last_service = now
        if now - self.started > 600:
            raise Stopped("600 s tower session limit")
        _, stamp = self.camera.latest()
        if stamp is None or now - stamp > 0.75:
            raise Stopped("camera stale; stopping even during replay")
        if now - self.battery_at >= 5:
            self.battery_at = now
            try:
                battery = self.read_register(BATTERY, 1)[0]
            except (OSError, ValueError, TimeoutError) as exc:
                raise Stopped("battery read failed: " + str(exc))
            if not 20 <= battery <= 100:
                raise Stopped("battery invalid or below 20%")
        self.driver.refresh(external_motion=external)

    def motion(self, motion, watch=None):
        """Replay a motion. watch=color: while carrying it, stop and return "floor" as soon as
        it is seen on the floor (3 fresh sharp frames in a row; one flicker is not a drop)."""
        self.heartbeat.check()
        self.replayer.start(motion)
        deadline = time.monotonic() + 65
        sightings, last = 0, None
        try:
            while self.replayer.active:
                self.service()
                if time.monotonic() > deadline:
                    raise Stopped("motion deadline exceeded")
                if watch is not None:
                    frame, stamp = self.camera.latest()
                    if stamp != last and stamp >= self.not_before:
                        last = stamp
                        if laplacian_sharpness(frame) >= 50:
                            scene = self.perception.observe(frame)
                            seen = pickup_verdict(scene, watch, self.plan.verify_floor,
                                                  None) == "floor"
                            sightings = sightings + 1 if seen else 0
                            if sightings >= 3:
                                self.log({"state": "evidence", "result": "floor",
                                          "key": "dropped while carrying " + watch})
                                return "floor"
                self.replayer.tick()
                time.sleep(0.03)
        finally:
            self.replayer.abort()
            self.driver.stop()
            self.not_before = time.monotonic() + 0.7
        return None

    def observe_until(self, evaluate, seconds=6):
        self.driver.stop()
        stable, deadline = Stable(3), time.monotonic() + seconds
        previous_key = object()
        while time.monotonic() < deadline:
            self.service()
            frame, stamp = self.camera.latest()
            if stamp >= self.not_before and laplacian_sharpness(frame) >= 50:
                scene = self.perception.observe(frame)
                scene["stamp"] = stamp
                result, key = evaluate(scene)
                if key != previous_key:
                    self.log({"state": "evidence", "result": result, "key": key})
                    previous_key = key
                if stable.update(key, stamp):
                    return result
            time.sleep(0.04)
        raise Stopped("no stable, fresh verification evidence before deadline")

    def survey(self, remaining):
        self.motion(self.plan.survey)
        tracker = InventoryTracker(remaining, self.plan.floor, self.plan.exclude)

        def evaluate(scene):
            found = tracker.update(scene, scene["stamp"])
            if not found["ready"]:
                return found, None
            key = tuple((color, found["tracks"][color]["id"]) for color in remaining)
            return found, key
        self.observe_until(evaluate)

    def align(self, reference, target, calibration_name):
        reference_key = json.dumps({"reference": reference, "marker_id": self.plan.marker_id}, sort_keys=True)
        digest = hashlib.sha256(reference_key.encode()).hexdigest()[:12]
        def measure(frame):
            scene = self.perception.observe(frame)
            if target == "marker":
                value = scene["marker"]
            else:
                found = scene["boxes"][target]
                value = features(found[0]) if len(found) == 1 else None
            return (value, None) if value else (None, "missing or ambiguous " + target)

        aligner = GyroAligner(
            LegStepper(alignment_legs_driver(self.driver), {"turn": 40, "forward": 12}),
            PosePreparer(self.arm, self.posture),
            Observer(self.camera, measure, laplacian_sharpness, time.monotonic),
            make_yaw_reader(self.dog.ser), lambda: reference,
            self.workdir / (calibration_name + "-" + digest + ".json"), self.say, time.monotonic,
            motion=True, drift=lambda: self.hold.bias, gyro_sign_learned=self.hold.adopt_sign)
        deadline = time.monotonic() + 65  # also bounds initial posture search/calibration
        try:
            while True:
                self.service(external=aligner.moving is not None)
                active = aligner.update(True, False)
                if aligner.phase == "aligned":
                    break
                if not active or time.monotonic() >= deadline:
                    raise Stopped("alignment did not finish for " + target)
                time.sleep(0.03)
        finally:
            aligner.update(False, False)
            self.driver.stop()
            self.not_before = time.monotonic() + 0.3

    def pick(self, color):
        entry = self.plan.pickups[color]
        self.arm.set_claw(0)  # only reached initially empty, or after confirmed floor failure
        self.align(entry["reference"], color, "pick-" + color)
        self.motion(entry["motion"])

    def grip_verdict(self, color):
        def evaluate(scene):
            grip = None
            if self.plan.grip_profile is not None:
                grip = self.read_register(MOTOR_ANGLES, 15)[12]
            verdict = pickup_verdict(scene, color, self.plan.verify_floor,
                                     self.plan.verify_held, grip, self.plan.grip_profile)
            return verdict, verdict
        return self.observe_until(evaluate)

    def verify_pick(self, color):
        if self.motion(self.plan.check, watch=color) == "floor":
            return "floor"
        return self.grip_verdict(color)

    def carry(self, color, layer):
        entry = self.plan.layers[layer]
        if self.motion(entry["to_stack"], watch=color) == "floor":
            return "floor"
        if self.motion(self.plan.check, watch=color) == "floor":
            return "floor"
        verdict = self.grip_verdict(color)
        if verdict != "held":
            return verdict
        self.align(entry["reference"], "marker", "stack-" + str(layer))
        # The profile enforces the same held-verification pose at marker alignment.
        self.observe_until(lambda scene: (
            scene, "marker_match" if marker_matches(scene["marker"], entry["reference"]["features"])
            else None))
        return self.grip_verdict(color)

    def place(self, color, layer):
        self.motion(self.plan.layers[layer]["place"])

    def verify_place(self, expected, layer):
        entry = self.plan.layers[layer]
        self.motion(entry["inspect"])
        return self.observe_until(lambda scene: (
            tower_verdict(scene, expected, entry["stack_roi"], entry["held_roi"],
                          entry["inspect_reference"]["features"]),
            tower_verdict(scene, expected, entry["stack_roi"], entry["held_roi"],
                          entry["inspect_reference"]["features"])))

    def recover(self, color, layer, origin):
        self.arm.set_claw(0)  # a stable floor verdict positively identified the failed box
        motion = (self.plan.pickups[color]["retry"] if origin == "pickup" else
                  self.plan.layers[layer]["recover_drop"])
        self.motion(motion)

    def return_to_survey(self, layer):
        self.motion(self.plan.layers[layer]["return"])

    def stop(self):
        if self.driver is not None:
            self.driver.stop()
        elif self.dog is not None:
            self.dog.stop()

    def close(self):
        try:
            self.stop()
        finally:
            try:
                if self.camera is not None:
                    self.camera.close()
            finally:
                if self.dog is not None:
                    self.dog.ser.close()
