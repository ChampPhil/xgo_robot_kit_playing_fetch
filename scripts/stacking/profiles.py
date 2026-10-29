"""Load taught workcell assets completely before any robot constructor is called."""

import copy
import json
import math
import sys
from pathlib import Path

from vision import COLORS, features, overlaps, roi

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "teleop"))
from body import ARM_LIMITS, in_reach  # noqa: E402
from teach import load_motion  # noqa: E402


def number(value, low, high, label):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or not low <= value <= high):
        raise ValueError(label + " is outside its finite allowed range")
    return value


def pose(value):
    if not isinstance(value, dict) or set(value) != {"arm", "kneel"}:
        raise ValueError("reference state must specify arm and kneel (no claw command)")
    arm = value["arm"]
    if not isinstance(arm, list) or len(arm) != 2:
        raise ValueError("teach an explicit arm pose; unknown/null arm starts are not accepted")
    x = number(arm[0], *ARM_LIMITS["x"], "arm x")
    z = number(arm[1], *ARM_LIMITS["z"], "arm z")
    if not in_reach(x, z):
        raise ValueError("arm pose is outside the measured reach ring")
    number(value["kneel"], 0, 1, "kneel")
    return value


def read_motion(path, role):
    motion = copy.deepcopy(load_motion(Path(path).parent)) if Path(path).name == "motion.json" else \
        json.loads(Path(path).read_text())
    if not isinstance(motion, dict) or motion.get("version") != 1 or not isinstance(motion.get("start"), dict):
        raise ValueError("motion must use teach/replay version 1")
    start = motion["start"]
    if set(start) - {"arm", "kneel", "claw"}:
        raise ValueError("unknown motion start key")
    pose({"arm": start.get("arm"), "kneel": start.get("kneel")})
    number(motion.get("duration"), 0, 60, "motion duration")
    events = motion.get("events")
    if not isinstance(events, list) or not events or len(events) > 1000:
        raise ValueError("motion must have 1..1000 events")
    previous = -1
    releases = 0
    last_walk = {"x": 0, "y": 0, "yaw": 0}
    for event in events:
        if not isinstance(event, dict) or len(event) != 2 or "t" not in event:
            raise ValueError("bad motion event")
        when = number(event["t"], 0, motion["duration"], "event time")
        if when < previous:
            raise ValueError("motion events must already be in time order")
        previous = when
        kind = next(k for k in event if k != "t")
        value = event[kind]
        if kind == "arm":
            pose({"arm": value, "kneel": 0})
        elif kind == "kneel":
            number(value, 0, 1, "kneel")
        elif kind == "walk":
            if not isinstance(value, dict) or set(value) != {"x", "y", "yaw"}:
                raise ValueError("walk needs x/y/yaw")
            for axis, maximum in (("x", 12), ("y", 8), ("yaw", 40)):
                number(value[axis], -maximum, maximum, "walk " + axis)
                if not isinstance(value[axis], int) or isinstance(value[axis], bool):
                    raise ValueError("walk commands must be integers")
            if role in ("check", "inspect", "place") and any(value.values()):
                raise ValueError("verification motions must not walk or turn")
            last_walk = value
        elif kind == "claw":
            number(value, 0, 255, "claw")
            if role not in ("pick", "place"):
                raise ValueError(role + " motion must preserve the grip; remove claw events")
            if role == "place":
                if value != 0:
                    raise ValueError("placement may only release with claw=0")
                releases += 1
        else:
            raise ValueError("unknown motion kind " + kind)
    if any(last_walk.values()):
        raise ValueError("record an explicit leg stop before finishing the motion")
    if start.get("claw") is not None:
        number(start["claw"], 0, 255, "start claw")
    if role == "pick":
        if start.get("claw") != 0 or not any(e.get("claw", 0) > 0 for e in events):
            raise ValueError("pickup must start open and include a close")
        last_claw = next(e["claw"] for e in reversed(events) if "claw" in e)
        if last_claw == 0:
            raise ValueError("pickup ends by opening the claw")
    else:
        # Recorded starts often include a claw setpoint. Preserve the current grasp until
        # the explicit release event, rather than replaying that stale setpoint.
        motion["start"]["claw"] = None
    if role == "place" and releases != 1:
        raise ValueError("placement needs exactly one explicit claw=0 release")
    return motion


def end_pose(motion):
    state = {key: motion["start"][key] for key in ("arm", "kneel")}
    for event in motion["events"]:
        for key in state:
            if key in event:
                state[key] = event[key]
    return state


def start_pose(motion):
    return {key: motion["start"][key] for key in ("arm", "kneel")}


class Plan:
    def __init__(self, filename, perception):
        self.filename = Path(filename).expanduser().resolve()
        self.data = json.loads(self.filename.read_text())
        d = self.data
        if not isinstance(d, dict) or d.get("version") != 1 or d.get("validated") is not True:
            raise ValueError("workcell is not validated; teach/review the example profile first")
        self.marker_id = d["marker_id"]
        if type(self.marker_id) is not int or self.marker_id != perception.marker_id:
            raise ValueError("marker ID does not match the perception dictionary")
        self.floor = roi(d["survey_floor_roi"])
        self.exclude = [roi(r) for r in d.get("survey_exclude_rois", [])]
        self.verify = d["verification"]
        if not isinstance(self.verify, dict) or self.verify.get("validated") is not True:
            raise ValueError("pickup verification view/ROIs have not been validated")
        self.verify_floor = roi(self.verify["floor_roi"])
        self.verify_held = roi(self.verify["held_roi"])
        if overlaps(self.verify_floor, self.verify_held):
            raise ValueError("floor and held ROIs must be disjoint in the taught check pose")
        self.grip_profile = self.verify.get("grip_raw")
        if self.grip_profile is not None:
            bands = []
            for key in ("empty", "occupied"):
                band = self.grip_profile[key]
                if not isinstance(band, list) or len(band) != 2:
                    raise ValueError("grip raw bands need two endpoints")
                number(band[0], 0, 255, key); number(band[1], band[0], 255, key)
                bands.append(band)
            if max(bands[0][0], bands[1][0]) <= min(bands[0][1], bands[1][1]):
                raise ValueError("empty and occupied readback bands overlap")
        self.survey = self.motion(d["survey_motion"], "survey")
        self.check = self.motion(self.verify["motion"], "check")
        self.pickups = {}
        for color in COLORS:
            entry = d["pickups"][color]
            if not isinstance(entry, dict) or entry.get("validated") is not True:
                raise ValueError(color + " pickup/recovery has not been validated")
            self.pickups[color] = {
                "reference": self.reference(entry["reference"], perception, color),
                "motion": self.motion(entry["motion"], "pick"),
                "retry": self.motion(entry["retry"], "recover")}
            if start_pose(self.pickups[color]["motion"]) != self.pickups[color]["reference"]["state"]:
                raise ValueError(color + " pickup start differs from its alignment reference pose")
        if not isinstance(d["layers"], list) or len(d["layers"]) != 3:
            raise ValueError("three separately taught layer profiles are required")
        self.layers = []
        for index, entry in enumerate(d["layers"]):
            if (not isinstance(entry, dict) or entry.get("validated") is not True
                    or isinstance(entry.get("height_mm"), bool) or entry.get("height_mm") != index * 25):
                raise ValueError("teach each layer at nominal heights 0, 25 and 50 mm")
            layer = {key: self.motion(entry[key], role) for key, role in (
                ("to_stack", "carry"), ("place", "place"), ("inspect", "inspect"),
                ("return", "return"), ("recover_drop", "recover"))}
            layer["reference"] = self.reference(entry["reference"], perception, "marker")
            layer["inspect_reference"] = self.reference(
                entry["inspect_reference"], perception, "marker")
            if not (start_pose(layer["place"]) == layer["reference"]["state"] == end_pose(self.check)):
                raise ValueError("place start, marker approach, and held-check pose must match")
            if end_pose(layer["inspect"]) != layer["inspect_reference"]["state"]:
                raise ValueError("tower inspection pose differs from its marker reference")
            layer["stack_roi"], layer["held_roi"] = roi(entry["stack_roi"]), roi(entry["held_roi"])
            if overlaps(layer["stack_roi"], layer["held_roi"]):
                raise ValueError("stack and held verification ROIs must be disjoint")
            self.layers.append(layer)

    def path(self, value):
        path = Path(value).expanduser()
        return path if path.is_absolute() else self.filename.parent / path

    def motion(self, value, role):
        return read_motion(self.path(value), role)

    def reference(self, value, perception, target):
        import cv2

        path = self.path(value)
        data = json.loads(path.read_text())
        state = pose(data["state"])
        image = path.parent / data["image"]
        frame = cv2.imread(str(image))
        if frame is None:
            raise ValueError("cannot read reference image " + str(image))
        scene = perception.observe(frame)
        if target == "marker":
            value = scene["marker"]
        else:
            boxes = scene["boxes"][target]
            value = features(boxes[0]) if len(boxes) == 1 else None
        if value is None:
            raise ValueError("reference must show one unambiguous " + target)
        return {"features": value, "state": state, "image": str(image)}
