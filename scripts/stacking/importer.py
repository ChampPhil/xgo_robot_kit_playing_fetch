"""Copy a teleop recording into a named workcell asset (read-only on the original).

The motion is validated for its role first (an unknown arm start is refused, never guessed);
a pickup also brings its start reference, normalized to {"state": {arm, kneel}, ...}. Nothing
is overwritten, and every touched "validated" flag is set false until reviewed on the robot.
"""

import copy
import json
import os
import re
import shutil
import sys
from pathlib import Path

from profiles import pose, read_motion, start_pose, validate_motion
from vision import COLORS, features

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "teleop"))
from teach import load_motion  # noqa: E402

EXAMPLE = Path(__file__).with_name("stack_config.example.json")
LAYER_HEIGHTS_MM = (0, 25, 50)
FLOOR_ROI = [0, 0.4, 1, 1]  # alignment's floor filter: the lifted box is in the top part
RELEASE_AFTER = 0.5  # seconds at the release point before opening
LIFT_AFTER = 1.0  # seconds after opening before moving away
RETREAT_SECONDS, RETREAT_STEPS = 2.0, 10
LAYER_KEYS = {"to-stack": ("to_stack", "carry"), "place": ("place", "place"),
              "inspect": ("inspect", "inspect"), "return": ("return", "return"),
              "recover-drop": ("recover_drop", "recover")}


def asset_spec(asset):
    """asset name -> (config field path, motion role, reference target or None)."""
    if asset == "survey":
        return ("survey_motion",), "survey", None
    if asset == "carry-check":
        return ("verification", "motion"), "check", None
    match = re.fullmatch(r"(pick|retry)-(%s)" % "|".join(map(re.escape, COLORS)), asset)
    if match:
        kind, color = match.groups()
        if kind == "pick":
            return ("pickups", color, "motion"), "pick", color
        return ("pickups", color, "retry"), "recover", None
    match = re.fullmatch(r"layer-([012])/(%s)" % "|".join(map(re.escape, LAYER_KEYS)), asset)
    if match:
        key, role = LAYER_KEYS[match.group(2)]
        return ("layers", int(match.group(1)), key), role, None
    raise ValueError("unknown asset %r; use survey, carry-check, pick-<color>, retry-<color> "
                     "or layer-<0..2>/<%s>" % (asset, "|".join(LAYER_KEYS)))


def pickup_reference(session, color, motion, perception):
    """The recording's start reference in the stacking format, checked like Plan.reference."""
    import cv2

    data = json.loads((session / "reference.json").read_text())
    saved = data.get("state") or {}
    state = pose({"arm": saved.get("arm"), "kneel": saved.get("kneel")})
    if start_pose(motion) != state:
        raise ValueError("the recording's start differs from its reference pose")
    image = session / data.get("image", "reference.jpg")
    frame = cv2.imread(str(image))
    if frame is None:
        raise ValueError("cannot read reference image " + str(image))
    boxes = perception.observe(frame)["boxes"][color]
    if len(boxes) != 1:
        raise ValueError("reference must show one unambiguous " + color + " box")
    return {"version": 1, "target": color, "state": state, "features": features(boxes[0]),
            "image": "reference.jpg"}, image


def import_recording(session, asset, config, perception):
    session = Path(session).expanduser()
    config = Path(config).expanduser()
    fields, role, color = asset_spec(asset)
    motion = read_motion(session / "motion.json", role)  # FileNotFoundError if unfinished
    reference = image = None
    if color is not None:
        reference, image = pickup_reference(session, color, motion, perception)
    cell = load_cell(config)
    entry = cell
    for field in fields[:-1]:
        entry = entry[field]
    destination = config.parent / "taught" / asset
    if destination.exists():
        raise ValueError(str(destination) + " already exists; remove it yourself to re-import")

    destination.mkdir(parents=True)
    shutil.copyfile(session / "motion.json", destination / "motion.json")
    relative = destination.relative_to(config.parent).as_posix()
    entry[fields[-1]] = relative + "/motion.json"
    if reference is not None:
        shutil.copyfile(image, destination / "reference.jpg")
        (destination / "reference.json").write_text(json.dumps(reference, indent=2) + "\n")
        entry["reference"] = relative + "/reference.json"
    if isinstance(entry, dict) and entry is not cell:
        entry["validated"] = False
    save_cell(config, cell)
    return {"asset": asset, "source": str(session), "copied_to": str(destination),
            "config": str(config), "validated": False}


def load_cell(config):
    cell = json.loads((config if config.exists() else EXAMPLE).read_text())
    if not isinstance(cell, dict):
        raise ValueError("the workcell must be a JSON object")
    return cell


def save_cell(config, cell):
    cell["validated"] = False  # anything written here is unreviewed
    config.parent.mkdir(parents=True, exist_ok=True)
    temporary = config.with_name(config.name + ".tmp")
    temporary.write_text(json.dumps(cell, indent=2) + "\n")
    os.replace(temporary, config)


def write_new(path, data):
    path.mkdir(parents=True)
    (path / "motion.json").write_text(json.dumps(data, indent=2) + "\n")


def raised(z, offset, near, far):
    """z moved up by `offset` in proportion to how far it is from `far` toward `near` (the
    release point): `far` stays put, `near` rises by the full offset."""
    if near == far:
        return round(z + offset)
    share = min(1.0, max(0.0, (far - z) / (far - near)))
    return round(z + offset * share)


def placement(pickup, offset):
    """The pickup's reach replayed while holding, released where the claw closed, then a
    straight retreat back and up to the reach's start; the reach ends `offset` mm higher
    (0 / 25 / 50 for layers 1-3). The pickup's own lift is not replayed: it pulls the gripped
    box back along the floor, which with an open claw would drag the placed block."""
    events = sorted(pickup["events"], key=lambda event: event["t"])
    claws = [event for event in events if "claw" in event]
    close = next((event["t"] for event in claws if event["claw"] > 0), None)
    if close is None:
        raise ValueError("the pickup recording has no claw close to mirror")
    reach = [event for event in events if "arm" in event and event["t"] <= close]
    if not reach:
        raise ValueError("the pickup recording has no arm reach before the claw closes")
    kneels = [event["kneel"] for event in events if "kneel" in event and event["t"] <= close]
    kneel = kneels[-1] if kneels else pickup["start"].get("kneel")
    first, release = reach[0], reach[-1]
    start_z, close_z = first["arm"][1], release["arm"][1]
    out = [{"t": round(e["t"] - first["t"], 3),
            "arm": [e["arm"][0], raised(e["arm"][1], offset, close_z, start_z)]} for e in reach]
    opened = out[-1]["t"] + RELEASE_AFTER
    out.append({"t": round(opened, 3), "claw": 0})
    (x0, z0), (x1, z1) = out[-2]["arm"], first["arm"]
    for step in range(1, RETREAT_STEPS + 1):
        share = step / RETREAT_STEPS
        out.append({"t": round(opened + LIFT_AFTER + RETREAT_SECONDS * share, 3),
                    "arm": [round(x0 + (x1 - x0) * share), round(z0 + (z1 - z0) * share)]})
    return {"version": 1, "duration": out[-1]["t"],
            "start": {"arm": list(first["arm"]), "kneel": kneel, "claw": None}, "events": out}


def generate_placements(session, config):
    """Write taught/layer-<n>/place and taught/carry-check from a proven pickup recording."""
    session, config = Path(session).expanduser(), Path(config).expanduser()
    pickup = load_motion(session)
    places = []
    for index, offset in enumerate(LAYER_HEIGHTS_MM):
        motion = placement(pickup, offset)
        try:
            validate_motion(copy.deepcopy(motion), "place")
        except ValueError as exc:
            raise ValueError("layer %d (+%d mm): %s" % (index + 1, offset, exc)) from exc
        places.append(motion)
    start = places[0]["start"]
    check = {"version": 1, "duration": 0.5, "start": dict(start),
             "events": [{"t": 0.5, "arm": list(start["arm"])}]}  # hold the box, arm ready
    validate_motion(copy.deepcopy(check), "check")
    cell = load_cell(config)
    root = config.parent / "taught"
    targets = [root / ("layer-%d" % index) / "place" for index in range(3)]
    targets.append(root / "carry-check")
    for target in targets:
        if target.exists():
            raise ValueError(str(target) + " already exists; remove it yourself to regenerate")
    for index, (target, motion) in enumerate(zip(targets, places)):
        write_new(target, motion)
        cell["layers"][index]["place"] = target.relative_to(config.parent).as_posix() + \
            "/motion.json"
        cell["layers"][index]["validated"] = False
    write_new(targets[3], check)
    verify = cell["verification"]
    verify["motion"] = "taught/carry-check/motion.json"
    verify["held_roi"] = None  # the floor rule
    if verify.get("floor_roi") is None:
        verify["floor_roi"] = list(FLOOR_ROI)
    verify["validated"] = False
    save_cell(config, cell)
    return {"generated": [str(target) for target in targets], "source": str(session),
            "release_heights_mm": list(LAYER_HEIGHTS_MM), "validated": False}


def capture_approach(config, frame, perception):
    """Save the marker as seen from the placement start (the robot aligned to a block on the
    stack spot); every layer aligns to it."""
    import cv2

    config = Path(config).expanduser()
    cell = load_cell(config)
    place = cell["layers"][0].get("place")
    if not place:
        raise ValueError("generate the placements first (--generate-placements)")
    state = start_pose(read_motion(config.parent / place, "place"))
    marker = perception.observe(frame)["marker"]
    if marker is None:
        raise ValueError("the marker is not clearly in view")
    target = config.parent / "taught" / "approach"
    if target.exists():
        raise ValueError(str(target) + " already exists; remove it yourself to recapture")
    target.mkdir(parents=True)
    if not cv2.imwrite(str(target / "reference.jpg"), frame):
        raise ValueError("cannot save the approach image")
    (target / "reference.json").write_text(json.dumps(
        {"version": 1, "target": "marker", "state": state, "features": marker,
         "image": "reference.jpg", "marker_id": perception.marker_id}, indent=2) + "\n")
    for layer in cell["layers"]:
        layer["reference"] = "taught/approach/reference.json"
        layer["validated"] = False
    save_cell(config, cell)
    return {"captured": str(target), "state": state, "validated": False}
