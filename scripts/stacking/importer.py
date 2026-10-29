"""Copy a teleop recording into a named workcell asset (read-only on the original).

The motion is validated for its role first (an unknown arm start is refused, never guessed);
a pickup also brings its start reference, normalized to {"state": {arm, kneel}, ...}. Nothing
is overwritten, and every touched "validated" flag is set false until reviewed on the robot.
"""

import json
import os
import re
import shutil
from pathlib import Path

from profiles import pose, read_motion, start_pose
from vision import COLORS, features

EXAMPLE = Path(__file__).with_name("stack_config.example.json")
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
    cell = json.loads((config if config.exists() else EXAMPLE).read_text())
    if not isinstance(cell, dict):
        raise ValueError("the workcell must be a JSON object")
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
    cell["validated"] = False
    temporary = config.with_name(config.name + ".tmp")
    temporary.write_text(json.dumps(cell, indent=2) + "\n")
    os.replace(temporary, config)
    return {"asset": asset, "source": str(session), "copied_to": str(destination),
            "config": str(config), "validated": False}
