"""Deliver a fetched block to the stacking marker.

The marker view and the motions come from a stacking workcell (scripts/stacking:
--generate-placements, then --capture-approach). While carrying, the block is held up out of
the camera's view, so the robot lines up with the marker without moving the arm; then it
lowers the block slowly (one jump flung it, 2026-10-30), does the pickup's reach, opens the
claw and retreats.
"""

import json
import sys
from pathlib import Path

STACKING = Path(__file__).resolve().parents[1] / "stacking"
GAP = 0.5  # seconds between the chained motions
# The first block only needs to land near the marker; looser than a pickup's 2% / 4%.
TOLERANCE = {"u": 0.03, "h": 0.06, "v": 0.05}


def _stacking():
    if str(STACKING) not in sys.path:
        sys.path.insert(0, str(STACKING))


def marker_measure(marker_id):
    """frame -> (features, reason) for the printed ArUco marker."""
    _stacking()
    from vision import Perception

    perception = Perception(marker_id)

    def measure(frame):
        marker = perception.observe(frame)["marker"]
        return (marker, None) if marker else (None, "marker not clearly in view")

    return measure


def chain(*motions):
    """One motion that plays the given ones back to back, starting from the first's start."""
    events, offset = [], 0.0
    for motion in motions:
        events += [dict(event, t=round(event["t"] + offset, 3)) for event in motion["events"]]
        offset += motion["duration"] + GAP
    return {"version": 1, "duration": round(offset - GAP, 3),
            "start": dict(motions[0]["start"]), "events": events}


def load(config, layer=0):
    """{"reference", "motion", "marker_id"} for delivering to `layer` of the workcell."""
    _stacking()
    from profiles import read_motion

    config = Path(config).expanduser()
    cell = json.loads(config.read_text())
    entry = cell["layers"][layer]
    lower_path = (cell.get("verification") or {}).get("motion")
    if not (entry.get("place") and entry.get("reference") and lower_path):
        raise ValueError("generate the placements (--generate-placements) and capture the "
                         "marker view (--capture-approach) first")
    lower = read_motion(config.parent / lower_path, "check")
    place = read_motion(config.parent / entry["place"], "place")
    view = json.loads((config.parent / entry["reference"]).read_text())
    return {"motion": chain(lower, place),
            "reference": {"features": view["features"],
                          "state": {"kneel": view["state"]["kneel"], "arm": None}},
            "marker_id": view.get("marker_id", cell.get("marker_id", 0))}
