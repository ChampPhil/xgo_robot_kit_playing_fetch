"""Self-calibrating alignment: walk until the box looks like the recording's reference photo.

The robot measures how each step primitive (turn, forward, strafe) moves the box in the
image, uses that to pick the next step, and keeps refining it as it goes. See
docs/superpowers/specs/2026-10-07-self-calibrating-alignment-design.md.
"""

import json
import math
from datetime import datetime

from teach import write_json_atomic

PRIMITIVES = ("turn", "forward", "strafe")
NOISE = 0.005  # per-step image change below this is treated as "this primitive does nothing"
LEARN_RATE = 0.3
LEARN_MIN_AMOUNT = 0.25


def errors(features, reference):
    """(eu, eh, ev): horizontal offset, relative size error, vertical offset vs the reference."""
    return (features["u"] - reference["u"], features["h"] / reference["h"] - 1.0,
            features["v"] - reference["v"])


def within(error, tol):
    """True when the driven errors (eu, eh) are inside tolerance; ev is checked separately."""
    return abs(error[0]) <= tol["u"] and abs(error[1]) <= tol["h"]


def usable(column):
    return column is not None and math.hypot(column[0], column[1]) >= NOISE


def choose_step(error, columns):
    """Pick (primitive, amount in [-1, 1]) predicted to shrink (eu, eh) the most.

    Returns (None, 0.0) when no usable primitive is predicted to help.
    """
    eu, eh = error[0], error[1]
    best, best_amount, best_residual = None, 0.0, math.hypot(eu, eh)
    for name in PRIMITIVES:
        column = columns.get(name)
        if not usable(column):
            continue
        cu, ch = column
        amount = -(cu * eu + ch * eh) / (cu * cu + ch * ch)
        amount = max(-1.0, min(1.0, amount))
        residual = math.hypot(eu + amount * cu, eh + amount * ch)
        if residual < best_residual - 1e-9:
            best, best_amount, best_residual = name, amount, residual
    return best, best_amount


def broyden_update(column, observed, amount, rate=LEARN_RATE):
    """Move a column toward the change actually observed for `amount` of a step."""
    if abs(amount) < LEARN_MIN_AMOUNT:
        return list(column)
    return [c + rate * (o / amount - c) for c, o in zip(column, observed)]


def finite_pair(value):
    return (isinstance(value, list) and len(value) == 2
            and all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)
                    for x in value))


def load_calibration(path):
    """The saved calibration, or None if missing, damaged or the wrong shape."""
    try:
        data = json.loads(path.read_text())
    except (OSError, ValueError):
        return None
    if not isinstance(data, dict) or data.get("version") != 1:
        return None
    columns = data.get("columns")
    if not isinstance(columns, dict):
        return None
    for name in PRIMITIVES:
        if columns.get(name) is not None and not finite_pair(columns[name]):
            return None
    return data


def save_calibration(path, calibration):
    path.parent.mkdir(parents=True, exist_ok=True)
    write_json_atomic(path, dict(calibration, version=1,
                                 created=datetime.now().isoformat(timespec="seconds")))


def compatible(calibration, setup):
    """A saved calibration only applies with the same step speeds/time and camera pose."""
    return all(calibration.get(key) == setup.get(key)
               for key in ("speeds", "step_seconds", "kneel", "arm"))
