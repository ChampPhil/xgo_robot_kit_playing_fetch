"""The gamepad command protocol: one JSON object per line from gamepad_sender.py.

  {"x": fwd, "y": left, "yaw": ccw, "enable": bool,     walking (enable = LB held)
   "arm": bool, "ax": reach, "az": lift, "claw": close,  arm mode (arm = RB held)
   "bark"/"record"/"replay"/"reset"/"full_reset"/"fetch": press counters,
   "kneel": 1 | 0 | -1, "align": bool, "attended": bool}

Axes are -1..1. Counters only ever increase, so a skipped (stale) line cannot lose a press.
"""

import argparse
import json
import math
from typing import NamedTuple, Optional


AXES = ("x", "y", "yaw")
ARM_AXES = ("ax", "az", "claw")


def bounded_number(value, *, minimum, maximum, label):
    try:
        number = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"{label} must be a number") from exc
    if not math.isfinite(number) or not minimum <= number <= maximum:
        raise argparse.ArgumentTypeError(f"{label} must be between {minimum} and {maximum}")
    return number


def axis_value(message, axis):
    value = message.get(axis, 0)
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{axis} must be a finite number")
    return max(-1.0, min(1.0, float(value)))


class Message(NamedTuple):
    walk: dict
    arm: Optional[dict]
    bark: int
    kneel: int
    record: int
    replay: int
    raw: dict
    manual: bool  # the operator is touching a stick, LB/RB arm mode or the kneel combo
    align: bool = False  # A held: align with the latest recording's reference
    reset: int = 0  # BACK presses: open the claw and move the arm to its start pose
    full_reset: int = 0  # RT / Z presses: full robot reset, then kneel
    fetch: int = 0  # F presses: reset, align by itself, then replay the pickup
    attended: bool = True  # False when the keyboard window lost focus


def counter(message, key):
    value = message.get(key, 0)
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{key} must be a non-negative integer")
    return value


def parse_message(line):
    """Parse one JSON command line into a Message. Raises ValueError on bad input.

    walk is {axis: -1..1}, all zero unless enable is true and arm mode is off.
    arm is {ax, az, claw: -1..1} in arm mode, else None. bark/record/replay are press
    counters. kneel is 1 (kneel down), -1 (stand up) or 0.
    """
    message = json.loads(line)
    if not isinstance(message, dict):
        raise ValueError("command must be a JSON object")
    bark, record, replay = (counter(message, key) for key in ("bark", "record", "replay"))
    kneel = message.get("kneel", 0)
    if isinstance(kneel, bool) or kneel not in (-1, 0, 1):
        raise ValueError("kneel must be -1, 0 or 1")
    arm = None
    if message.get("arm") is True:
        arm = {axis: axis_value(message, axis) for axis in ARM_AXES}
    sticks = {axis: axis_value(message, axis) for axis in AXES}
    walk = dict.fromkeys(AXES, 0.0)
    if message.get("enable") is True and arm is None:
        walk = sticks
    manual = any(sticks.values()) or arm is not None or kneel != 0
    return Message(walk, arm, bark, kneel, record, replay, message, manual,
                   message.get("align") is True, counter(message, "reset"),
                   counter(message, "full_reset"), counter(message, "fetch"),
                   message.get("attended", True) is not False)
