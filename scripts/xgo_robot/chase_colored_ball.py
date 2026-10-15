"""
Chase a solid-colored ball with HSV + roundness. No YOLO.

Purpose:
    YOLOv8n's ``sports ball`` class is a basketball-sized object in COCO.
    A ping-pong ball at hallway distance is a handful of pixels after the
    640 resize, so the network misses it. This script never loads
    ultralytics. It thresholds a named HSV range (orange, red, yellow,
    green, blue) and keeps contours that are round enough.

    From this folder:

        python3 chase_colored_ball.py --color orange

    Same thing through the package CLI:

        python3 main.py chase-ball --color orange

Key terms:
    - HSV blob: Pixels whose hue/saturation/value match the named color.
    - Circularity: How round a contour is. Speckles and sleeves are dropped.
    - Shim: This file prepends ``chase-ball`` and calls ``main.py``.

Exports:
    - main: Compatibility wrapper around ``xgo_robot.main.main``.
"""

from __future__ import annotations

import sys
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from xgo_robot.main import main as package_main


def main(argv: list[str] | None = None) -> int:
    """Run ball chase with the script-style CLI (no subcommand word).

    :param argv: Optional tokens, not including the script name. Must
      include ``--color``.
    :returns: Process exit code.
    """

    tokens = list(sys.argv[1:] if argv is None else argv)
    return package_main(["chase-ball", *tokens])


if __name__ == "__main__":
    raise SystemExit(main())
