"""
Chase one person with YOLOv8n. No ``--class``, no ball detector.

Purpose:
    Stand in front of the dog and run this on the Pi. The network is
    restricted to COCO person (class id 0), so chairs and TVs are not
    chase targets. Identity lock lives in ``PersonChaser``: once it picks
    you, a bystander walking through the shot is ignored until you leave
    the frame for a couple of seconds.

    From this folder:

        python3 chase_person.py

    Same thing through the package CLI:

        python3 main.py chase-person

Key terms:
    - YOLOv8n: Small person detector. Not used for balls.
    - Lock: The one person box the gait follows.
    - Shim: This file prepends ``chase-person`` and calls ``main.py``.

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
    """Run person chase with the script-style CLI (no subcommand word).

    :param argv: Optional tokens, not including the script name.
    :returns: Process exit code.
    """

    tokens = list(sys.argv[1:] if argv is None else argv)
    return package_main(["chase-person", *tokens])


if __name__ == "__main__":
    raise SystemExit(main())
