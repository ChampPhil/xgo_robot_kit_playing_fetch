"""
Deprecated shim: old ``python3 xgo_chase_object.py --class …``.

Purpose:
    Generic ``--class`` chase is gone. Person and ball use different
    detectors. This filename maps the old flags onto the new commands
    so a Pi alias does not immediately break, then you should switch.

    Prefer:

        python3 chase_person.py
        python3 chase_colored_ball.py --color orange

Key terms:
    - Shim: Rewrites argv and calls ``main.py``.
    - ``--class person``: Becomes ``chase-person`` (YOLOv8n).
    - ``--ball-color`` / ``sports ball``: Becomes ``chase-ball --color``.

Exports:
    - main: Compatibility wrapper around ``xgo_robot.main.main``.
"""

from __future__ import annotations

import sys
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from xgo_robot.main import main as package_main


def _legacy_chase_argv(argv: list[str]) -> list[str]:
    """Map old ``--class`` / ``--ball-color`` onto chase-person or chase-ball.

    :param argv: Tokens after the script name.
    :returns: Tokens ``main.py`` understands.
    """

    target_class = None
    ball_color = None
    leftover: list[str] = []
    index = 0
    while index < len(argv):
        token = argv[index]
        if token in ("--class", "--ball-color", "--color") and index + 1 < len(argv):
            value = argv[index + 1]
            if token == "--class":
                target_class = value
            else:
                ball_color = value
            index += 2
            continue
        if token.startswith("--class="):
            target_class = token.split("=", 1)[1]
            index += 1
            continue
        if token.startswith("--ball-color="):
            ball_color = token.split("=", 1)[1]
            index += 1
            continue
        leftover.append(token)
        index += 1

    class_name = (target_class or "").strip().lower()
    if ball_color or class_name in {"sports ball", "ball"}:
        color = ball_color or "orange"
        print(
            "xgo_chase_object.py is deprecated. "
            f"Use: python3 chase_colored_ball.py --color {color}",
            file=sys.stderr,
        )
        return ["chase-ball", "--color", color, *leftover]

    print(
        "xgo_chase_object.py is deprecated. Use: python3 chase_person.py",
        file=sys.stderr,
    )
    return ["chase-person", *leftover]


def main(argv: list[str] | None = None) -> int:
    """Run the mapped chase command.

    :param argv: Optional tokens, not including the script name.
    :returns: Process exit code.
    """

    tokens = list(sys.argv[1:] if argv is None else argv)
    return package_main(_legacy_chase_argv(tokens))


if __name__ == "__main__":
    raise SystemExit(main())
