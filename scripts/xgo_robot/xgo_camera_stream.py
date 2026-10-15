"""
Shim: old ``python3 xgo_camera_stream.py`` command, new package guts.

Purpose:
    Before the refactor this file *was* the entire live-page program
    (~1700 lines). It now forwards to ``main.py stream`` (or ``view`` if
    you passed ``--view URL``) so existing muscle memory and Pi aliases
    keep working.

    Prefer the new command:

        python3 main.py stream
        python3 main.py view --url http://<pi>:8080/stream

    This shim still accepts the old flag layout with no subcommand.

Key terms:
    - Shim: A leftover filename that only rewrites argv and calls ``main``.
    - ``--view``: Old laptop-side flag. Mapped to subcommand ``view --url``.

Exports:
    - main: Compatibility wrapper around ``xgo_robot.main.main``.
"""

from __future__ import annotations

import sys
from pathlib import Path

# Same bootstrap as main.py: this file is often run as a script from inside
# the package folder, so relative imports are not available yet.
if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from xgo_robot.main import main as package_main


def _legacy_argv(argv: list[str]) -> list[str]:
    """Translate the old no-subcommand CLI into ``stream`` / ``view``.

    :param argv: Tokens after the script name.
    :returns: Tokens ``main.py`` understands.
    """

    view_url = None
    leftover: list[str] = []
    index = 0
    while index < len(argv):
        token = argv[index]
        if token == "--view" and index + 1 < len(argv):
            view_url = argv[index + 1]
            index += 2
            continue
        if token.startswith("--view="):
            view_url = token.split("=", 1)[1]
            index += 1
            continue
        leftover.append(token)
        index += 1
    if view_url is not None:
        return ["view", "--url", view_url, *leftover]
    return ["stream", *leftover]


def main(argv: list[str] | None = None) -> int:
    """Run stream (or view) with the old argument style.

    :param argv: Optional tokens, not including the script name.
    :returns: Process exit code.
    """

    tokens = list(sys.argv[1:] if argv is None else argv)
    return package_main(_legacy_argv(tokens))


if __name__ == "__main__":
    raise SystemExit(main())
