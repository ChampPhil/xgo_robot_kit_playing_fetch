"""
Package entry so ``python3 -m xgo_robot stream`` works.

Purpose:
    Python looks for this file when you run the package as a module from
    the *parent* of ``xgo_robot`` (the folder that contains this folder).
    It does not add flags; it hands argv to ``main.py``.

    From inside ``xgo_robot`` itself, use ``python3 main.py stream`` instead.
    ``python3 -m xgo_robot`` from in here looks for a nested package that
    does not exist.

Key terms:
    - ``-m``: Run a package by name. The current directory (or PYTHONPATH)
      must contain ``xgo_robot/``.
    - ``__main__.py``: The file Python executes for ``python3 -m packagename``.

Exports:
    - None. This file only calls ``main()`` and exits.
"""

from .main import main

raise SystemExit(main())
