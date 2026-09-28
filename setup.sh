#!/usr/bin/env bash
# Create a local Python environment for the XGO Lite movement test.
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt

printf '\nReady. Run: .venv/bin/python scripts/movement_tests/basic_movement.py --help\n'
