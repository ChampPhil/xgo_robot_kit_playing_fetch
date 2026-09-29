#!/usr/bin/env bash
# Create a local Python environment for XGO Lite movement and camera-only tests.
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt

printf '\nReady. Movement: .venv/bin/python scripts/movement_tests/basic_movement.py --help\n'
printf 'Box detection: .venv/bin/python scripts/box_detection/detect_boxes.py --help\n'
