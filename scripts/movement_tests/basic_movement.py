"""Short movement test for the XGO Lite on its Raspberry Pi controller.

From the repository root on the robot, run:
    python3 scripts/movement_tests/basic_movement.py

If the built-in menu owns the serial port, stop it first (e.g. sudo pkill -f main.py).
Place the robot on a clear, level floor; Ctrl+C stops the test.
"""

import time

from xgolib import XGO  # type: ignore[import-not-found]  # installed on the robot

DURATION = 1.0  # seconds per movement
PAUSE = 0.5     # seconds at rest between movements


def main():
    input("Clear the area around the robot. Press Enter to start (Ctrl+C to cancel)... ")
    dog = XGO(port="/dev/ttyAMA0", version="xgolite")

    try:
        # Lite ranges: x step +/-25, y step +/-18, rotation +/-150 degrees/s.
        steps = [
            ("forward", dog.move_x, 10),
            ("backward", dog.move_x, -10),
            ("left sidestep", dog.move_y, 8),
            ("right sidestep", dog.move_y, -8),
            ("turn left", dog.turn, 30),
            ("turn right", dog.turn, -30),
        ]
        for label, command, value in steps:
            print(f"Testing {label}...")
            try:
                command(value)
                time.sleep(DURATION)
            finally:
                dog.stop()  # motion commands persist until stopped
            time.sleep(PAUSE)
    finally:
        dog.stop()  # also stop on Ctrl+C or an error

    print("Movement test complete.")


if __name__ == "__main__":
    main()
