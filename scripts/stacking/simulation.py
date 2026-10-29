"""Deterministic end-to-end demo using real color/marker detectors on synthetic images."""

import cv2
import numpy as np

from vision import (COLORS, Perception, inventory, pickup_verdict, tower_verdict)
from workflow import Stopped

FLOOR = [0, 0.45, 1, 1]
HELD = [0.75, 0, 1, 0.35]
STACK = [0.35, 0.12, 0.6, 0.75]
HUES = {"purple": 135, "orange": 15, "light-blue": 100}


def scene(floor=(), held=None, tower=(), marker=True):
    hsv = np.zeros((480, 640, 3), np.uint8)
    hsv[:] = (0, 0, 100)

    def box(color, x, y):
        cv2.rectangle(hsv, (x, y), (x + 40, y + 39), (HUES[color], 210, 210), -1)

    for index, color in enumerate(floor):
        box(color, 40 + index * 100, 370)
    if held:
        box(held, 540, 70)
    for index, color in enumerate(tower):
        box(color, 280, 300 - index * 40)
    frame = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    if marker:
        tile = cv2.aruco.generateImageMarker(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), 0, 80)
        frame[240:340, 100:200] = 255
        frame[250:330, 110:190] = cv2.cvtColor(tile, cv2.COLOR_GRAY2BGR)
    return frame


class Simulation:
    def __init__(self, fail_first=None, drop_first=None, uncertain=None):
        self.perception = Perception()
        self.remaining = list(COLORS)
        self.held, self.placed = None, []
        self.fail_first, self.drop_first, self.uncertain = fail_first, drop_first, uncertain
        self.failed, self.dropped = set(), set()
        self.stopped = False
        self.reference = self.perception.observe(scene())["marker"]

    def survey(self, required):
        view = self.perception.observe(scene(self.remaining))
        if not inventory(view, required, FLOOR)["ready"]:
            raise Stopped("simulated survey missing/ambiguous")

    def pick(self, color):
        if color == self.fail_first and color not in self.failed:
            self.failed.add(color)
            self.held = None
            return
        self.remaining.remove(color)
        self.held = color

    def verify_pick(self, color):
        if color == self.uncertain:
            return "unknown"
        view = self.perception.observe(scene(self.remaining, self.held))
        return pickup_verdict(view, color, FLOOR, HELD)

    def carry(self, color, layer):
        if color == self.drop_first and color not in self.dropped:
            self.dropped.add(color)
            self.remaining.append(color)
            self.held = None
        return self.verify_pick(color)

    def place(self, color, layer):
        self.held = None
        self.placed.append(color)

    def verify_place(self, expected, layer):
        view = self.perception.observe(scene(held=self.held, tower=self.placed))
        return tower_verdict(view, expected, STACK, HELD, self.reference)

    def recover(self, color, layer, origin):
        pass  # no motors; next frame still shows the failed/dropped box on the floor

    def return_to_survey(self, layer):
        pass

    def stop(self):
        self.stopped = True
