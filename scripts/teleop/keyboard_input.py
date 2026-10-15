"""Keyboard control for gamepad_sender.py --keyboard: hold keys in a small window.

Keys are turned into the same controller state the gamepad produces, so the robot
side is unchanged. Holding a key moves; releasing it stops. Fixed speeds (half, or
full with Shift) make recordings cleaner and more repeatable than analog sticks.
"""

import io
import threading
import time
import urllib.request

HALF, FULL = 0.5, 1.0
VIDEO_SIZE = (480, 360)
SHIFT_KEYS = ("left shift", "right shift")
CTRL_KEYS = {"left ctrl", "right ctrl", "left meta", "right meta"}
WALK_KEYS = {"w", "s", "a", "d", "q", "e"}
ARM_KEYS = {"i", "k", "j", "l", "u", "o"}
HELP = (
    "Hold: W/S walk  A/D side-step  Q/E turn  (Shift = full speed)",
    "      I/K arm up/down  J/L arm in/out  U/O claw open/close",
    "      G/T kneel down/stand up   Space align",
    "Press: F fetch (align + pick up; any key cancels)  X record  Y replay",
    "       C reset claw  Z full reset+kneel  B bark  Esc quit",
    "Click this window first; clicking elsewhere releases held keys (fetch keeps going).",
)


def axis(held, negative, positive, speed):
    return speed * ((positive in held) - (negative in held))


def keys_to_state(held, focused=True):
    """Map the set of held key names to the controller state used by make_message."""
    speed = FULL if any(key in held for key in SHIFT_KEYS) else HALF
    arm_mode = bool(held & ARM_KEYS)
    if arm_mode:  # same layout as the gamepad's RB mode: left stick = arm, right stick = claw
        left_x = axis(held, "j", "l", speed)
        left_y = axis(held, "i", "k", speed)  # up is negative, like a stick
        right_x = axis(held, "u", "o", speed)
    else:
        left_x = axis(held, "a", "d", speed)
        left_y = axis(held, "w", "s", speed)
        right_x = axis(held, "q", "e", speed)
    return {
        "left_x": left_x, "left_y": left_y, "right_x": right_x,
        "lb": not arm_mode and bool(held & WALK_KEYS),  # holding a walk key is the dead-man
        "rb": arm_mode,
        "lt": "g" in held or "t" in held,
        "dpad_down": "g" in held, "dpad_up": "t" in held,  # G kneels down, T stands up
        "fetch": "f" in held,
        "attended": focused,
        "a": "space" in held,
        "x": "x" in held, "y": "y" in held, "back": "c" in held, "b": "b" in held,
        "rt": "z" in held,  # full reset, then kneel
        "start": "escape" in held,
    }


class KeyTracker:
    """Which keys are held, from ("down", name) / ("up", name) / ("focus_lost",) / ("quit",)."""

    def __init__(self):
        self.held = set()
        self.focused = True

    def process(self, events):
        for event in events:
            if event[0] == "down":
                if event[1] == "c" and self.held & CTRL_KEYS:
                    self.held = {"escape"}  # Ctrl+C (or Cmd+C) in the window quits
                    continue
                self.held.add(event[1])
            elif event[0] == "up":
                self.held.discard(event[1])
            elif event[0] == "focus_lost":
                self.held.clear()  # keys released while unfocused would never report "up"
                self.focused = False
            elif event[0] == "focus_gained":
                self.focused = True
            elif event[0] == "quit":
                self.held = {"escape"}


def mjpeg_frames(stream):
    """Yield each JPEG from a multipart MJPEG stream (parts carry a Content-Length)."""
    length = None
    while True:
        line = stream.readline()
        if not line:
            return
        line = line.strip()
        if line.lower().startswith(b"content-length:"):
            length = int(line.split(b":", 1)[1])
        elif not line and length is not None:
            data = stream.read(length)
            if len(data) < length:
                return
            yield data
            length = None


class VideoFeed:
    """Keep the newest JPEG from the robot's stream (through the SSH tunnel); reconnects."""

    def __init__(self, url):
        self.url = url
        self.lock = threading.Lock()
        self.jpeg = None
        self.running = True
        threading.Thread(target=self.loop, daemon=True).start()

    def loop(self):
        while self.running:
            try:
                with urllib.request.urlopen(self.url, timeout=3) as response:
                    for jpeg in mjpeg_frames(response):
                        with self.lock:
                            self.jpeg = jpeg
                        if not self.running:
                            return
            except (OSError, ValueError):
                pass  # robot not serving yet, or the session ended
            time.sleep(1.0)

    def latest(self):
        with self.lock:
            return self.jpeg


class KeyboardPad:
    """A small pygame window that captures keys and shows what is being sent."""

    name = "keyboard window"

    def __init__(self, video_url=None):
        import signal

        import pygame

        self.pygame = pygame
        pygame.display.init()
        # SDL takes over SIGINT underneath Python; restore it so Ctrl+C in the terminal
        # still quits (and stops the robot) instead of being swallowed.
        signal.signal(signal.SIGINT, signal.default_int_handler)
        pygame.font.init()
        self.feed = VideoFeed(video_url) if video_url else None
        width = 640 + (VIDEO_SIZE[0] + 12 if self.feed else 0)
        height = max(220, VIDEO_SIZE[1] + 20) if self.feed else 220
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("XGO keyboard control")
        self.font = pygame.font.SysFont(None, 22)
        self.tracker = KeyTracker()

    def events(self):
        pygame = self.pygame
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                yield "down", pygame.key.name(event.key)
            elif event.type == pygame.KEYUP:
                yield "up", pygame.key.name(event.key)
            elif event.type == pygame.WINDOWFOCUSLOST:
                yield ("focus_lost",)
            elif event.type == pygame.WINDOWFOCUSGAINED:
                yield ("focus_gained",)
            elif event.type == pygame.QUIT:
                yield ("quit",)

    def read(self):
        self.tracker.process(list(self.events()))
        state = keys_to_state(self.tracker.held, self.tracker.focused)
        self.draw()
        return state

    def draw(self):
        self.screen.fill((24, 24, 28))
        lines = list(HELP) + ["Held: " + (" ".join(sorted(self.tracker.held)) or "-")]
        for row, text in enumerate(lines):
            colour = (120, 220, 140) if row == len(lines) - 1 else (225, 225, 225)
            self.screen.blit(self.font.render(text, True, colour), (12, 10 + row * 28))
        if self.feed:
            self.draw_video((652, 10))
        self.pygame.display.flip()

    def draw_video(self, origin):
        jpeg = self.feed.latest()
        if jpeg is None:
            text = self.font.render("waiting for video...", True, (160, 160, 160))
            self.screen.blit(text, (origin[0] + 10, origin[1] + 10))
            return
        try:
            image = self.pygame.image.load(io.BytesIO(jpeg), "frame.jpg")
        except Exception:  # a damaged frame: keep the previous picture
            return
        self.screen.blit(self.pygame.transform.smoothscale(image, VIDEO_SIZE), origin)
