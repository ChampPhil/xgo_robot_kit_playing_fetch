"""Button-triggered behaviours: bark (B), reset claw (BACK/C), full reset (RT/Z) and
fetch (F: reset, align by itself, replay the recorded pickup)."""

import subprocess
from pathlib import Path


BARK_SOUND = Path(__file__).resolve().parent / "sounds" / "bark.wav"


class ResetButton:
    """BACK: open the claw, then move the arm back to its start pose."""

    def __init__(self, arm, args, log):
        self.arm = arm
        self.args = args
        self.log = log
        self.seen = 0

    def update(self, count, allowed):
        pressed = count > self.seen
        self.seen = count
        if not pressed:
            return
        if not allowed:
            self.log("Reset ignored during replay or alignment.")
            return
        self.arm.set_claw(0)
        self.arm.set_pose(self.args.arm_home_x, self.args.arm_home_z)
        self.log("Claw opened; arm back to its start pose.")


class FullReset:
    """RT / Z: stop, run the robot's own full reset, resync our state, then kneel fully."""

    def __init__(self, dog, driver, arm, posture, ramp, log):
        self.dog = dog
        self.driver = driver
        self.arm = arm
        self.posture = posture
        self.ramp = ramp
        self.log = log
        self.seen = 0

    def update(self, count, allowed):
        pressed = count > self.seen
        self.seen = count
        if not pressed:
            return
        if not allowed:
            self.log("Full reset ignored during recording, replay or alignment.")
            return
        self.perform()

    def perform(self):
        self.driver.stop()
        self.dog.reset()  # xgolib: action 255, then 0.75 s for the robot to settle
        self.arm.forget()
        self.arm.set_claw(0)  # reset the claw to fully open
        self.posture.forget()
        self.ramp.reset()
        if hasattr(self.driver, "forget_drift"):
            self.driver.forget_drift()  # the reset may recalibrate the gyro
        self.posture.set_level(1.0)
        self.log("Full reset done; claw open, kneeling. Stand still ~3 s so heading hold can "
                 "re-measure the gyro drift.")


class Fetcher:
    """F: full reset (camera pose = the reference's), settle, align without a button held,
    then replay the recorded pickup. F again, any manual input or the watchdog cancels it;
    clicking away from the keyboard window does not (it only releases held keys)."""

    def __init__(self, fullreset, aligner, teach, log, clock, motion, settle=1.5,
                 align_status=None):
        self.fullreset = fullreset
        self.aligner = aligner
        self.teach = teach
        self.log = log
        self.clock = clock
        self.motion = motion
        self.settle = settle
        self.align_status = align_status or (lambda: (aligner.active,
                                                      aligner.phase == "aligned"))
        self.state = None
        self.seen = 0

    @property
    def active(self):
        return self.state is not None

    @property
    def replaying(self):
        return self.state == "replay"

    def update(self, presses, manual, attended):
        """Call once per message. True while fetch owns the legs, arm and kneel."""
        pressed = presses > self.seen
        self.seen = presses
        if pressed:
            if self.active:
                self.cancel("F pressed again")
                return False
            self.start()
        if not self.active:
            return False
        if manual:
            self.cancel("manual input")
            return False
        if self.state == "settle":
            if self.clock() < self.until:
                return True
            self.state = "align"
            self.log("FETCH: aligning with the recording's start view...")
        if self.state == "align":
            self.aligner.update(True, False)
            active, aligned = self.align_status()
            if aligned:
                self.aligner.update(False, False)  # hand the robot over to the replay
                self.teach.start_replay()
                if not self.teach.replayer.active:
                    self.end("FETCH stopped: the pickup replay could not start")
                    return False
                self.state = "replay"
                self.log("FETCH: aligned; replaying the pickup...")
                return True
            if not active:
                self.end("FETCH stopped: alignment did not finish (reason above)")
                return False
            return True
        if self.teach.replayer.active:
            return True
        self.end("FETCH done.")
        return False

    def start(self):
        if not self.motion:
            self.log("Fetch needs --align-motion (it walks by itself).")
            return
        if self.teach.busy:
            self.log("Fetch ignored while recording or replaying.")
            return
        self.aligner.update(False, False)  # clear a refusal left by an earlier failed attempt
        self.fullreset.perform()
        self.state, self.until = "settle", self.clock() + self.settle
        self.log("FETCH: reset and kneel; F, any movement key or Esc cancels.")

    def cancel(self, reason):
        if self.state == "align":
            self.aligner.update(False, False)  # stops the legs
        elif self.state == "replay" and hasattr(self.teach, "watchdog"):
            self.teach.watchdog()  # aborts the replay and stops the legs
        self.end(f"FETCH cancelled: {reason}.")

    def end(self, text):
        self.state = None
        self.log(text)

    def watchdog(self):
        if self.active:
            self.cancel("no commands")


class Barker:
    """Play the bark when the press counter increases; ignore presses while one is playing."""

    def __init__(self, play=None, log=print):
        self.play = play or (lambda: subprocess.Popen(
            ["aplay", "-q", str(BARK_SOUND)], stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
        self.log = log
        self.seen = 0  # each sender run starts its own receiver, both counting from 0
        self.playing = None

    def update(self, count):
        if count <= self.seen:
            self.seen = count
            return
        self.seen = count
        if self.playing is not None and self.playing.poll() is None:
            return
        try:
            self.playing = self.play()
        except OSError as exc:
            self.log(f"Cannot play bark ({exc}).")
