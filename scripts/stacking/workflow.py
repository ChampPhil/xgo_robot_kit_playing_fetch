"""Bounded three-color tower sequencing, independent of hardware."""

from vision import COLORS


class Stopped(RuntimeError):
    pass


def validate_order(order):
    if len(order) != 3 or set(order) != set(COLORS):
        raise ValueError("order must contain purple, orange and light-blue exactly once, bottom to top")
    return list(order)


class Tower:
    """Backend owns all resources. Unknown evidence stops; only confirmed floor failures retry."""
    def __init__(self, backend, order, attempts=3, log=None):
        self.backend = backend
        self.order = validate_order(order)
        if isinstance(attempts, bool) or not isinstance(attempts, int) or not 1 <= attempts <= 5:
            raise ValueError("attempts must be 1..5, including the first pickup")
        self.attempts = attempts
        self.log = log or (lambda event: None)
        self.placed = []
        self.state = "idle"

    def event(self, state, **data):
        self.state = state
        self.log(dict(state=state, placed=list(self.placed), **data))

    def run(self):
        try:
            self.event("survey", required=self.order)
            self.backend.survey(self.order)
            for layer, color in enumerate(self.order):
                if layer:
                    self.backend.survey(self.order[layer:])
                for attempt in range(1, self.attempts + 1):
                    self.event("pick", color=color, layer=layer, attempt=attempt)
                    self.backend.pick(color)
                    result = self.backend.verify_pick(color)
                    self.event("verify_pick", color=color, result=result, attempt=attempt)
                    if result == "unknown":
                        raise Stopped("pickup uncertain; box disappearance does not prove a grasp")
                    if result == "floor":
                        if attempt == self.attempts:
                            raise Stopped("pickup attempts exhausted for " + color)
                        self.backend.recover(color, layer, "pickup")
                        continue
                    if result != "held":
                        raise Stopped("invalid pickup verification result")
                    self.event("carry", color=color, layer=layer)
                    result = self.backend.carry(color, layer)
                    if result == "floor":
                        self.event("dropped", color=color, layer=layer, attempt=attempt)
                        if attempt == self.attempts:
                            raise Stopped("pickup attempts exhausted after a dropped box")
                        self.backend.recover(color, layer, "transport")
                        continue
                    if result != "held":
                        raise Stopped("grip uncertain during transport; placement cancelled")
                    self.event("place", color=color, layer=layer, nominal_height_mm=25 * layer)
                    self.backend.place(color, layer)
                    expected = self.placed + [color]
                    if self.backend.verify_place(expected, layer) != "placed":
                        raise Stopped("placement/tower uncertain; do not add another layer")
                    self.placed.append(color)  # commit only after placement evidence
                    self.event("placed", color=color, layer=layer)
                    if layer < len(self.order) - 1:
                        self.backend.return_to_survey(layer)
                    break
            self.event("done")
            return {"status": "done", "order_bottom_to_top": self.order, "placed": self.placed}
        except BaseException as exc:
            self.event("stopped", reason=str(exc))
            raise
        finally:
            self.backend.stop()  # no reset, claw-open, or automatic standing while carrying
