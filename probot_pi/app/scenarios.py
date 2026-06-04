"""Scripted (v, w) demo profiles for repeatable, presentable tests.

Each scenario is a list of segments (duration_s, v, w, label). A ScenarioRunner
advances on the loop clock and exposes the current label + progress so the
dashboard can show what phase a demo is in. These make T1-T4 style runs
repeatable and let you compare fuzzy on/off under the *same* commanded motion.
"""


class ScenarioRunner:
    def __init__(self, name, segments):
        self.name = name
        self.segments = segments
        self._total = sum(seg[0] for seg in segments)
        self.t = 0.0
        self.label = segments[0][3] if segments else "done"

    def step(self, dt):
        """Advance by dt; return (v, w, done)."""
        self.t += dt
        acc = 0.0
        for dur, v, w, label in self.segments:
            if self.t < acc + dur:
                self.label = label
                return v, w, False
            acc += dur
        self.label = "done"
        return 0.0, 0.0, True

    def progress(self):
        return min(1.0, self.t / self._total) if self._total > 0 else 1.0


# duration_s, v (m/s), w (rad/s), label
SCENARIOS = {
    "straight": [
        (1.0, 0.00, 0.0, "settle"),
        (5.0, 0.30, 0.0, "straight 0.30 m/s — yaw-hold should keep it straight"),
        (1.0, 0.00, 0.0, "stop"),
    ],
    "step_turn": [
        (1.0, 0.00, 0.0, "settle"),
        (2.0, 0.25, 0.0, "straight"),
        (2.0, 0.25, 1.0, "turn left (w=+1.0)"),
        (2.0, 0.25, 0.0, "straight again"),
        (1.0, 0.00, 0.0, "stop"),
    ],
    "spin": [
        (1.0, 0.00, 0.0, "settle"),
        (3.0, 0.00, 1.5, "spin CCW (yaw+)"),
        (1.0, 0.00, 0.0, "pause"),
        (3.0, 0.00, -1.5, "spin CW (yaw-)"),
        (1.0, 0.00, 0.0, "stop"),
    ],
    "figure8": [
        (1.0, 0.00, 0.0, "settle"),
        (4.0, 0.25, 1.0, "loop left"),
        (4.0, 0.25, -1.0, "loop right"),
        (1.0, 0.00, 0.0, "stop"),
    ],
    "slip_test": [
        (1.0, 0.00, 0.0, "settle"),
        (7.0, 0.30, 0.0, "straight — introduce a low-grip patch under ONE wheel now"),
        (1.0, 0.00, 0.0, "stop"),
    ],
}


def names():
    return list(SCENARIOS.keys())


def make(name):
    if name not in SCENARIOS:
        raise KeyError(name)
    return ScenarioRunner(name, SCENARIOS[name])
