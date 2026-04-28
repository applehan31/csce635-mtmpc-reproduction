"""Dynamic-obstacle extension of Scenario B — v2 (open-space layout).

The Scenario B static cluster occupies x∈[0.8, 2.3], y∈[0.5, 2.5]. To
challenge controllers without colliding visually with that cluster, two
moving cylinders sweep across the OPEN SPACE just outside it, on the
diagonal between START=(0,0) and GOAL=(3,3):

  obstacle 1 — lower band, y=0.0 fixed, x oscillates 0.5 ↔ 2.5,
               period 3.0 s. Cuts across the START-side approach.
  obstacle 2 — upper band, y=3.0 fixed, x oscillates 0.5 ↔ 2.5,
               period 3.5 s, phase π/2 (out of sync). Cuts across the
               GOAL-side approach.

`get_dynamic_obstacle_position(t)` returns a list of (x, y) tuples,
one per moving obstacle, in the order [obstacle 1, obstacle 2].
"""
import math

from scenarios import (  # noqa: F401  re-export for downstream
    OBS_HEIGHT,
    START_POS,
    GOAL_POS,
    SCENARIO_A,
    SCENARIO_B,
    SCENARIO_C,
    SCENARIOS,
)

SCENARIO_B_DYNAMIC = SCENARIO_B   # six static cylinders, unchanged

# Two moving cylinders, both in open space (above and below the static
# cluster). Each runs a sin sweep so it spends most of its time near
# the diagonal corridor.
DYNAMIC_OBSTACLES = [
    {
        "name":       "lower",
        "radius":     0.25,
        "cz_base":    0.0,
        # y fixed at 0.0; x ∈ [0.50, 2.50]
        "x_center":   1.50, "x_amp": 1.00,
        "y_center":   0.00, "y_amp": 0.00,
        "period":     3.0,
        "phase":      0.0,
    },
    {
        "name":       "upper",
        "radius":     0.25,
        "cz_base":    0.0,
        # y fixed at 3.0; x ∈ [0.50, 2.50]
        "x_center":   1.50, "x_amp": 1.00,
        "y_center":   3.00, "y_amp": 0.00,
        "period":     3.5,
        "phase":      math.pi / 2,
    },
]

# Backwards-compat alias — older scripts may import a singular dict.
DYNAMIC_OBSTACLE = DYNAMIC_OBSTACLES[0]


def get_dynamic_obstacle_position(t: float):
    """Return a list of (x, y) centres for every moving obstacle at time t."""
    out = []
    for o in DYNAMIC_OBSTACLES:
        omega = 2.0 * math.pi / o["period"]
        x = o["x_center"] + o["x_amp"] * math.sin(omega * t + o["phase"])
        y = o["y_center"] + o["y_amp"] * math.sin(omega * t + o["phase"])
        out.append((x, y))
    return out
