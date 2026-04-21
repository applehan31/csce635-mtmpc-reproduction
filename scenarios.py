"""Obstacle scenarios for fair-comparison experiments.

SCENARIO_A: 3 cylinders (original scenario from the paper reproducibility runs).
SCENARIO_B: 6 cylinders, mixed sizes — moderate difficulty.
SCENARIO_C: Vertical-wall doorway (Singletary 2020 scenario 3 analogue). Seven
    r=0.3 cylinders form a fully sealed wall at x=1.5 (surface y=-0.6 to
    y=4.05); the centre obstacle at (1.5,1.5) blocks the direct diagonal path.
    Doorway at y≈2.025 (surface gap y=1.8–2.25, 0.45 m wide) is the only
    passage; centre-to-centre gap 1.05 m gives mt-MPC 0.025 m above its
    safety threshold. CBF-QP expected to collide on reactive late correction.
    Both escape routes sealed: bottom by (1.5,-0.3), top by (1.5,3.75).

All scenarios share the same START=[0,0,1] and GOAL=[3,3,1].
OBS_HEIGHT = 2.0 m (fully vertical cylinders).

Format: each obstacle is [cx, cy, cz_base, radius].
cz_base is the bottom of the cylinder; center = cz_base + OBS_HEIGHT/2.
"""

OBS_HEIGHT = 2.0

START_POS = [0.0, 0.0, 1.0]
GOAL_POS  = [3.0, 3.0, 1.0]

SCENARIO_A = [
    [1.0, 1.0, 0.0, 0.3],
    [2.0, 1.5, 0.0, 0.3],
    [1.5, 2.5, 0.0, 0.3],
]

SCENARIO_B = [
    # original three, preserved
    [1.0, 1.0, 0.0, 0.3],
    [2.0, 1.5, 0.0, 0.3],
    [1.5, 2.5, 0.0, 0.3],
    # three additions, varied sizes
    [0.8, 2.0, 0.0, 0.2],
    [2.3, 2.3, 0.0, 0.4],
    [1.2, 0.5, 0.0, 0.25],
]

SCENARIO_C = [
    # Vertical wall along x=1.5 with obstacle blocking the direct path.
    # Start (0,0,1) -> Goal (3,3,1): direct diagonal crosses (1.5, 1.5).
    # Obstacle at (1.5, 1.5) blocks it; doorway at y≈2.025 forces a
    # 0.525 m upward detour. Surface gap y=1.8 to y=2.25 = 0.45 m.
    # Centre-to-centre gap 1.05 m: mt-MPC margin 0.025 m above threshold,
    # CBF reactive control expected to collide on late course correction.
    # Wall fully sealed: bottom surface y=-0.6, top surface y=4.05.
    [1.5, -0.3, 0.0, 0.3],  # seal bottom
    [1.5,  0.3, 0.0, 0.3],
    [1.5,  0.9, 0.0, 0.3],
    [1.5,  1.5, 0.0, 0.3],  # blocks direct diagonal
    # doorway: surface gap y=1.8 to y=2.25 = 0.45m, centre y=2.025
    [1.5,  2.55, 0.0, 0.3],
    [1.5,  3.15, 0.0, 0.3],
    [1.5,  3.75, 0.0, 0.3],  # seal top
]

SCENARIOS = {"A": SCENARIO_A, "B": SCENARIO_B, "C": SCENARIO_C}
