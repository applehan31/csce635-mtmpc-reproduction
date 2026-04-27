"""CBF-QP single-drone 3D navigation.

Based on:
  Singletary et al. (2020). Comparative Analysis of Control Barrier Functions and
  Artificial Potential Fields for Obstacle Avoidance. IROS 2021.

Template: gym-pybullet-drones/examples/pid.py (CtrlAviary + DSLPIDControl).
Scenario is identical to apf_3d.py for a fair comparison.

QP solver: scipy.optimize.minimize / SLSQP (no extra dependencies).
  cvxpy would also work but requires a separate install; SLSQP is sufficient for
  a 3-variable, 4-constraint QCQP at 48 Hz.

Usage:
    conda activate drones
    python experiments/cbf_qp_3d.py [--no-gui]

# REPRODUCIBILITY NOTES - CBF-QP 3D
# vs 2D implementation:
# - CBF operates in the HORIZONTAL (XY) plane — correct for vertical cylinders.
#   Although the task spec says "3D sphere", using a 3D sphere centred at
#   [cx, cy, 1.0] introduces a downward escape: when pos_z < 1.0 the gradient
#   2*(pos - c_i) has a NEGATIVE z-component, so the QP pushes v_z < 0 to
#   reduce 3D distance, causing the drone to dive under the obstacle. Tested
#   and confirmed: drone descended to z=0.54 before the fix.
#   Horizontal CBF is equivalent to an infinitely-tall cylinder and eliminates
#   the z-escape while keeping the math identical to the 2D case.
#   h_i = (px-cx)^2 + (py-cy)^2 - R_i^2,  R_i = r + safety_margin
#   A_i = [2*(px-cx), 2*(py-cy), 0.0]      (z row is zero; no coupling)
# - Velocity-space CBF (relative degree 1 w.r.t. position input to DSL-PID).
#   dh_i/dt = A_i @ v  (affine in v)
#   Constraint: A_i @ v >= -alpha * h_i
# - v_safe fed into DSL-PID via normalised LOOKAHEAD (same trick as APF):
#   target_pos = pos + (v_safe / ||v_safe||) * 0.10
# - z-altitude is regulated by v_nom: since GOAL_POS.z = 1.0 and v_nom
#   points from pos to GOAL_POS, any z-drift produces a restoring v_nom_z.
#   The QP objective then drives v_z -> v_nom_z (no CBF interference).
# - alpha tuning: too high => early intervention but aggressive swerving;
#   too low => late avoidance, may violate h_i >= 0.
# Note on HO-CBF: HO-CBF (relative degree 2) was suggested for
#   acceleration-level control. The implementation here uses a 1st-order
#   velocity CBF: the DSL-PID inner loop tracks the commanded velocity well
#   enough that velocity-space CBF is sufficient. Direct thrust control would
#   require HO-CBF.
# QP feasibility: always feasible when h_i(pos) > 0 for all i (proved by
#   choosing v = 0, which trivially satisfies A_i @ 0 = 0 >= -alpha * h_i
#   when h_i >= 0). Infeasibility only occurs if drone enters the unsafe set.
"""

import os
import csv
import time
import argparse

import numpy as np
import pybullet as p
from scipy.optimize import minimize

from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.utils import sync

# ── Scenario (identical to apf_3d.py for fair comparison) ────────────────────
START_POS = np.array([0.0, 0.0, 1.0])
GOAL_POS  = np.array([3.0, 3.0, 1.0])

# [cx, cy, cz_base, radius]
OBSTACLES = [
    [1.0, 1.0, 0.0, 0.3],
    [2.0, 1.5, 0.0, 0.3],
    [1.5, 2.5, 0.0, 0.3],
]
OBS_HEIGHT = 2.0

# ── CBF-QP Parameters ─────────────────────────────────────────────────────────
K_ATT         = 1.0    # nominal speed toward goal [m/s]; clipped to V_MAX
V_MAX         = 0.5    # max commanded velocity [m/s]
ALPHA         = 1.0    # CBF class-K gain: larger → later but sharper avoidance
SAFETY_MARGIN = 0.15   # extra buffer beyond obstacle radius [m]

# Pre-computed safe radii: physical radius + safety buffer
_OBS_SAFE_R = [r + SAFETY_MARGIN for _, _, _, r in OBSTACLES]

# Lookahead: same reasoning as apf_3d.py — 0.10 m normalised carrot gives the
# DSL-PID a consistent position error to track (~0.4 m/s cruise speed).
LOOKAHEAD = 0.10

# ── Simulation ────────────────────────────────────────────────────────────────
CTRL_FREQ  = 48
SIM_FREQ   = 240
DURATION_S = 60
DT         = 1.0 / CTRL_FREQ

# ── Termination ───────────────────────────────────────────────────────────────
GOAL_TOL  = 0.15
CRASH_TOL = 0.05

# ── Logging ───────────────────────────────────────────────────────────────────
_HERE       = os.path.dirname(os.path.abspath(__file__))
RESULTS_DIR = os.path.join(_HERE, "results")
LOG_PATH    = os.path.join(RESULTS_DIR, "cbf_qp_3d_log.csv")


# ── CBF-QP Solver ─────────────────────────────────────────────────────────────

def _compute_v_nom(pos: np.ndarray) -> np.ndarray:
    """Goal-directed nominal velocity, clipped to V_MAX."""
    d = GOAL_POS - pos
    dist = max(float(np.linalg.norm(d)), 0.01)
    v = K_ATT * d / dist          # unit vector * K_ATT
    speed = float(np.linalg.norm(v))
    if speed > V_MAX:
        v = v * (V_MAX / speed)
    return v


def _cbf_qp(pos: np.ndarray, v_nom: np.ndarray,
            v0: np.ndarray) -> tuple:
    """Solve CBF-QP for a safe velocity command.

    Problem:
        min   ||v - v_nom||^2
        s.t.  2*(pos - c_i)^T v >= -alpha * h_i    for each obstacle i
              ||v||^2 <= V_MAX^2

    Returns (v_safe [3,], feasible [bool]).
    """
    # Build constraint data A @ v >= b  (one row per obstacle, horizontal only)
    # The z-row is zero so the CBF never couples into vertical motion.
    A_list, b_list = [], []
    for (cx, cy, _, _r), R_i in zip(OBSTACLES, _OBS_SAFE_R):
        dx   = float(pos[0] - cx)
        dy   = float(pos[1] - cy)
        h_i  = dx * dx + dy * dy - R_i * R_i   # horizontal CBF value
        A_list.append([2.0 * dx, 2.0 * dy, 0.0])
        b_list.append(-ALPHA * h_i)
    A = np.array(A_list)   # (n_obs, 3)
    b = np.array(b_list)   # (n_obs,)

    # Fast-path: v_nom already satisfies all constraints
    if (np.all(A @ v_nom >= b - 1e-9)
            and float(np.dot(v_nom, v_nom)) <= V_MAX ** 2 + 1e-9):
        return v_nom.copy(), True

    # SLSQP solve
    constraints = [
        {
            "type": "ineq",
            "fun": lambda v, A=A, b=b: A @ v - b,
            "jac": lambda v, A=A: A,
        },
        {
            "type": "ineq",
            "fun": lambda v: V_MAX ** 2 - float(np.dot(v, v)),
            "jac": lambda v: -2.0 * v,
        },
    ]

    res = minimize(
        fun=lambda v: float(np.dot(v - v_nom, v - v_nom)),
        jac=lambda v: 2.0 * (v - v_nom),
        x0=v0,
        method="SLSQP",
        constraints=constraints,
        options={"ftol": 1e-8, "maxiter": 200},
    )

    feasible = res.success or res.status in (0, 1)
    if feasible and res.x is not None:
        v_safe = res.x
    else:
        # Fallback: clip v_nom to V_MAX — no CBF guarantee, but keeps moving
        v_safe = v_nom * min(1.0, V_MAX / max(float(np.linalg.norm(v_nom)), 1e-6))
    return v_safe, feasible


# ── Helpers (identical to apf_3d.py) ─────────────────────────────────────────

def _min_surface_dist(pos: np.ndarray) -> float:
    """Horizontal distance from pos to nearest obstacle surface."""
    return min(
        float(np.sqrt((pos[0] - cx) ** 2 + (pos[1] - cy) ** 2)) - r
        for cx, cy, _, r in OBSTACLES
    )


def _load_obstacles(client: int) -> list:
    ids = []
    for cx, cy, cz_b, r in OBSTACLES:
        cz_c = cz_b + OBS_HEIGHT / 2.0
        col  = p.createCollisionShape(p.GEOM_CYLINDER, radius=r,
                                      height=OBS_HEIGHT, physicsClientId=client)
        vis  = p.createVisualShape(p.GEOM_CYLINDER, radius=r, length=OBS_HEIGHT,
                                   rgbaColor=[0.2, 0.6, 1.0, 0.7],
                                   physicsClientId=client)
        body = p.createMultiBody(baseMass=0,
                                 baseCollisionShapeIndex=col,
                                 baseVisualShapeIndex=vis,
                                 basePosition=[cx, cy, cz_c],
                                 physicsClientId=client)
        ids.append(body)
    return ids


# ── Main ──────────────────────────────────────────────────────────────────────

def run(gui: bool = True):
    os.makedirs(RESULTS_DIR, exist_ok=True)

    env = CtrlAviary(
        drone_model=DroneModel.CF2X,
        num_drones=1,
        initial_xyzs=START_POS.reshape(1, 3),
        initial_rpys=np.zeros((1, 3)),
        physics=Physics.PYB,
        pyb_freq=SIM_FREQ,
        ctrl_freq=CTRL_FREQ,
        gui=gui,
        record=False,
        obstacles=False,
        user_debug_gui=False,
    )
    PYB_CLIENT = env.getPyBulletClient()
    _load_obstacles(PYB_CLIENT)

    p.resetDebugVisualizerCamera(
        cameraDistance=4.7,
        cameraYaw=83,
        cameraPitch=-51,
        cameraTargetPosition=[1.5, 1.5, 0.5],
        physicsClientId=PYB_CLIENT,
    )
    p.addUserDebugText("GOAL", [3, 3, 1.3],
        textColorRGB=[0, 1, 0], textSize=1.5, physicsClientId=PYB_CLIENT)
    p.addUserDebugText("START", [0, 0, 1.3],
        textColorRGB=[1, 0, 0], textSize=1.5, physicsClientId=PYB_CLIENT)

    # Video recording — requires GUI mode; p.STATE_LOGGING_VIDEO_MP4 uses ffmpeg.
    VIDEO_PATH = os.path.join(RESULTS_DIR, "cbf_qp_3d.mp4")
    video_id = None
    if gui:
        video_id = p.startStateLogging(
            p.STATE_LOGGING_VIDEO_MP4, VIDEO_PATH, physicsClientId=PYB_CLIENT
        )

    ctrl = DSLPIDControl(drone_model=DroneModel.CF2X)

    log_file = open(LOG_PATH, "w", newline="")
    writer   = csv.writer(log_file)
    writer.writerow(
        ["t", "x", "y", "z", "dist_to_goal", "min_dist_to_obs",
         "step_count", "qp_feasible"]
    )

    action       = np.zeros((1, 4))
    max_steps    = int(DURATION_S * CTRL_FREQ)
    status       = f"TIMEOUT after {max_steps} steps"
    infeas_count = 0
    # Warm-start: initialise v_prev to nominal direction at full speed
    v_prev = _compute_v_nom(START_POS)
    START  = time.time()

    for step in range(max_steps):
        obs_env, _, _, _, _ = env.step(action)
        state = obs_env[0]
        pos   = state[0:3].copy()

        dist_to_goal    = float(np.linalg.norm(pos - GOAL_POS))
        min_dist_to_obs = float(_min_surface_dist(pos))

        # ── Termination ───────────────────────────────────────────────────────
        if dist_to_goal < GOAL_TOL:
            writer.writerow(
                [step * DT, *pos, dist_to_goal, min_dist_to_obs, step, True]
            )
            status = f"SUCCESS in {step} steps"
            break
        if min_dist_to_obs < CRASH_TOL:
            writer.writerow(
                [step * DT, *pos, dist_to_goal, min_dist_to_obs, step, False]
            )
            status = f"FAILED (collision) at step {step}"
            break

        # ── CBF-QP ────────────────────────────────────────────────────────────
        v_nom             = _compute_v_nom(pos)
        v_safe, feasible  = _cbf_qp(pos, v_nom, v_prev)
        if not feasible:
            infeas_count += 1
        v_prev = v_safe   # warm start for next step

        # Normalised lookahead → target_pos for DSL-PID
        v_safe_norm = float(np.linalg.norm(v_safe))
        if v_safe_norm > 1e-6:
            target_pos = pos + (v_safe / v_safe_norm) * LOOKAHEAD
        else:
            target_pos = GOAL_POS

        # ── Low-level PID ─────────────────────────────────────────────────────
        action[0], _, _ = ctrl.computeControlFromState(
            control_timestep=env.CTRL_TIMESTEP,
            state=state,
            target_pos=target_pos,
        )

        # ── Log & print ───────────────────────────────────────────────────────
        writer.writerow(
            [step * DT, *pos, dist_to_goal, min_dist_to_obs, step, feasible]
        )
        print(
            f"step {step:4d} | "
            f"pos [{pos[0]:+.3f} {pos[1]:+.3f} {pos[2]:+.3f}] | "
            f"d_goal={dist_to_goal:.3f} | d_obs={min_dist_to_obs:.3f} | "
            f"{'OK' if feasible else 'INFEAS'}"
        )

        env.render()
        if gui:
            sync(step, START, env.CTRL_TIMESTEP)

    if video_id is not None:
        p.stopStateLogging(video_id, physicsClientId=PYB_CLIENT)

    log_file.close()
    env.close()

    print(f"\n{'=' * 60}")
    print(f"  Result          : {status}")
    print(f"  QP infeasible   : {infeas_count} steps")
    print(f"  Log             : {LOG_PATH}")
    if gui:
        print(f"  Video           : {VIDEO_PATH}")
    print(f"{'=' * 60}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="CBF-QP 3D navigation demo")
    parser.add_argument("--no-gui", action="store_true",
                        help="Run headless (no PyBullet window)")
    args = parser.parse_args()
    run(gui=not args.no_gui)
