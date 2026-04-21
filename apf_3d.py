"""APF (Artificial Potential Field) single-drone 3D navigation.

Based on:
  Khatib, O. (1986). Real-time obstacle avoidance for manipulators and mobile robots.
  IJRR 5(1), 90-98.

Template: gym-pybullet-drones/examples/pid.py (CtrlAviary + DSLPIDControl).

Usage:
    conda activate drones
    python experiments/apf_3d.py [--no-gui]

# REPRODUCIBILITY NOTES
# 2D->3D changes from Khatib 1986:
# - Force now 3D vector (x,y,z); z-component keeps altitude stable via
#   attractive force when goal and start share the same z.
# - rho measured to cylinder surface (not center) to match 2D formulation:
#   rho = sqrt((px-cx)^2 + (py-cy)^2) - r  (horizontal only, z-invariant)
# - target_pos clipped to MAX_STEP to prevent large jumps that exceed
#   DSLPIDControl's tracking bandwidth.
# - k_att_close added near goal to prevent overshoot (not in original).
# Parameter choices:
# - k_rep=0.5: larger values cause oscillation near obstacles (tested).
# - rho_0=0.8: ~2.5x obstacle radius; smaller causes late reaction.
# Local minima risk: documented when goal is directly behind obstacle.
"""

import os
import csv
import time
import argparse

import numpy as np
import pybullet as p

from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.utils import sync

# ── Scenario ─────────────────────────────────────────────────────────────────
START_POS = np.array([0.0, 0.0, 1.0])
GOAL_POS  = np.array([3.0, 3.0, 1.0])

# [cx, cy, cz_base, radius]  — cz_base is the bottom of the cylinder
OBSTACLES = [
    [1.0, 1.0, 0.0, 0.3],
    [2.0, 1.5, 0.0, 0.3],
    [1.5, 2.5, 0.0, 0.3],
]
OBS_HEIGHT = 2.0

# ── APF Parameters ────────────────────────────────────────────────────────────
K_ATT       = 1.0   # attractive gain (far from goal)
K_ATT_CLOSE = 0.5   # attractive gain (near goal) — reduces overshoot
K_REP       = 0.5   # repulsive gain
RHO_0       = 0.8   # influence radius from obstacle surface [m]
THRESHOLD   = 0.5   # switch to K_ATT_CLOSE inside this distance [m]
# Lookahead: target_pos is placed LOOKAHEAD metres ahead along the APF gradient.
# This gives the PID a fixed-size carrot regardless of force magnitude, which
# drives the drone at its natural tracking speed instead of tiny F*dt increments.
# 0.10 m chosen empirically: large enough for responsive PID tracking (~0.4 m/s),
# small enough that the drone doesn't overshoot the narrow corridors between
# obstacles. 0.30 m caused the drone to enter obstacle influence zones too
# aggressively and get trapped near obs3.
LOOKAHEAD   = 0.10  # lookahead distance along normalised APF direction [m]

# ── Simulation ────────────────────────────────────────────────────────────────
CTRL_FREQ  = 48
SIM_FREQ   = 240
DURATION_S = 60     # extended: APF path ~5 m detour around 3 obstacles
DT         = 1.0 / CTRL_FREQ

# ── Termination ───────────────────────────────────────────────────────────────
GOAL_TOL  = 0.15   # success: dist_to_goal < GOAL_TOL [m]
CRASH_TOL = 0.05   # failure: min surface dist < CRASH_TOL [m]

# ── Logging ───────────────────────────────────────────────────────────────────
_HERE        = os.path.dirname(os.path.abspath(__file__))
RESULTS_DIR  = os.path.join(_HERE, "results")
LOG_PATH     = os.path.join(RESULTS_DIR, "apf_3d_log.csv")


# ── APF Functions ─────────────────────────────────────────────────────────────

def _apf_force(pos: np.ndarray) -> np.ndarray:
    """Compute APF total force vector (3,) at current position."""
    diff = pos - GOAL_POS
    dist = np.linalg.norm(diff)

    if dist > THRESHOLD:
        F_att = -K_ATT * diff
    else:
        F_att = -K_ATT_CLOSE * diff

    F_rep = np.zeros(3)
    for cx, cy, _, r in OBSTACLES:
        dx = pos[0] - cx
        dy = pos[1] - cy
        dist_xy = np.sqrt(dx * dx + dy * dy)
        rho = max(dist_xy - r, 1e-4)   # surface distance; clamped > 0

        if rho <= RHO_0 and dist_xy > 1e-6:
            direction = np.array([dx / dist_xy, dy / dist_xy, 0.0])
            F_rep += K_REP * (1.0 / rho - 1.0 / RHO_0) * (1.0 / rho ** 2) * direction

    return F_att + F_rep


def _min_surface_dist(pos: np.ndarray) -> float:
    """Minimum horizontal distance from pos to any obstacle surface."""
    return min(
        np.sqrt((pos[0] - cx) ** 2 + (pos[1] - cy) ** 2) - r
        for cx, cy, _, r in OBSTACLES
    )


# ── Obstacle Loading ──────────────────────────────────────────────────────────

def _load_obstacles(client: int) -> list:
    """Spawn cylinder obstacles into the PyBullet world. Returns body ID list."""
    ids = []
    for cx, cy, cz_base, r in OBSTACLES:
        cz_center = cz_base + OBS_HEIGHT / 2.0
        col = p.createCollisionShape(
            p.GEOM_CYLINDER, radius=r, height=OBS_HEIGHT,
            physicsClientId=client,
        )
        vis = p.createVisualShape(
            p.GEOM_CYLINDER, radius=r, length=OBS_HEIGHT,
            rgbaColor=[1.0, 0.3, 0.1, 0.7],
            physicsClientId=client,
        )
        body = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=col,
            baseVisualShapeIndex=vis,
            basePosition=[cx, cy, cz_center],
            physicsClientId=client,
        )
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
        obstacles=False,       # custom obstacles loaded below
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
    VIDEO_PATH = os.path.join(RESULTS_DIR, "apf_3d.mp4")
    video_id = None
    if gui:
        video_id = p.startStateLogging(
            p.STATE_LOGGING_VIDEO_MP4, VIDEO_PATH, physicsClientId=PYB_CLIENT
        )

    ctrl = DSLPIDControl(drone_model=DroneModel.CF2X)

    log_file = open(LOG_PATH, "w", newline="")
    writer = csv.writer(log_file)
    writer.writerow(["t", "x", "y", "z", "dist_to_goal", "min_dist_to_obs", "step_count"])

    action = np.zeros((1, 4))
    max_steps = int(DURATION_S * CTRL_FREQ)
    status = f"TIMEOUT after {max_steps} steps"
    START = time.time()

    for step in range(max_steps):
        obs, _, _, _, _ = env.step(action)
        state = obs[0]                    # (20,) state vector
        pos   = state[0:3].copy()         # current XYZ position [m]

        dist_to_goal    = float(np.linalg.norm(pos - GOAL_POS))
        min_dist_to_obs = float(_min_surface_dist(pos))

        # ── Termination ───────────────────────────────────────────────────────
        if dist_to_goal < GOAL_TOL:
            writer.writerow([step * DT, *pos, dist_to_goal, min_dist_to_obs, step])
            status = f"SUCCESS in {step} steps"
            break
        if min_dist_to_obs < CRASH_TOL:
            writer.writerow([step * DT, *pos, dist_to_goal, min_dist_to_obs, step])
            status = f"FAILED (collision) at step {step}"
            break

        # ── APF: next reference position ──────────────────────────────────────
        # Normalise force direction and step a fixed LOOKAHEAD distance so the
        # PID position loop has a meaningful error to chase (not a tiny F*dt).
        F = _apf_force(pos)
        F_norm = float(np.linalg.norm(F))
        if F_norm > 1e-6:
            target_pos = pos + (F / F_norm) * LOOKAHEAD
        else:
            target_pos = GOAL_POS   # at a local minimum: aim directly at goal

        # ── Low-level PID ─────────────────────────────────────────────────────
        action[0], _, _ = ctrl.computeControlFromState(
            control_timestep=env.CTRL_TIMESTEP,
            state=state,
            target_pos=target_pos,
        )

        # ── Log & print ───────────────────────────────────────────────────────
        writer.writerow([step * DT, *pos, dist_to_goal, min_dist_to_obs, step])
        print(
            f"step {step:4d} | "
            f"pos [{pos[0]:+.3f} {pos[1]:+.3f} {pos[2]:+.3f}] | "
            f"d_goal={dist_to_goal:.3f} | d_obs={min_dist_to_obs:.3f}"
        )

        env.render()
        if gui:
            sync(step, START, env.CTRL_TIMESTEP)

    if video_id is not None:
        p.stopStateLogging(video_id, physicsClientId=PYB_CLIENT)

    log_file.close()
    env.close()

    print(f"\n{'=' * 52}")
    print(f"  Result : {status}")
    print(f"  Log    : {LOG_PATH}")
    if gui:
        print(f"  Video  : {VIDEO_PATH}")
    print(f"{'=' * 52}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="APF 3D navigation demo")
    parser.add_argument("--no-gui", action="store_true",
                        help="Run headless (no PyBullet window)")
    args = parser.parse_args()
    run(gui=not args.no_gui)
