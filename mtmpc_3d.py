"""mt-MPC single-drone 3D navigation.

Based on:
  Saccani et al. (2023). Multi-trajectory Model Predictive Control for Safe
  UAV Navigation. IEEE/RSJ IROS 2023.

Template: gym-pybullet-drones/examples/pid.py (CtrlAviary + DSLPIDControl).
Scenario identical to apf_3d.py and cbf_qp_3d.py for fair comparison.

QP solver: scipy.optimize.minimize / SLSQP (no extra dependencies).

Usage:
    conda activate drones
    python experiments/mtmpc_3d.py [--no-gui]

# REPRODUCIBILITY NOTES - mt-MPC 3D
#
# mt-MPC core idea (Saccani 2023):
#   Maintain TWO simultaneous MPC trajectories from the current state:
#     Trajectory I  ("exploiting"): minimize distance to final goal.
#     Trajectory II ("exploring"):  independent safe backup trajectory.
#   Both trajectories satisfy the safe polytope at every horizon step.
#   KEY CONSTRAINT: U_I[:,0] == U_II[:,0]  — shared first velocity command.
#   This coupling guarantees recursive feasibility via the "backup trajectory"
#   argument: shifting the jointly-feasible pair by one step stays feasible.
#
# Velocity-space formulation (relative degree 1 w.r.t. position):
#   State: p = [px, py]  (2D position in horizontal plane)
#   Input: v = [vx, vy]  (velocity command, directly constrained)
#   p_{k+1} = p_k + v_k * DT_MPC  (Euler integration at planning timescale)
#
#   Using velocity as input (vs. acceleration in the original paper) avoids
#   the velocity-accumulation instability that arises when the DSL-PID inner
#   loop tracks slightly faster than the double-integrator model predicts.
#   The resulting "safe velocity" is semantically identical to the CBF-QP
#   output, but the N-step lookahead adds predictive obstacle awareness.
#
# Planning timescale DT_MPC = 0.25 s (12× control DT):
#   One horizon step = 12 control steps.  Lookahead = N * V_MAX * DT_MPC
#   = 8 * 0.5 * 0.25 = 1.0 m, large enough to predict obstacle encounters.
#
# Safe polytope (linearized at current position, rebuilt each step):
#   Obstacle half-spaces: A_obs @ [px,py] <= b_obs.
#   Tightening on OBSTACLE rows only: b_k = b_obs - k * ZETA.
#   Box [0,4]^2 applied without tightening.
#
# Temporary Target Shifting (TTS):
#   If goal falls outside tightened obstacle polytope at step N, project it
#   to the nearest feasible point (uses SLSQP sub-problem).
#
# LOOKAHEAD bridge to DSL-PID (same as apf_3d / cbf_qp_3d):
#   target_pos = pos + normalize([v_safe_x, v_safe_y, 0]) * 0.10 m
#   DSL-PID tracks this 0.10 m carrot and drives the drone at ~0.4 m/s.
#
# Condensed formulation:
#   Variables: [V_I_flat (2*N), V_II_flat (2*N)] = 4*N = 32 for N=8.
#   Constraints: 2 equality (shared first action) + inequalities.
#   Warm start: zero for first call (hover, always feasible), shifted
#   previous solution thereafter.
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

# ── Scenario (identical to apf_3d.py / cbf_qp_3d.py) ────────────────────────
START_POS = np.array([0.0, 0.0, 1.0])
GOAL_POS  = np.array([3.0, 3.0, 1.0])

OBSTACLES = [
    [1.0, 1.0, 0.0, 0.3],
    [2.0, 1.5, 0.0, 0.3],
    [1.5, 2.5, 0.0, 0.3],
]
OBS_HEIGHT = 2.0

# ── mt-MPC Parameters ─────────────────────────────────────────────────────────
MPC_N         = 15     # prediction horizon (planning steps)
DT_MPC        = 0.10   # planning timescale [s] (≈5 control steps at 48 Hz)
               # reach = N * V_MAX * DT_MPC = 15 * 1.0 * 0.10 = 1.5 m

V_MAX         = 1.0    # velocity bound [m/s] — MPC model only; PID speed ~0.4 m/s via LOOKAHEAD
SAFETY_MARGIN = 0.20   # extra buffer beyond obstacle radius [m]
ZETA          = 0.003  # Minkowski tightening per planning step (obstacle rows only) [m]
               # total tightening = N * ZETA = 15 * 0.003 = 0.045 m
               # obs1–obs2 corridor gap = 1.118 - 2*(0.50+0.045) = 0.028 m (open)
LAMBDA_U      = 0.10   # velocity effort weight
LAMBDA_TERM   = 6.0    # terminal cost weight
LOOKAHEAD     = 0.40   # PID carrot distance [m]

_OBS_SAFE_R = [r + SAFETY_MARGIN for _, _, _, r in OBSTACLES]

# ── Simulation ────────────────────────────────────────────────────────────────
CTRL_FREQ  = 48
SIM_FREQ   = 240
DT         = 1.0 / CTRL_FREQ
DURATION_S = 90

# ── Termination ───────────────────────────────────────────────────────────────
GOAL_TOL  = 0.15
CRASH_TOL = 0.05

# ── Logging ───────────────────────────────────────────────────────────────────
_HERE       = os.path.dirname(os.path.abspath(__file__))
RESULTS_DIR = os.path.join(_HERE, "results")
LOG_PATH    = os.path.join(RESULTS_DIR, "mtmpc_3d_log.csv")


# ── MtMPCPlanner3D ────────────────────────────────────────────────────────────

class MtMPCPlanner3D:
    """Two-trajectory velocity-space MPC planner (horizontal XY plane).

    Model: p_{k+1} = p_k + v_k * DT_MPC  (v is the control input).
    Variables: [V_I_flat (2*N), V_II_flat (2*N)] = 4*N = 32 for N=8.
    """

    def __init__(self):
        self.N = MPC_N
        self._V_I_prev  = None   # (2, N) warm-start cache
        self._V_II_prev = None
        self._last_v    = np.zeros(2)   # last applied velocity command (fallback)

    # ── Forward rollout ───────────────────────────────────────────────────────

    def _rollout(self, p0: np.ndarray, V: np.ndarray) -> np.ndarray:
        """Position rollout from p0 under velocity sequence V (2 x N).
        Returns P (2 x N+1)."""
        N = V.shape[1]
        P = np.zeros((2, N + 1))
        P[:, 0] = p0
        for k in range(N):
            P[:, k + 1] = P[:, k] + V[:, k] * DT_MPC
        return P

    # ── Obstacle half-space polytope ──────────────────────────────────────────

    def _build_obs_polytope(self, pos_xy: np.ndarray):
        """Linearized obstacle half-spaces: A_obs @ [px,py] <= b_obs."""
        rows_A, rows_b = [], []
        for (cx, cy, _, _r), R_i in zip(OBSTACLES, _OBS_SAFE_R):
            dx = float(pos_xy[0] - cx)
            dy = float(pos_xy[1] - cy)
            dist = float(np.sqrt(dx * dx + dy * dy))
            if dist < 1e-4:
                continue
            nx, ny = dx / dist, dy / dist
            rows_A.append([-nx, -ny])
            rows_b.append(-(R_i + nx * cx + ny * cy))
        if not rows_A:
            return np.zeros((0, 2)), np.zeros(0)
        return np.array(rows_A), np.array(rows_b)

    # ── TTS ───────────────────────────────────────────────────────────────────

    def _ray_clip(self, pos_xy: np.ndarray, direction: np.ndarray,
                  A_obs: np.ndarray, b_obs_N: np.ndarray) -> float:
        """Farthest λ s.t. pos + λ*direction satisfies A @ p <= b_N."""
        lambda_max = 1e6
        for i in range(len(b_obs_N)):
            denom = float(A_obs[i] @ direction)
            if denom > 1e-9:
                lam = (float(b_obs_N[i]) - float(A_obs[i] @ pos_xy)) / denom
                if lam > 0:
                    lambda_max = min(lambda_max, lam)
        return lambda_max if lambda_max < 1e5 else 1e6

    def _shift_target_ray(self, pos_xy: np.ndarray, goal_xy: np.ndarray,
                          A_obs: np.ndarray, b_obs_N: np.ndarray):
        """Ray-project: return farthest reachable point toward goal within polytope.

        Returns (p_eff [2,], activated [bool]).
        """
        d = goal_xy - pos_xy
        d_norm = float(np.linalg.norm(d))
        if d_norm < 1e-6:
            return goal_xy.copy(), False

        direction = d / d_norm
        lambda_max = self._ray_clip(pos_xy, direction, A_obs, b_obs_N)

        # If the primary ray is blocked very early (<0.3 m), swing ±45°
        if lambda_max < 0.3:
            perp = np.array([-direction[1], direction[0]])
            for side in [+1.0, -1.0]:
                lat_dir = 0.7 * direction + 0.7 * side * perp
                lat_dir /= float(np.linalg.norm(lat_dir))
                lam_lat = self._ray_clip(pos_xy, lat_dir, A_obs, b_obs_N)
                if lam_lat > lambda_max:
                    lambda_max = lam_lat
                    direction  = lat_dir

        # Cap at planning horizon reach: prevents p_eff from overshooting to
        # infinity when no constraint clips the lateral direction (lam=1e6).
        reach = MPC_N * V_MAX * DT_MPC   # = 1.5 m
        lambda_use = min(lambda_max * 0.95, reach, d_norm)
        p_eff      = pos_xy + lambda_use * direction
        activated  = lambda_use < d_norm - 1e-6
        return p_eff, activated

    # ── Main plan step ────────────────────────────────────────────────────────

    def plan(self, pos_xy: np.ndarray, p_target: np.ndarray):
        """Solve mt-MPC QP.

        Returns (v_safe [2,], feasible [bool], tts_activated [bool], solve_ms, p_eff [2,]).
        v_safe is the first velocity command [vx, vy].
        p_eff is the effective (possibly TTS-projected) goal used by the optimizer.
        """
        N = self.N
        p0 = pos_xy.copy()

        A_obs, b_obs = self._build_obs_polytope(pos_xy)
        n_obs = A_obs.shape[0]

        # TTS: ray-project toward goal, clipped at the tightened polytope boundary.
        # Gives the farthest reachable point in the goal direction so the drone
        # has a long carrot to chase rather than stopping at the nearest boundary.
        b_obs_N = b_obs - N * ZETA
        p_eff, tts_activated = self._shift_target_ray(p0, p_target, A_obs, b_obs_N)

        n_v = 2

        def unpack(z):
            V_I  = z[:n_v * N].reshape(n_v, N)
            V_II = z[n_v * N:].reshape(n_v, N)
            return V_I, V_II

        # ── Objective ─────────────────────────────────────────────────────
        def obj(z):
            V_I, V_II = unpack(z)
            P_I = self._rollout(p0, V_I)
            cost = 0.0
            for k in range(1, N + 1):
                d = P_I[:, k] - p_eff
                cost += float(np.dot(d, d))
            d_N = P_I[:, N] - p_eff
            cost += LAMBDA_TERM * float(np.dot(d_N, d_N))
            cost += LAMBDA_U * (float(np.sum(V_I ** 2)) + float(np.sum(V_II ** 2)))
            return cost

        # ── Equality: shared first velocity command ────────────────────────
        def eq_shared(z):
            V_I, V_II = unpack(z)
            return V_I[:, 0] - V_II[:, 0]

        # ── Inequalities ──────────────────────────────────────────────────
        def ineq_all(z):
            V_I, V_II = unpack(z)
            P_I  = self._rollout(p0, V_I)
            P_II = self._rollout(p0, V_II)
            res = []

            for k in range(1, N + 1):
                # Box constraints (no tightening)
                for P in (P_I, P_II):
                    px, py = P[0, k], P[1, k]
                    res.extend([px, 4.0 - px, py, 4.0 - py])
                # Obstacle constraints (tightened by k * ZETA)
                if n_obs > 0:
                    b_k = b_obs - k * ZETA
                    res.extend(list(b_k - A_obs @ P_I[:, k]))
                    res.extend(list(b_k - A_obs @ P_II[:, k]))

            # Velocity magnitude bounds on all inputs
            for k in range(N):
                for V in (V_I, V_II):
                    vx, vy = V[0, k], V[1, k]
                    res.extend([V_MAX - vx, V_MAX + vx,
                                V_MAX - vy, V_MAX + vy])

            return np.array(res, dtype=float)

        constraints = [
            {"type": "eq",   "fun": eq_shared},
            {"type": "ineq", "fun": ineq_all},
        ]

        # ── Warm start (hover = always feasible) ──────────────────────────
        if self._V_I_prev is not None:
            V_I0  = np.hstack([self._V_I_prev[:, 1:],  self._V_I_prev[:, -1:]])
            V_II0 = np.hstack([self._V_II_prev[:, 1:], self._V_II_prev[:, -1:]])
        else:
            V_I0  = np.zeros((n_v, N))
            V_II0 = np.zeros((n_v, N))

        z0 = np.concatenate([V_I0.flatten(), V_II0.flatten()])

        # ── Solve ─────────────────────────────────────────────────────────
        t0 = time.time()
        res_opt = minimize(
            fun=obj,
            x0=z0,
            method="SLSQP",
            constraints=constraints,
            options={"ftol": 1e-7, "maxiter": 400},
        )
        solve_ms = (time.time() - t0) * 1000.0

        feasible = res_opt.success or res_opt.status in (0, 1)

        if feasible and res_opt.x is not None:
            V_I, V_II = unpack(res_opt.x)
            self._V_I_prev  = V_I
            self._V_II_prev = V_II
            v_safe = V_I[:, 0].copy()
            self._last_v = v_safe
            return v_safe, True, tts_activated, solve_ms, p_eff

        # ── Fallback: steer toward effective target at reduced speed ───────
        self._V_I_prev  = None
        self._V_II_prev = None
        d = p_eff - pos_xy
        d_norm = max(float(np.linalg.norm(d)), 1e-6)
        # Use half speed toward TTS-shifted goal (conservative)
        v_safe = min(V_MAX * 0.4, float(np.linalg.norm(self._last_v)) * 0.5 + 0.01) * d / d_norm
        return v_safe, False, tts_activated, solve_ms, p_eff


# ── Helpers ───────────────────────────────────────────────────────────────────

def _min_surface_dist(pos: np.ndarray) -> float:
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
                                   rgbaColor=[0.6, 0.2, 0.8, 0.7],
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

    VIDEO_PATH = os.path.join(RESULTS_DIR, "mtmpc_3d.mp4")
    video_id = None
    if gui:
        video_id = p.startStateLogging(
            p.STATE_LOGGING_VIDEO_MP4, VIDEO_PATH, physicsClientId=PYB_CLIENT
        )

    ctrl    = DSLPIDControl(drone_model=DroneModel.CF2X)
    planner = MtMPCPlanner3D()

    log_file = open(LOG_PATH, "w", newline="")
    writer   = csv.writer(log_file)
    writer.writerow([
        "t", "x", "y", "z", "dist_to_goal", "min_dist_to_obs",
        "step_count", "qp_feasible", "fallback_count", "tts_activated", "solve_ms",
    ])

    action         = np.zeros((1, 4))
    max_steps      = int(DURATION_S * CTRL_FREQ)
    status         = f"TIMEOUT after {max_steps} steps"
    infeas_count   = 0
    fallback_count = 0
    tts_count      = 0
    START          = time.time()

    # Cache v_safe across ctrl steps so the LOOKAHEAD direction is stable
    v_safe_cached = np.zeros(2)
    tts_on        = False
    p_eff_dbg     = GOAL_POS[:2].copy()

    for step in range(max_steps):
        obs_env, _, _, _, _ = env.step(action)
        state = obs_env[0]
        pos   = state[0:3].copy()
        vel   = state[10:13].copy()

        dist_to_goal    = float(np.linalg.norm(pos - GOAL_POS))
        min_dist_to_obs = float(_min_surface_dist(pos))

        # ── Termination ───────────────────────────────────────────────────────
        if dist_to_goal < GOAL_TOL:
            writer.writerow([
                step * DT, *pos, dist_to_goal, min_dist_to_obs,
                step, True, fallback_count, False, 0.0,
            ])
            status = f"SUCCESS in {step} steps"
            break
        if min_dist_to_obs < CRASH_TOL:
            writer.writerow([
                step * DT, *pos, dist_to_goal, min_dist_to_obs,
                step, False, fallback_count, False, 0.0,
            ])
            status = f"FAILED (collision) at step {step}"
            break

        # ── mt-MPC (re-solve every ctrl step) ────────────────────────────────
        v_safe, feasible, tts_on, solve_ms, p_eff_dbg = planner.plan(pos[:2], GOAL_POS[:2])
        if not feasible:
            infeas_count   += 1
            fallback_count += 1
        if tts_on:
            tts_count += 1
        v_safe_cached = v_safe

        # Polytope diagnostic: print A, b, TTS state every 50 steps
        if step < 3 or step % 50 == 0:
            _A, _b = planner._build_obs_polytope(pos[:2])
            print(f"  [POLY step={step}] n_rows={len(_b)}")
            for i, ((cx,cy,_,_r), Ri) in enumerate(zip(OBSTACLES, _OBS_SAFE_R)):
                if len(_b) > i:
                    ap_goal = float(_A[i] @ GOAL_POS[:2])
                    print(f"    obs{i}@({cx:.1f},{cy:.1f}) R_safe={Ri:.2f}"
                          f" A@goal={ap_goal:.3f} b={_b[i]:.3f} viol={ap_goal>_b[i]}")
            if tts_on:
                print(f"    TTS ON -> p_eff=[{p_eff_dbg[0]:.3f},{p_eff_dbg[1]:.3f}]"
                      f" (goal=[{GOAL_POS[0]:.1f},{GOAL_POS[1]:.1f}])")

        # Normalized LOOKAHEAD → target_pos for DSL-PID
        v_norm = float(np.linalg.norm(v_safe_cached))
        if v_norm > 1e-6:
            dir_3d = np.array([v_safe_cached[0], v_safe_cached[1], 0.0]) / v_norm
            target_pos = pos + dir_3d * LOOKAHEAD
        else:
            # Stalled: aim directly at goal
            d = GOAL_POS - pos
            d_norm = max(float(np.linalg.norm(d[:2])), 1e-6)
            target_pos = pos + np.array([d[0], d[1], 0.0]) / d_norm * LOOKAHEAD
        target_pos[2] = GOAL_POS[2]

        # ── Low-level PID ─────────────────────────────────────────────────────
        action[0], _, _ = ctrl.computeControlFromState(
            control_timestep=env.CTRL_TIMESTEP,
            state=state,
            target_pos=target_pos,
        )

        # ── Log & print ───────────────────────────────────────────────────────
        writer.writerow([
            step * DT, *pos, dist_to_goal, min_dist_to_obs,
            step, feasible, fallback_count, tts_on, solve_ms,
        ])
        print(
            f"step {step:4d} | "
            f"pos [{pos[0]:+.3f} {pos[1]:+.3f} {pos[2]:+.3f}] | "
            f"d_goal={dist_to_goal:.3f} | d_obs={min_dist_to_obs:.3f} | "
            f"{'TTS' if tts_on else '   '} | "
            f"{'INFEAS' if not feasible else 'OK    '} | "
            f"{solve_ms:5.1f} ms"
        )

        env.render()
        if gui:
            sync(step, START, env.CTRL_TIMESTEP)

    if video_id is not None:
        p.stopStateLogging(video_id, physicsClientId=PYB_CLIENT)

    log_file.close()
    env.close()

    print(f"\n{'=' * 65}")
    print(f"  Result          : {status}")
    print(f"  QP infeasible   : {infeas_count} steps")
    print(f"  Fallback used   : {fallback_count} steps")
    total_steps = max_steps if "TIMEOUT" in status else int(status.split()[-2])
    print(f"  TTS activations : {tts_count} / {total_steps} ({100*tts_count/max(total_steps,1):.1f}%)")
    print(f"  Log             : {LOG_PATH}")
    if gui:
        print(f"  Video           : {VIDEO_PATH}")
    print(f"{'=' * 65}")
    print()
    print("  ── 3-way comparison ──────────────────────────────────────")
    print("  Controller   Steps    Notes")
    print("  APF          1246     Khatib 1986,      min_clearance 0.37 m")
    print("  CBF-QP       1074     Singletary 2020,  min_clearance 0.12 m")
    step_val = status.split()[-2] if "SUCCESS" in status else "N/A"
    print(f"  mt-MPC       {step_val:<8s} Saccani 2023,    see log for clearance")
    print(f"{'=' * 65}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="mt-MPC 3D navigation demo")
    parser.add_argument("--no-gui", action="store_true",
                        help="Run headless (no PyBullet window)")
    args = parser.parse_args()
    run(gui=not args.no_gui)
