"""fair_comparison.py — Unified experiment runner for APF / CBF-QP / mt-MPC.

Runs one trial headless (or with GUI) and appends one row to
experiments/results/fair_comparison_master.csv.

Usage:
    python fair_comparison.py --scenario B --controller mtmpc \\
        --vmax 1.0 --lookahead 0.40 --trial 0 --no-gui

Controller logic is copied and parameterised from the three standalone scripts
(apf_3d.py, cbf_qp_3d.py, mtmpc_3d.py).  Those files are NOT imported or
modified.
"""

import os
import csv
import time
import math
import argparse

import numpy as np
import pybullet as p
from scipy.optimize import minimize

from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.utils import sync

import scenarios as SC

# ── Shared constants ──────────────────────────────────────────────────────────
START_POS  = np.array(SC.START_POS, dtype=float)
GOAL_POS   = np.array(SC.GOAL_POS,  dtype=float)
OBS_HEIGHT = SC.OBS_HEIGHT

CTRL_FREQ  = 48
SIM_FREQ   = 240
DT         = 1.0 / CTRL_FREQ

GOAL_TOL   = 0.15   # success threshold [m]
CRASH_TOL  = 0.05   # collision threshold [m]
MAX_STEPS  = 2500   # per-trial step cap (≈52 s)

_HERE       = os.path.dirname(os.path.abspath(__file__))
RESULTS_DIR = os.path.join(_HERE, "results")
MASTER_CSV  = os.path.join(RESULTS_DIR, "fair_comparison_master.csv")

_CSV_HEADER = [
    "scenario", "controller", "vmax", "lookahead", "trial", "seed",
    "outcome", "steps", "path_length", "min_clearance", "avg_speed",
    "max_speed", "time_to_goal_s", "safety_violation_magnitude",
    "qp_feasibility_rate", "tts_rate", "fallback_count",
    "avg_solve_ms", "max_solve_ms",
]

# ─────────────────────────────────────────────────────────────────────────────
# Shared helpers
# ─────────────────────────────────────────────────────────────────────────────

def _min_surface_dist(pos: np.ndarray, obstacles: list) -> float:
    """Horizontal distance from pos to nearest obstacle surface."""
    return min(
        float(np.sqrt((pos[0] - cx)**2 + (pos[1] - cy)**2)) - r
        for cx, cy, _, r in obstacles
    )


def _load_obstacles(client: int, obstacles: list, color_rgba) -> list:
    """Spawn collision + visual cylinders. Returns body-ID list."""
    ids = []
    for cx, cy, cz_b, r in obstacles:
        cz_c = cz_b + OBS_HEIGHT / 2.0
        col = p.createCollisionShape(p.GEOM_CYLINDER, radius=r,
                                     height=OBS_HEIGHT, physicsClientId=client)
        vis = p.createVisualShape(p.GEOM_CYLINDER, radius=r, length=OBS_HEIGHT,
                                  rgbaColor=color_rgba, physicsClientId=client)
        body = p.createMultiBody(baseMass=0,
                                 baseCollisionShapeIndex=col,
                                 baseVisualShapeIndex=vis,
                                 basePosition=[cx, cy, cz_c],
                                 physicsClientId=client)
        ids.append(body)
    return ids


def _spawn_marker(client: int, pos, color_rgba,
                  radius: float = 0.10, height: float = 0.05) -> int:
    """Collision-less visual marker cylinder."""
    vis = p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=height,
                              rgbaColor=color_rgba, physicsClientId=client)
    return p.createMultiBody(baseMass=0,
                             baseCollisionShapeIndex=-1,   # NO collision
                             baseVisualShapeIndex=vis,
                             basePosition=list(pos),
                             physicsClientId=client)


def _append_csv(row: dict):
    """Append one row to master CSV, creating header if needed."""
    os.makedirs(RESULTS_DIR, exist_ok=True)
    write_header = not os.path.exists(MASTER_CSV)
    with open(MASTER_CSV, "a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=_CSV_HEADER)
        if write_header:
            writer.writeheader()
        writer.writerow(row)


# ─────────────────────────────────────────────────────────────────────────────
# APF Controller
# ─────────────────────────────────────────────────────────────────────────────

class APFController:
    """Khatib 1986 artificial potential field."""

    K_ATT       = 1.0
    K_ATT_CLOSE = 0.5
    K_REP       = 0.5
    RHO_0       = 0.8
    THRESHOLD   = 0.5

    def __init__(self, obstacles: list, goal_pos: np.ndarray, lookahead: float):
        self.obstacles = obstacles
        self.goal_pos  = goal_pos.copy()
        self.lookahead = lookahead

    def compute_target(self, pos: np.ndarray):
        """Returns (target_pos [3,], extras dict)."""
        F      = self._force(pos)
        F_norm = float(np.linalg.norm(F))
        target = (pos + (F / F_norm) * self.lookahead
                  if F_norm > 1e-6 else self.goal_pos.copy())
        return target, {}

    def _force(self, pos: np.ndarray) -> np.ndarray:
        diff = pos - self.goal_pos
        dist = float(np.linalg.norm(diff))
        k    = self.K_ATT_CLOSE if dist <= self.THRESHOLD else self.K_ATT
        F_att = -k * diff

        F_rep = np.zeros(3)
        for cx, cy, _, r in self.obstacles:
            dx      = pos[0] - cx
            dy      = pos[1] - cy
            dist_xy = float(np.sqrt(dx*dx + dy*dy))
            rho     = max(dist_xy - r, 1e-4)
            if rho <= self.RHO_0 and dist_xy > 1e-6:
                direction = np.array([dx/dist_xy, dy/dist_xy, 0.0])
                F_rep += (self.K_REP
                          * (1.0/rho - 1.0/self.RHO_0)
                          * (1.0/rho**2)
                          * direction)
        return F_att + F_rep


# ─────────────────────────────────────────────────────────────────────────────
# CBF-QP Controller
# ─────────────────────────────────────────────────────────────────────────────

class CBFController:
    """Singletary 2020 velocity-space CBF-QP."""

    ALPHA = 1.0
    SAFETY_MARGIN = 0.15
    K_ATT = 1.0

    def __init__(self, obstacles: list, goal_pos: np.ndarray,
                 v_max: float, lookahead: float):
        self.obstacles   = obstacles
        self.obs_safe_r  = [r + self.SAFETY_MARGIN for _, _, _, r in obstacles]
        self.goal_pos    = goal_pos.copy()
        self.v_max       = v_max
        self.lookahead   = lookahead
        self._v_prev     = None

    def compute_target(self, pos: np.ndarray):
        v_nom = self._v_nom(pos)
        if self._v_prev is None:
            self._v_prev = v_nom.copy()
        v_safe, feasible = self._qp(pos, v_nom, self._v_prev)
        self._v_prev = v_safe
        v_norm = float(np.linalg.norm(v_safe))
        target = (pos + (v_safe / v_norm) * self.lookahead
                  if v_norm > 1e-6 else self.goal_pos.copy())
        return target, {"feasible": feasible, "v_safe": v_safe}

    def _v_nom(self, pos: np.ndarray) -> np.ndarray:
        d    = self.goal_pos - pos
        dist = max(float(np.linalg.norm(d)), 0.01)
        v    = self.K_ATT * d / dist
        spd  = float(np.linalg.norm(v))
        if spd > self.v_max:
            v = v * (self.v_max / spd)
        return v

    def _qp(self, pos: np.ndarray, v_nom: np.ndarray,
            v0: np.ndarray):
        A_list, b_list = [], []
        for (cx, cy, _, _r), R_i in zip(self.obstacles, self.obs_safe_r):
            dx  = float(pos[0] - cx)
            dy  = float(pos[1] - cy)
            h_i = dx*dx + dy*dy - R_i*R_i
            A_list.append([2.0*dx, 2.0*dy, 0.0])
            b_list.append(-self.ALPHA * h_i)
        A = np.array(A_list)
        b = np.array(b_list)
        vm = self.v_max

        if (np.all(A @ v_nom >= b - 1e-9)
                and float(np.dot(v_nom, v_nom)) <= vm**2 + 1e-9):
            return v_nom.copy(), True

        constraints = [
            {"type": "ineq",
             "fun":  lambda v, A=A, b=b: A @ v - b,
             "jac":  lambda v, A=A: A},
            {"type": "ineq",
             "fun":  lambda v: vm**2 - float(np.dot(v, v)),
             "jac":  lambda v: -2.0*v},
        ]
        res = minimize(
            fun=lambda v: float(np.dot(v - v_nom, v - v_nom)),
            jac=lambda v: 2.0*(v - v_nom),
            x0=v0, method="SLSQP",
            constraints=constraints,
            options={"ftol": 1e-8, "maxiter": 200},
        )
        feasible = res.success or res.status in (0, 1)
        v_safe   = (res.x if feasible and res.x is not None
                    else v_nom * min(1.0, vm / max(float(np.linalg.norm(v_nom)), 1e-6)))
        return v_safe, feasible


# ─────────────────────────────────────────────────────────────────────────────
# mt-MPC Controller
# ─────────────────────────────────────────────────────────────────────────────

# Fixed mt-MPC structural parameters (ablation: only V_MAX & LOOKAHEAD vary)
_MPC_N        = 15
_DT_MPC       = 0.10
_ZETA         = 0.003
_SAFETY_MARGIN_MPC = 0.20
_LAMBDA_U     = 0.10
_LAMBDA_TERM  = 6.0


class MtMPCController:
    """Two-trajectory velocity-space MPC planner (Saccani 2023).

    Structural parameters fixed for ablation purity:
      N=15, DT_MPC=0.10, ZETA=0.003, SAFETY_MARGIN=0.20, λ_u=0.10, λ_term=6.0.
    Only v_max and lookahead are swept.
    """

    def __init__(self, obstacles: list, goal_pos: np.ndarray,
                 v_max: float, lookahead: float):
        self.obstacles   = obstacles
        self.obs_safe_r  = [r + _SAFETY_MARGIN_MPC for _, _, _, r in obstacles]
        self.goal_pos    = goal_pos.copy()
        self.v_max       = v_max
        self.lookahead   = lookahead
        self._V_I_prev      = None
        self._V_II_prev     = None
        self._X_II_prev     = None   # Stage 1: persist backup position trajectory
        self._last_v        = np.zeros(2)
        self._prev_feasible = True   # Stage 4: routes warm-start choice

    # ── geometry ─────────────────────────────────────────────────────────────

    def _rollout(self, p0, V):
        N = V.shape[1]
        P = np.zeros((2, N+1))
        P[:, 0] = p0
        for k in range(N):
            P[:, k+1] = P[:, k] + V[:, k] * _DT_MPC
        return P

    def _build_obs_polytope(self, pos_xy):
        rows_A, rows_b = [], []
        for (cx, cy, _, _r), R_i in zip(self.obstacles, self.obs_safe_r):
            dx   = float(pos_xy[0] - cx)
            dy   = float(pos_xy[1] - cy)
            dist = float(np.sqrt(dx*dx + dy*dy))
            if dist < 1e-4:
                continue
            nx, ny = dx/dist, dy/dist
            rows_A.append([-nx, -ny])
            rows_b.append(-(R_i + nx*cx + ny*cy))
        if not rows_A:
            return np.zeros((0, 2)), np.zeros(0)
        return np.array(rows_A), np.array(rows_b)

    def _ray_clip(self, pos_xy, direction, A_obs, b_obs_N):
        lambda_max = 1e6
        for i in range(len(b_obs_N)):
            denom = float(A_obs[i] @ direction)
            if denom > 1e-9:
                lam = (float(b_obs_N[i]) - float(A_obs[i] @ pos_xy)) / denom
                if lam > 0:
                    lambda_max = min(lambda_max, lam)
        return lambda_max if lambda_max < 1e5 else 1e6

    def _shift_target_ray(self, pos_xy, goal_xy, A_obs, b_obs_N):
        d      = goal_xy - pos_xy
        d_norm = float(np.linalg.norm(d))
        if d_norm < 1e-6:
            return goal_xy.copy(), False

        direction  = d / d_norm
        lambda_max = self._ray_clip(pos_xy, direction, A_obs, b_obs_N)

        if lambda_max < 0.3:
            perp = np.array([-direction[1], direction[0]])
            for side in [+1.0, -1.0]:
                lat_dir = 0.7*direction + 0.7*side*perp
                lat_dir /= float(np.linalg.norm(lat_dir))
                lam_lat  = self._ray_clip(pos_xy, lat_dir, A_obs, b_obs_N)
                if lam_lat > lambda_max:
                    lambda_max = lam_lat
                    direction  = lat_dir

        reach      = _MPC_N * self.v_max * _DT_MPC
        lambda_use = min(lambda_max * 0.95, reach, d_norm)
        p_eff      = pos_xy + lambda_use * direction
        activated  = lambda_use < d_norm - 1e-6
        return p_eff, activated

    # ── plan ─────────────────────────────────────────────────────────────────

    def compute_target(self, pos: np.ndarray):
        """Returns (target_pos [3,], extras dict)."""
        t0 = time.time()
        v_safe, feasible, tts_on, solve_ms = self._plan(pos[:2])
        _ = solve_ms  # already measured inside _plan; recompute for extras
        solve_ms = (time.time() - t0) * 1000.0

        v_norm = float(np.linalg.norm(v_safe))
        if v_norm > 1e-6:
            dir_3d = np.array([v_safe[0], v_safe[1], 0.0]) / v_norm
            target = pos + dir_3d * self.lookahead
        else:
            # Stage 3 fix: v_safe ≈ 0 means the planner intends to HOVER
            # (either emergency hover, or Algorithm 2's shifted V_II has
            # fully decayed to hover). Target must stay at current position
            # so DSLPIDControl holds the drone in place. The old behaviour —
            # steering toward the goal — silently defeated the safety
            # guarantee by pushing the drone into the obstacle cluster
            # precisely when the QP gave up.
            target = pos.copy()
        target[2] = self.goal_pos[2]

        return target, {"feasible": feasible, "tts": tts_on, "solve_ms": solve_ms}

    def _plan(self, pos_xy):
        N  = _MPC_N
        p0 = pos_xy.copy()
        A_obs, b_obs = self._build_obs_polytope(pos_xy)
        n_obs        = A_obs.shape[0]
        b_obs_N      = b_obs - N * _ZETA

        p_eff, tts = self._shift_target_ray(p0, self.goal_pos[:2], A_obs, b_obs_N)
        n_v = 2
        vm  = self.v_max

        def unpack(z):
            return z[:n_v*N].reshape(n_v, N), z[n_v*N:].reshape(n_v, N)

        def obj(z):
            V_I, _ = unpack(z)
            P_I    = self._rollout(p0, V_I)
            cost   = 0.0
            for k in range(1, N+1):
                d = P_I[:, k] - p_eff
                cost += float(np.dot(d, d))
            d_N   = P_I[:, N] - p_eff
            cost += _LAMBDA_TERM * float(np.dot(d_N, d_N))
            V_I2, V_II2 = unpack(z)
            cost += _LAMBDA_U * (float(np.sum(V_I2**2)) + float(np.sum(V_II2**2)))
            return cost

        def eq_shared(z):
            V_I, V_II = unpack(z)
            return V_I[:, 0] - V_II[:, 0]

        def eq_term_hover(z):
            # Stage 2: safe trajectory must end at hover (last velocity command = 0).
            # Since velocity IS the control input, V_II[:, N-1] = 0 enforces both
            # "terminal input = 0" and "terminal velocity = 0" simultaneously.
            # Only applied to V_II (backup); V_I (exploiting) is unconstrained at terminal.
            _, V_II = unpack(z)
            return V_II[:, N-1]  # shape (2,), must == 0

        def ineq_all(z):
            V_I, V_II = unpack(z)
            P_I  = self._rollout(p0, V_I)
            P_II = self._rollout(p0, V_II)
            res  = []
            for k in range(1, N+1):
                for P in (P_I, P_II):
                    px, py = P[0, k], P[1, k]
                    res.extend([px, 4.0-px, py, 4.0-py])
                if n_obs > 0:
                    b_k = b_obs - k * _ZETA
                    res.extend(list(b_k - A_obs @ P_I[:, k]))
                    res.extend(list(b_k - A_obs @ P_II[:, k]))
            for k in range(N):
                for V in (V_I, V_II):
                    vx, vy = V[0, k], V[1, k]
                    res.extend([vm-vx, vm+vx, vm-vy, vm+vy])
            return np.array(res, dtype=float)

        constraints = [
            {"type": "eq",   "fun": eq_shared},
            {"type": "eq",   "fun": eq_term_hover},   # Stage 2: terminal hover on V_II
            {"type": "ineq", "fun": ineq_all},
        ]

        # Stage 4 warm-start strategy:
        #   After a SUCCESSFUL step: V_I0 = _V_I_prev (exploiting trajectory,
        #     already pre-shifted — best seed near the current operating point).
        #   After a FALLBACK step: V_I0 = _V_II_prev (safe hover plan, satisfies
        #     obstacle constraints by construction — feasible starting point so
        #     SLSQP avoids the expensive constraint-restoration phase that caused
        #     the 5+ s spikes). V_II0 always = _V_II_prev.
        if self._V_II_prev is not None:
            V_II0 = self._V_II_prev.copy()
            if self._prev_feasible and self._V_I_prev is not None:
                V_I0 = self._V_I_prev.copy()   # last step feasible → use exploiting traj
            else:
                V_I0 = V_II0.copy()            # recovering from fallback → hover seed
        else:
            V_I0 = V_II0 = np.zeros((n_v, N))

        z0 = np.concatenate([V_I0.flatten(), V_II0.flatten()])

        t0 = time.time()
        res_opt = minimize(obj, z0, method="SLSQP", constraints=constraints,
                           options={"ftol": 1e-7, "maxiter": 400})
        solve_ms = (time.time() - t0) * 1000.0

        feasible = res_opt.success or res_opt.status in (0, 1)
        if feasible and res_opt.x is not None:
            V_I, V_II = unpack(res_opt.x)
            X_II = self._rollout(p0, V_II)
            # Store SHIFTED versions: drop just-executed action (col 0),
            # append hover (zero velocity column) at the end. This maintains
            # the invariant that _V_II_prev[:, 0] is always "the command to
            # execute at the NEXT timestep if its QP fails".
            self._V_I_prev  = np.hstack([V_I[:,  1:], np.zeros((n_v, 1))])
            self._V_II_prev = np.hstack([V_II[:, 1:], np.zeros((n_v, 1))])
            # X_II shift: drop first position (current p0), keep through X_II[:, N]
            # and repeat last column (hover means position unchanged).
            self._X_II_prev = np.hstack([X_II[:, 1:], X_II[:, -1:]])
            v_safe = V_I[:, 0].copy()
            self._last_v = v_safe
            self._prev_feasible = True
            return v_safe, True, tts, solve_ms

        # ── Stage 3: Algorithm 2 fallback (Saccani 2023) ──────────────────────
        # QP infeasible: execute first action of previously stored safe plan,
        # then shift the plan forward by one step. Recursive feasibility is
        # guaranteed because the stored plan was proven feasible at the previous
        # step with N*ZETA tightening margin, and terminal hover ensures the
        # appended zero action keeps the drone at a safe position.
        if self._V_II_prev is not None and self._V_II_prev.shape[1] > 0:
            v_safe = self._V_II_prev[:, 0].copy()
            # Shift safe plan: drop executed action, append hover at the end.
            self._V_II_prev = np.hstack([
                self._V_II_prev[:, 1:],
                np.zeros((n_v, 1)),
            ])
            if self._X_II_prev is not None and self._X_II_prev.shape[1] > 1:
                self._X_II_prev = np.hstack([
                    self._X_II_prev[:, 1:],
                    self._X_II_prev[:, -1:],   # hover → position stays
                ])
            # Stage 4: do NOT shift _V_I_prev during fallback.
            # Keep it frozen at the last-successful exploiting trajectory —
            # a much better warm-start than the all-zeros that results from
            # shifting it 200+ times during a long infeasible sequence.
        else:
            # No prior safe trajectory (QP failed on very first timestep) —
            # emergency hover. Safe because we haven't moved yet.
            v_safe = np.zeros(n_v)
        self._last_v = v_safe
        self._prev_feasible = False
        return v_safe, False, tts, solve_ms


# ─────────────────────────────────────────────────────────────────────────────
# Main runner
# ─────────────────────────────────────────────────────────────────────────────

def run(scenario_key: str, controller_name: str, v_max: float,
        lookahead: float, trial: int, seed: int, gui: bool):
    """Run one trial and append a row to the master CSV."""
    obstacles = SC.SCENARIOS[scenario_key]
    rng = np.random.RandomState(seed)
    # Tiny positional noise (1 cm std) so trials differ slightly
    noise = rng.randn(3) * 0.01
    noise[2] = 0.0          # keep altitude fixed
    init_pos = START_POS + noise

    # ── Controller instantiation ──────────────────────────────────────────────
    if controller_name == "apf":
        ctrl_obj = APFController(obstacles, GOAL_POS, lookahead)
        color    = [1.0, 0.3, 0.1, 0.7]    # orange-red
    elif controller_name == "cbf":
        ctrl_obj = CBFController(obstacles, GOAL_POS, v_max, lookahead)
        color    = [0.2, 0.6, 1.0, 0.7]    # blue
    elif controller_name == "mtmpc":
        ctrl_obj = MtMPCController(obstacles, GOAL_POS, v_max, lookahead)
        color    = [0.6, 0.2, 0.8, 0.7]    # purple
    else:
        raise ValueError(f"Unknown controller: {controller_name!r}")

    # ── Env setup ─────────────────────────────────────────────────────────────
    os.makedirs(RESULTS_DIR, exist_ok=True)

    env = CtrlAviary(
        drone_model=DroneModel.CF2X,
        num_drones=1,
        initial_xyzs=init_pos.reshape(1, 3),
        initial_rpys=np.zeros((1, 3)),
        physics=Physics.PYB,
        pyb_freq=SIM_FREQ,
        ctrl_freq=CTRL_FREQ,
        gui=gui,
        record=False,
        obstacles=False,
        user_debug_gui=False,
    )
    client = env.getPyBulletClient()

    # ── Video recording (GUI mode only) ───────────────────────────────────────
    video_id   = None
    VIDEO_PATH = None
    if gui:
        _video_dir = os.path.join(RESULTS_DIR, "videos")
        os.makedirs(_video_dir, exist_ok=True)
        VIDEO_PATH = os.path.join(
            _video_dir,
            f"fc_{scenario_key}_{controller_name}_vmax{v_max}_la{lookahead}_t{trial}.mp4",
        )
        video_id = p.startStateLogging(
            p.STATE_LOGGING_VIDEO_MP4, VIDEO_PATH, physicsClientId=client
        )
    _load_obstacles(client, obstacles, color)

    # Ghost start/goal markers (no collision)
    _spawn_marker(client, [init_pos[0], init_pos[1], 1.0], [1, 0, 0, 0.8])
    _spawn_marker(client, [GOAL_POS[0], GOAL_POS[1], GOAL_POS[2]],
                  [0, 1, 0, 0.8])
    p.addUserDebugText("START", [init_pos[0], init_pos[1], 1.3],
                       textColorRGB=[1, 0, 0], textSize=1.2,
                       physicsClientId=client)
    p.addUserDebugText("GOAL", [GOAL_POS[0], GOAL_POS[1], 1.3],
                       textColorRGB=[0, 1, 0], textSize=1.2,
                       physicsClientId=client)
    p.resetDebugVisualizerCamera(
        cameraDistance=5.5,
        cameraYaw=240.6,
        cameraPitch=-78.6,
        cameraTargetPosition=[1.5, 1.5, 0.5],
        physicsClientId=client,
    )

    pid = DSLPIDControl(drone_model=DroneModel.CF2X)
    action = np.zeros((1, 4))

    # ── Per-step tracking vars ────────────────────────────────────────────────
    positions   = []
    min_clear   = math.inf
    max_speed_v = 0.0

    # mt-MPC-specific accumulators
    qp_ok_count  = 0
    tts_count    = 0
    fallback_cnt = 0
    solve_ms_list = []

    outcome = f"timeout"
    term_step = MAX_STEPS

    START_T = time.time()

    for step in range(MAX_STEPS):
        obs_env, _, _, _, _ = env.step(action)
        state = obs_env[0]
        pos   = state[0:3].copy()
        vel   = state[10:13].copy()

        positions.append(pos.copy())
        speed = float(np.linalg.norm(vel))
        if speed > max_speed_v:
            max_speed_v = speed

        d_goal = float(np.linalg.norm(pos - GOAL_POS))
        d_obs  = float(_min_surface_dist(pos, obstacles))
        if d_obs < min_clear:
            min_clear = d_obs

        # ── Termination ───────────────────────────────────────────────────────
        if d_goal < GOAL_TOL:
            outcome   = "success"
            term_step = step
            break
        if d_obs < CRASH_TOL:
            outcome   = "collision"
            term_step = step
            break

        # ── Controller step ───────────────────────────────────────────────────
        target_pos, extras = ctrl_obj.compute_target(pos)

        # Accumulate mt-MPC stats
        if controller_name == "mtmpc":
            if extras.get("feasible", True):
                qp_ok_count += 1
            else:
                fallback_cnt += 1
            if extras.get("tts", False):
                tts_count += 1
            if "solve_ms" in extras:
                solve_ms_list.append(extras["solve_ms"])

        # Track CBF QP feasibility
        if controller_name == "cbf":
            if extras.get("feasible", True):
                qp_ok_count += 1

        # ── Per-step print (every 10 steps, or every step when near obstacle) ──
        if step % 10 == 0 or d_obs < 0.15:
            feas_str = ""
            if controller_name == "mtmpc":
                feas = extras.get("feasible", True)
                tts  = extras.get("tts", False)
                feas_str = f" | {'OK    ' if feas else 'INFEAS'} | {'TTS' if tts else '   '}"
            elif controller_name == "cbf":
                feas_str = f" | {'OK' if extras.get('feasible', True) else 'INFEAS'}"
            print(
                f"step {step:4d} | "
                f"pos [{pos[0]:+.3f} {pos[1]:+.3f} {pos[2]:+.3f}] | "
                f"d_goal={d_goal:.3f} | d_obs={d_obs:.3f}"
                + feas_str
            )

        # ── DSL-PID ───────────────────────────────────────────────────────────
        action[0], _, _ = pid.computeControlFromState(
            control_timestep=env.CTRL_TIMESTEP,
            state=state,
            target_pos=target_pos,
        )

        env.render()
        if gui:
            sync(step, START_T, env.CTRL_TIMESTEP)

    if video_id is not None:
        p.stopStateLogging(video_id, physicsClientId=client)

    env.close()

    # ── Post-process metrics ──────────────────────────────────────────────────
    n_steps = term_step + 1   # steps executed including termination step

    # Path length
    path_length = 0.0
    for i in range(1, len(positions)):
        path_length += float(np.linalg.norm(positions[i] - positions[i-1]))

    total_time = n_steps * DT
    avg_speed  = path_length / total_time if total_time > 0 else 0.0

    time_to_goal = total_time if outcome == "success" else float("nan")
    safety_viol  = max(0.0, CRASH_TOL - min_clear)

    # mt-MPC rates
    if controller_name == "mtmpc" and n_steps > 0:
        qp_feas_rate = qp_ok_count / n_steps
        tts_rate_val = tts_count / n_steps
        avg_sms      = float(np.mean(solve_ms_list)) if solve_ms_list else float("nan")
        max_sms      = float(np.max(solve_ms_list))  if solve_ms_list else float("nan")
    elif controller_name == "cbf" and n_steps > 0:
        qp_feas_rate = qp_ok_count / n_steps
        tts_rate_val = float("nan")
        avg_sms = max_sms = float("nan")
        fallback_cnt = 0
    else:
        qp_feas_rate = tts_rate_val = float("nan")
        avg_sms = max_sms = float("nan")
        fallback_cnt = 0

    row = {
        "scenario":                 scenario_key,
        "controller":               controller_name,
        "vmax":                     v_max,
        "lookahead":                lookahead,
        "trial":                    trial,
        "seed":                     seed,
        "outcome":                  outcome,
        "steps":                    n_steps,
        "path_length":              round(path_length, 4),
        "min_clearance":            round(min_clear, 4),
        "avg_speed":                round(avg_speed, 4),
        "max_speed":                round(max_speed_v, 4),
        "time_to_goal_s":           round(time_to_goal, 3) if not math.isnan(time_to_goal) else "nan",
        "safety_violation_magnitude": round(safety_viol, 4),
        "qp_feasibility_rate":      round(qp_feas_rate, 4) if not math.isnan(qp_feas_rate) else "nan",
        "tts_rate":                 round(tts_rate_val, 4) if not math.isnan(tts_rate_val) else "nan",
        "fallback_count":           fallback_cnt,
        "avg_solve_ms":             round(avg_sms, 2) if not math.isnan(avg_sms) else "nan",
        "max_solve_ms":             round(max_sms, 2) if not math.isnan(max_sms) else "nan",
    }
    _append_csv(row)

    print(
        f"[{scenario_key}|{controller_name}|vmax={v_max}|la={lookahead}"
        f"|t{trial}] {outcome.upper()} in {n_steps} steps "
        f"| clear={min_clear:.3f}m | speed={avg_speed:.3f} m/s"
    )
    return row


# ─────────────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Fair comparison experiment runner")
    parser.add_argument("--scenario",   default="A",     choices=["A", "B", "C"])
    parser.add_argument("--controller", required=True,   choices=["apf", "cbf", "mtmpc"])
    parser.add_argument("--vmax",       type=float,      default=0.5)
    parser.add_argument("--lookahead",  type=float,      default=0.10)
    parser.add_argument("--trial",      type=int,        default=0)
    parser.add_argument("--seed",       type=int,        default=None)
    parser.add_argument("--no-gui",     action="store_true")
    args = parser.parse_args()

    seed = args.seed if args.seed is not None else 42 + args.trial
    run(
        scenario_key    = args.scenario,
        controller_name = args.controller,
        v_max           = args.vmax,
        lookahead       = args.lookahead,
        trial           = args.trial,
        seed            = seed,
        gui             = not args.no_gui,
    )
