"""fair_comparison_dynamic.py — dynamic-obstacle variant of fair_comparison.

Scenario fixed to SCENARIO_B_DYNAMIC: the six static obstacles of Scenario B
plus one horizontally-oscillating cylinder spawned at run time. Each control
step the cylinder is moved in PyBullet AND the controller's obstacle list is
updated in place so APF/CBF/mt-MPC all see its current position.

Usage:
    python fair_comparison_dynamic.py --controller mtmpc --vmax 1.0 \\
        --lookahead 0.40 --trial 0 --no-gui

Output appended to experiments/results/fair_comparison_dynamic.csv (same
column set as fair_comparison_master.csv, with `scenario` always 'B_DYN').
"""
import os
import csv
import time
import math
import argparse
import copy

import numpy as np
import pybullet as p

from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.utils import sync

import scenarios_dynamic as SCD
from fair_comparison import (
    APFController, CBFController, MtMPCController,
    _min_surface_dist, _load_obstacles, _spawn_marker,
    CTRL_FREQ, SIM_FREQ, DT, GOAL_TOL, CRASH_TOL, MAX_STEPS,
    _CSV_HEADER, _SAFETY_MARGIN_MPC,
)

# ── Constants ────────────────────────────────────────────────────────────────
START_POS  = np.array(SCD.START_POS, dtype=float)
GOAL_POS   = np.array(SCD.GOAL_POS,  dtype=float)
OBS_HEIGHT = SCD.OBS_HEIGHT

_HERE       = os.path.dirname(os.path.abspath(__file__))
RESULTS_DIR = os.path.join(_HERE, "results")
DYN_CSV     = os.path.join(RESULTS_DIR, "fair_comparison_dynamic_v2.csv")


def _append_dynamic_csv(row: dict):
    os.makedirs(RESULTS_DIR, exist_ok=True)
    write_header = not os.path.exists(DYN_CSV)
    with open(DYN_CSV, "a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=_CSV_HEADER)
        if write_header:
            writer.writeheader()
        writer.writerow(row)


def _refresh_safe_radii(ctrl_obj, obstacles):
    """Re-derive obs_safe_r if the controller caches it (CBF, mt-MPC)."""
    if hasattr(ctrl_obj, "obs_safe_r"):
        if isinstance(ctrl_obj, CBFController):
            margin = ctrl_obj.SAFETY_MARGIN
        elif isinstance(ctrl_obj, MtMPCController):
            margin = _SAFETY_MARGIN_MPC
        else:
            margin = 0.0
        ctrl_obj.obs_safe_r = [r + margin for _, _, _, r in obstacles]


def run(controller_name: str, v_max: float, lookahead: float,
        trial: int, seed: int, gui: bool):
    """Run one trial on Scenario B + N moving obstacles (v2 = 2 dyn)."""
    static_obstacles = list(SCD.SCENARIO_B_DYNAMIC)
    # Per-obstacle constants (radius, cz_base) for the dynamic cylinders.
    dyn_specs = SCD.DYNAMIC_OBSTACLES
    n_dyn = len(dyn_specs)

    # Initial dynamic positions (t = 0).
    init_dyn_xy = SCD.get_dynamic_obstacle_position(0.0)
    initial_combined = static_obstacles + [
        [init_dyn_xy[i][0], init_dyn_xy[i][1],
         dyn_specs[i]["cz_base"], dyn_specs[i]["radius"]]
        for i in range(n_dyn)
    ]

    rng = np.random.RandomState(seed)
    noise = rng.randn(3) * 0.01
    noise[2] = 0.0
    init_pos = START_POS + noise

    # ── Controller (fed the *current* obstacles each step via mutation) ─────
    if controller_name == "apf":
        ctrl_obj = APFController(initial_combined, GOAL_POS, lookahead)
        color    = [1.0, 0.3, 0.1, 0.7]
    elif controller_name == "cbf":
        ctrl_obj = CBFController(initial_combined, GOAL_POS, v_max, lookahead)
        color    = [0.2, 0.6, 1.0, 0.7]
    elif controller_name == "mtmpc":
        ctrl_obj = MtMPCController(initial_combined, GOAL_POS, v_max, lookahead)
        color    = [0.6, 0.2, 0.8, 0.7]
    else:
        raise ValueError(f"Unknown controller: {controller_name!r}")

    # ── Env ─────────────────────────────────────────────────────────────────
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

    # ── Video (GUI only) ─────────────────────────────────────────────────────
    video_id   = None
    VIDEO_PATH = None
    if gui:
        _video_dir = os.path.join(RESULTS_DIR, "videos")
        os.makedirs(_video_dir, exist_ok=True)
        VIDEO_PATH = os.path.join(
            _video_dir,
            f"dyn_{controller_name}_vmax{v_max}_la{lookahead}_t{trial}.mp4",
        )
        video_id = p.startStateLogging(
            p.STATE_LOGGING_VIDEO_MP4, VIDEO_PATH, physicsClientId=client
        )

    # Static obstacles (orange-red regardless of controller, to distinguish)
    _load_obstacles(client, static_obstacles, color)

    # Dynamic obstacles — separately spawned and tracked. Two distinct
    # bright colours so the user can tell them apart in the GUI.
    dyn_colors = [[1.0, 0.85, 0.0, 0.85],   # yellow (left, vertical sweep)
                  [1.0, 0.10, 0.95, 0.85]]  # magenta (right, horizontal sweep)
    dyn_body_ids = []
    for i, spec in enumerate(dyn_specs):
        col = p.createCollisionShape(
            p.GEOM_CYLINDER, radius=spec["radius"], height=OBS_HEIGHT,
            physicsClientId=client,
        )
        vis = p.createVisualShape(
            p.GEOM_CYLINDER, radius=spec["radius"], length=OBS_HEIGHT,
            rgbaColor=dyn_colors[i % len(dyn_colors)],
            physicsClientId=client,
        )
        body = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=col,
            baseVisualShapeIndex=vis,
            basePosition=[init_dyn_xy[i][0], init_dyn_xy[i][1],
                          OBS_HEIGHT / 2.0],
            physicsClientId=client,
        )
        dyn_body_ids.append(body)

    # Markers
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
        cameraDistance=5.5, cameraYaw=240.6, cameraPitch=-78.6,
        cameraTargetPosition=[1.5, 1.5, 0.5], physicsClientId=client,
    )

    pid = DSLPIDControl(drone_model=DroneModel.CF2X)
    action = np.zeros((1, 4))

    positions   = []
    min_clear   = math.inf
    max_speed_v = 0.0
    qp_ok_count = 0
    tts_count   = 0
    fallback_cnt = 0
    solve_ms_list = []

    outcome   = "timeout"
    term_step = MAX_STEPS

    START_T = time.time()

    for step in range(MAX_STEPS):
        # ── Update moving obstacle positions BEFORE controller step ─────────
        cur_t = step * DT
        dyn_xy = SCD.get_dynamic_obstacle_position(cur_t)
        for i, (dx_, dy_) in enumerate(dyn_xy):
            p.resetBasePositionAndOrientation(
                dyn_body_ids[i],
                [dx_, dy_, OBS_HEIGHT / 2.0],
                [0, 0, 0, 1],
                physicsClientId=client,
            )

        # Build current combined obstacles list and inject into controller.
        current_obstacles = static_obstacles + [
            [dyn_xy[i][0], dyn_xy[i][1],
             dyn_specs[i]["cz_base"], dyn_specs[i]["radius"]]
            for i in range(n_dyn)
        ]
        ctrl_obj.obstacles = current_obstacles
        _refresh_safe_radii(ctrl_obj, current_obstacles)

        # Verification prints at canonical steps.
        if step in (0, 100, 200):
            pos_str = "  ".join(
                f"obs{i}=({dyn_xy[i][0]:+.3f}, {dyn_xy[i][1]:+.3f})"
                for i in range(n_dyn)
            )
            print(f"[verify step={step}]  t={cur_t:.3f}s  {pos_str}")

        obs_env, _, _, _, _ = env.step(action)
        state = obs_env[0]
        pos   = state[0:3].copy()
        vel   = state[10:13].copy()

        positions.append(pos.copy())
        speed = float(np.linalg.norm(vel))
        if speed > max_speed_v:
            max_speed_v = speed

        d_goal = float(np.linalg.norm(pos - GOAL_POS))
        d_obs  = float(_min_surface_dist(pos, current_obstacles))
        if d_obs < min_clear:
            min_clear = d_obs

        if d_goal < GOAL_TOL:
            outcome = "success"; term_step = step; break
        if d_obs < CRASH_TOL:
            outcome = "collision"; term_step = step; break

        target_pos, extras = ctrl_obj.compute_target(pos)

        if controller_name == "mtmpc":
            if extras.get("feasible", True):
                qp_ok_count += 1
            else:
                fallback_cnt += 1
            if extras.get("tts", False):
                tts_count += 1
            if "solve_ms" in extras:
                solve_ms_list.append(extras["solve_ms"])
        elif controller_name == "cbf":
            if extras.get("feasible", True):
                qp_ok_count += 1

        if step % 25 == 0 or d_obs < 0.15:
            dyn_str = " ".join(
                f"d{i}=({dyn_xy[i][0]:+.2f},{dyn_xy[i][1]:+.2f})"
                for i in range(n_dyn)
            )
            print(
                f"step {step:4d} | pos [{pos[0]:+.3f} {pos[1]:+.3f} {pos[2]:+.3f}] | "
                f"d_goal={d_goal:.3f} | d_obs={d_obs:.3f} | {dyn_str}"
            )

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

    n_steps = term_step + 1
    path_length = sum(
        float(np.linalg.norm(positions[i] - positions[i-1]))
        for i in range(1, len(positions))
    )
    total_time = n_steps * DT
    avg_speed  = path_length / total_time if total_time > 0 else 0.0
    time_to_goal = total_time if outcome == "success" else float("nan")
    safety_viol  = max(0.0, CRASH_TOL - min_clear)

    if controller_name == "mtmpc" and n_steps > 0:
        qp_feas_rate = qp_ok_count / n_steps
        tts_rate_val = tts_count / n_steps
        avg_sms = float(np.mean(solve_ms_list)) if solve_ms_list else float("nan")
        max_sms = float(np.max(solve_ms_list))  if solve_ms_list else float("nan")
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
        "scenario": "B_DYN",
        "controller": controller_name,
        "vmax": v_max, "lookahead": lookahead, "trial": trial, "seed": seed,
        "outcome": outcome, "steps": n_steps,
        "path_length":   round(path_length, 4),
        "min_clearance": round(min_clear, 4),
        "avg_speed":     round(avg_speed, 4),
        "max_speed":     round(max_speed_v, 4),
        "time_to_goal_s": round(time_to_goal, 3) if not math.isnan(time_to_goal) else "nan",
        "safety_violation_magnitude": round(safety_viol, 4),
        "qp_feasibility_rate": round(qp_feas_rate, 4) if not math.isnan(qp_feas_rate) else "nan",
        "tts_rate":            round(tts_rate_val, 4) if not math.isnan(tts_rate_val) else "nan",
        "fallback_count":      fallback_cnt,
        "avg_solve_ms":        round(avg_sms, 2) if not math.isnan(avg_sms) else "nan",
        "max_solve_ms":        round(max_sms, 2) if not math.isnan(max_sms) else "nan",
    }
    _append_dynamic_csv(row)

    print(
        f"[B_DYN|{controller_name}|vmax={v_max}|la={lookahead}|t{trial}] "
        f"{outcome.upper()} in {n_steps} steps "
        f"| clear={min_clear:.3f}m | speed={avg_speed:.3f} m/s"
    )
    return row


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Dynamic-obstacle Scenario B runner")
    parser.add_argument("--controller", required=True, choices=["apf", "cbf", "mtmpc"])
    parser.add_argument("--vmax",       type=float, default=1.0)
    parser.add_argument("--lookahead",  type=float, default=0.40)
    parser.add_argument("--trial",      type=int,   default=0)
    parser.add_argument("--seed",       type=int,   default=None)
    parser.add_argument("--no-gui",     action="store_true")
    args = parser.parse_args()
    seed = args.seed if args.seed is not None else 42 + args.trial
    run(controller_name=args.controller, v_max=args.vmax,
        lookahead=args.lookahead, trial=args.trial, seed=seed,
        gui=not args.no_gui)
