"""Generate Scenario A trajectory and speed-profile plots for the presentation.

Outputs:
  results/trajectory_scenario_A.png   — top-down XY overlay of all 3 controllers
  results/speed_over_time.png         — speed vs time for all 3 controllers
"""
import os

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

RESULTS = os.path.join(os.path.dirname(__file__), "results")

LOGS = {
    "APF":    ("apf_3d_log.csv",    "#d62728"),  # red
    "CBF-QP": ("cbf_qp_3d_log.csv", "#1f77b4"),  # blue
    "mt-MPC": ("mtmpc_3d_log.csv",  "#2ca02c"),  # green
}

OBSTACLES = [  # (cx, cy, r)  — Scenario A, from scenarios.py
    (1.0, 1.0, 0.3),
    (2.0, 1.5, 0.3),
    (1.5, 2.5, 0.3),
]
START = (0.0, 0.0)
GOAL = (3.0, 3.0)


def load(name):
    return pd.read_csv(os.path.join(RESULTS, LOGS[name][0]))


def compute_speed(df):
    """Instantaneous speed (m/s) from successive 3-D positions."""
    dx = np.diff(df["x"].values)
    dy = np.diff(df["y"].values)
    dz = np.diff(df["z"].values)
    dt = np.diff(df["t"].values)
    # avoid div-by-zero for duplicate timestamps
    dt = np.where(dt > 1e-9, dt, np.nan)
    speed = np.sqrt(dx * dx + dy * dy + dz * dz) / dt
    # align to time midpoint for plotting
    t_mid = (df["t"].values[1:] + df["t"].values[:-1]) / 2.0
    return t_mid, speed


def path_length(df):
    dx = np.diff(df["x"].values)
    dy = np.diff(df["y"].values)
    dz = np.diff(df["z"].values)
    return float(np.sum(np.sqrt(dx * dx + dy * dy + dz * dz)))


# ---------------- Figure 1: trajectory overlay ----------------
fig, ax = plt.subplots(figsize=(8, 8), dpi=300)

# obstacles
for cx, cy, r in OBSTACLES:
    ax.add_patch(plt.Circle((cx, cy), r, color="#888888", alpha=0.6, zorder=1))

# trajectories
summary = {}
for name, (csv, color) in LOGS.items():
    df = load(name)
    lw = 3.0 if name == "mt-MPC" else 1.8
    ax.plot(df["x"], df["y"], color=color, linewidth=lw, label=name,
            zorder=3 if name == "mt-MPC" else 2)
    summary[name] = dict(
        total_time=float(df["t"].iloc[-1]),
        path_len=path_length(df),
        mean_speed=path_length(df) / float(df["t"].iloc[-1]),
    )

# start / goal
ax.plot(*START, marker="o", markersize=14, color="#2ca02c",
        markeredgecolor="black", zorder=5)
ax.annotate("START", START, textcoords="offset points", xytext=(10, -15),
            fontsize=11, fontweight="bold")
ax.plot(*GOAL, marker="*", markersize=22, color="#d62728",
        markeredgecolor="black", zorder=5)
ax.annotate("GOAL", GOAL, textcoords="offset points", xytext=(10, 5),
            fontsize=11, fontweight="bold")

ax.set_xlabel("X (m)", fontsize=12)
ax.set_ylabel("Y (m)", fontsize=12)
ax.set_title("Scenario A: Trajectory Comparison", fontsize=14, fontweight="bold")
ax.set_aspect("equal")
ax.grid(True, alpha=0.3)
ax.legend(loc="lower right", fontsize=11, framealpha=0.95)
ax.set_xlim(-0.5, 3.5)
ax.set_ylim(-0.5, 3.5)

out1 = os.path.join(RESULTS, "trajectory_scenario_A.png")
fig.tight_layout()
fig.savefig(out1, dpi=300, bbox_inches="tight")
plt.close(fig)

# ---------------- Figure 2: speed profile ----------------
fig, ax = plt.subplots(figsize=(10, 5), dpi=300)
for name, (_csv, color) in LOGS.items():
    df = load(name)
    t_mid, speed = compute_speed(df)
    ax.plot(t_mid, speed, color=color, linewidth=1.5, label=name, alpha=0.85)

ax.set_xlabel("Time (s)", fontsize=12)
ax.set_ylabel("Instantaneous speed (m/s)", fontsize=12)
ax.set_title("Speed Profile Across Controllers (Scenario A)",
             fontsize=14, fontweight="bold")
ax.grid(True, alpha=0.3)
ax.legend(loc="upper right", fontsize=11)

out2 = os.path.join(RESULTS, "speed_over_time.png")
fig.tight_layout()
fig.savefig(out2, dpi=300, bbox_inches="tight")
plt.close(fig)

# ---------------- Summary stats ----------------
print("\n==== Scenario A summary ====")
print(f"{'Controller':<10} {'Total time (s)':>15} {'Path length (m)':>18} "
      f"{'Mean speed (m/s)':>18}")
for name, s in summary.items():
    print(f"{name:<10} {s['total_time']:>15.2f} {s['path_len']:>18.3f} "
          f"{s['mean_speed']:>18.3f}")

print(f"\nSaved: {out1}")
print(f"Saved: {out2}")
