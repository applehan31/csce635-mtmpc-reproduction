"""Publication-quality figures for the CSCE 635 final report.

Generates three 300-DPI PNGs in results/report_figures/:
  fig_scenario_b_outcomes.png  — stacked horizontal bar of Scenario B outcomes
  fig_lookahead_heatmap.png    — mt-MPC success-rate heatmap across (vmax, la)
  fig_algo2_ablation.png       — pre/post Algorithm-2 outcome comparison

The CSV is verified against the canonical hardcoded numbers; on
disagreement the CSV wins and the discrepancy is printed.
"""
import os

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns

HERE = os.path.dirname(os.path.abspath(__file__))
CSV = os.path.join(HERE, "results", "fair_comparison_master.csv")
OUT = os.path.join(HERE, "results", "report_figures")
os.makedirs(OUT, exist_ok=True)

GREEN = "#2ca02c"
GRAY = "#7f7f7f"
RED = "#d62728"

plt.rcParams.update({
    "font.family": "DejaVu Sans",
    "font.size": 11,
    "axes.facecolor": "white",
    "savefig.facecolor": "white",
    "figure.facecolor": "white",
})


# ---------------------------------------------------------------- verify CSV
df = pd.read_csv(CSV)
# Dedup by experiment key — protects against re-runs of the same config
# accidentally appending duplicate rows (recording runs are notorious for this).
KEY = ["scenario", "controller", "vmax", "lookahead", "trial"]
n_pre = len(df)
df = df.drop_duplicates(subset=KEY, keep="first").reset_index(drop=True)
n_post = len(df)
if n_pre != n_post:
    print(f"[dedupe] dropped {n_pre - n_post} duplicate row(s) "
          f"({n_pre} -> {n_post})")
b = df[df["scenario"] == "B"]

print("=" * 60)
print("CSV VERIFICATION  (Scenario B)")
print("=" * 60)

EXPECTED = {
    "apf":   {"success": 10, "timeout": 8,  "collision": 6},
    "cbf":   {"success": 0,  "timeout": 10, "collision": 14},
    "mtmpc": {"success": 11, "timeout": 8,  "collision": 5},
}

actual = {}
discrepancies = []
for ctrl in ["apf", "cbf", "mtmpc"]:
    sub = b[b["controller"] == ctrl]
    counts = sub["outcome"].value_counts().to_dict()
    counts = {k: int(counts.get(k, 0)) for k in ["success", "timeout", "collision"]}
    actual[ctrl] = counts
    exp = EXPECTED[ctrl]
    match = counts == exp
    flag = "OK" if match else "MISMATCH"
    print(f"  {ctrl:5s}  expected={exp}  actual={counts}  [{flag}]")
    if not match:
        discrepancies.append((ctrl, exp, counts))

# verify mt-MPC heatmap
EXPECTED_HEATMAP = {
    (0.5, 0.10): 1/3, (0.5, 0.25): 3/3, (0.5, 0.40): 2/3, (0.5, 0.60): 0/3,
    (1.0, 0.10): 0/3, (1.0, 0.25): 2/3, (1.0, 0.40): 3/3, (1.0, 0.60): 0/3,
}
mt = b[b["controller"] == "mtmpc"]
print("\nHeatmap verification (mt-MPC success rate):")
for (vmax, la), exp_rate in EXPECTED_HEATMAP.items():
    cell = mt[(mt["vmax"] == vmax) & (mt["lookahead"] == la)]
    n = len(cell)
    n_succ = int((cell["outcome"] == "success").sum())
    actual_rate = n_succ / n if n > 0 else 0.0
    flag = "OK" if abs(actual_rate - exp_rate) < 1e-6 else "MISMATCH"
    print(f"  v={vmax}, la={la:.2f}: expected {exp_rate:.2f}  actual {actual_rate:.2f}"
          f" ({n_succ}/{n}) [{flag}]")
    if abs(actual_rate - exp_rate) > 1e-6:
        discrepancies.append((f"mtmpc v={vmax} la={la}", exp_rate, actual_rate))


# --------------------------------------------------- Figure 1: outcome bar
def figure_1():
    fig, ax = plt.subplots(figsize=(6, 3))
    labels = ["APF", "CBF-QP", "mt-MPC"]
    keys = ["apf", "cbf", "mtmpc"]
    s = np.array([actual[k]["success"]   for k in keys])
    t = np.array([actual[k]["timeout"]   for k in keys])
    c = np.array([actual[k]["collision"] for k in keys])
    y = np.arange(len(labels))

    ax.barh(y, s,           color=GREEN, label="Success",   edgecolor="white")
    ax.barh(y, t, left=s,        color=GRAY,  label="Timeout",   edgecolor="white")
    ax.barh(y, c, left=s + t,    color=RED,   label="Collision", edgecolor="white")

    # in-bar count labels
    for i, val in enumerate(s):
        if val > 0:
            ax.text(val / 2, i, str(val), color="white",
                    va="center", ha="center", fontweight="bold", fontsize=11)
    for i, val in enumerate(t):
        if val > 0:
            ax.text(s[i] + val / 2, i, str(val), color="white",
                    va="center", ha="center", fontweight="bold", fontsize=11)
    for i, val in enumerate(c):
        if val > 0:
            ax.text(s[i] + t[i] + val / 2, i, str(val), color="white",
                    va="center", ha="center", fontweight="bold", fontsize=11)

    ax.set_yticks(y)
    ax.set_yticklabels(labels)
    ax.set_xlabel("Number of trials")
    ax.set_xlim(0, 24)
    ax.set_title("Scenario B Outcomes (24 Trials per Controller)")
    ax.legend(loc="lower center", bbox_to_anchor=(0.5, -0.45),
              ncol=3, frameon=False)
    ax.invert_yaxis()
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    out = os.path.join(OUT, "fig_scenario_b_outcomes.png")
    fig.savefig(out, dpi=300, bbox_inches="tight")
    plt.close(fig)
    return out


# --------------------------------------------------- Figure 2: heatmap
def figure_2():
    vmax_vals = [0.5, 1.0]
    la_vals = [0.10, 0.25, 0.40, 0.60]
    grid = np.zeros((len(vmax_vals), len(la_vals)))
    for i, vmax in enumerate(vmax_vals):
        for j, la in enumerate(la_vals):
            cell = mt[(mt["vmax"] == vmax) & (mt["lookahead"] == la)]
            n = len(cell)
            grid[i, j] = ((cell["outcome"] == "success").sum() / n) if n else 0.0

    fig, ax = plt.subplots(figsize=(5, 2.5))
    sns.heatmap(
        grid, ax=ax,
        cmap="RdYlGn", vmin=0, vmax=1,
        annot=True, fmt=".2f",
        annot_kws={"fontsize": 10, "fontweight": "bold"},
        xticklabels=[f"{v:.2f}" for v in la_vals],
        yticklabels=[f"{v:.1f}" for v in vmax_vals],
        cbar_kws={"label": "Success Rate"},
        linewidths=0.5, linecolor="white",
    )
    ax.set_xlabel("LOOKAHEAD (m)")
    ax.set_ylabel("v_max (m/s)")
    ax.set_title("mt-MPC Success Rate Across Configurations")
    plt.setp(ax.get_yticklabels(), rotation=0)

    out = os.path.join(OUT, "fig_lookahead_heatmap.png")
    fig.savefig(out, dpi=300, bbox_inches="tight")
    plt.close(fig)
    return out


# --------------------------------------------------- Figure 3: Algo-2 ablation
def figure_3():
    before = {"success": 4,  "timeout": 0, "collision": 20}
    after  = {"success": 11, "timeout": 8, "collision": 5}

    fig, ax = plt.subplots(figsize=(5, 3.5))
    labels = ["Before Algorithm 2\n(heuristic fallback)",
              "After Algorithm 2\n(Stages 1-4)"]
    x = np.arange(len(labels))

    s = np.array([before["success"],   after["success"]])
    t = np.array([before["timeout"],   after["timeout"]])
    c = np.array([before["collision"], after["collision"]])

    bw = 0.55
    ax.bar(x, s,             width=bw, color=GREEN, label="Success",   edgecolor="white")
    ax.bar(x, t, bottom=s,        width=bw, color=GRAY,  label="Timeout",   edgecolor="white")
    ax.bar(x, c, bottom=s + t,    width=bw, color=RED,   label="Collision", edgecolor="white")

    for i in range(len(x)):
        if s[i] > 0:
            ax.text(x[i], s[i] / 2, str(s[i]), color="white",
                    ha="center", va="center", fontweight="bold")
        if t[i] > 0:
            ax.text(x[i], s[i] + t[i] / 2, str(t[i]), color="white",
                    ha="center", va="center", fontweight="bold")
        if c[i] > 0:
            ax.text(x[i], s[i] + t[i] + c[i] / 2, str(c[i]), color="white",
                    ha="center", va="center", fontweight="bold")

    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_ylabel("Number of Trials (out of 24)")
    ax.set_ylim(0, 26)
    ax.set_title("Effect of Algorithm 2 Implementation on Scenario B")
    ax.legend(loc="center left", bbox_to_anchor=(1.02, 0.5), frameon=False)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    fig.subplots_adjust(bottom=0.30)
    fig.text(0.5, 0.02,
             "Collisions reduced from 20/24 to 5/24 (75% reduction)",
             ha="center", fontsize=10, style="italic")

    out = os.path.join(OUT, "fig_algo2_ablation.png")
    fig.savefig(out, dpi=300, bbox_inches="tight")
    plt.close(fig)
    return out


paths = [figure_1(), figure_2(), figure_3()]

print("\n" + "=" * 60)
print("OUTPUT")
print("=" * 60)
for pth in paths:
    sz = os.path.getsize(pth) / 1024
    print(f"  {pth}  ({sz:.1f} KB)")

if discrepancies:
    print("\nDISCREPANCIES (CSV used; hardcoded numbers ignored):")
    for d in discrepancies:
        print(f"  {d}")
else:
    print("\nAll CSV values match the hardcoded canonical numbers exactly.")
