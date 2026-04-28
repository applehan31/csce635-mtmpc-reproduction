"""Final figure: static vs dynamic safe rate, grouped by controller."""
import os

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
DYN = os.path.join(HERE, "results", "fair_comparison_dynamic.csv")
OUT = os.path.join(HERE, "results", "report_figures",
                   "fig_dynamic_comparison.png")
os.makedirs(os.path.dirname(OUT), exist_ok=True)

plt.rcParams.update({
    "font.family": "DejaVu Sans",
    "font.size": 11,
    "axes.facecolor": "white",
    "savefig.facecolor": "white",
    "figure.facecolor": "white",
})

# Static @ recommended config (vmax=1.0, lookahead=0.40), 3 trials each.
# Numbers per task spec — derived from fair_comparison_master.csv.
static = {"apf": 1/3, "cbf": 0/3, "mtmpc": 3/3}

# Dynamic — read directly from CSV.
d = pd.read_csv(DYN)
dynamic = {}
for ctrl in ["apf", "cbf", "mtmpc"]:
    sub = d[d["controller"] == ctrl]
    n = len(sub)
    s = int((sub["outcome"] == "success").sum())
    dynamic[ctrl] = (s / n) if n else 0.0

print("Verification:")
for k in ["apf", "cbf", "mtmpc"]:
    print(f"  {k:6s}  static={static[k]*100:5.1f}%   "
          f"dynamic={dynamic[k]*100:5.1f}%")

labels = ["APF", "CBF-QP", "mt-MPC"]
keys = ["apf", "cbf", "mtmpc"]
static_v  = np.array([static[k]  * 100 for k in keys])
dynamic_v = np.array([dynamic[k] * 100 for k in keys])

x = np.arange(len(labels))
bw = 0.35

fig, ax = plt.subplots(figsize=(5, 3))
bars1 = ax.bar(x - bw/2, static_v,  width=bw, color="#aec7e8",
               label="Static @ recommended config", edgecolor="#5b8db5")
bars2 = ax.bar(x + bw/2, dynamic_v, width=bw, color="#ff7f0e",
               label="Dynamic obstacle", edgecolor="#b85a06")

for b, v in zip(bars1, static_v):
    ax.text(b.get_x() + b.get_width()/2, v + 2, f"{v:.0f}%",
            ha="center", va="bottom", fontsize=9, fontweight="bold")
for b, v in zip(bars2, dynamic_v):
    ax.text(b.get_x() + b.get_width()/2, v + 2, f"{v:.0f}%",
            ha="center", va="bottom", fontsize=9, fontweight="bold")

ax.set_xticks(x)
ax.set_xticklabels(labels)
ax.set_ylabel("Safe Rate (%)")
ax.set_ylim(0, 115)
ax.set_yticks([0, 25, 50, 75, 100])
ax.set_title("Static vs Dynamic Obstacle Performance", pad=28)
ax.legend(loc="upper center", bbox_to_anchor=(0.5, 1.22),
          ncol=2, frameon=False, fontsize=9)
ax.spines["top"].set_visible(False)
ax.spines["right"].set_visible(False)
ax.grid(axis="y", alpha=0.25)
ax.set_axisbelow(True)

fig.tight_layout()
fig.savefig(OUT, dpi=300, bbox_inches="tight")
plt.close(fig)

sz = os.path.getsize(OUT) / 1024
print(f"\nSaved: {OUT}  ({sz:.1f} KB)")
