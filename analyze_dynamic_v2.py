"""Compare static vs dynamic-v2 (open-space layout) Scenario B outcomes.

Reads:
  results/fair_comparison_master.csv          — static canonical (76 rows)
  results/fair_comparison_dynamic.csv         — dynamic-v1 (1 obstacle, near cluster)
  results/fair_comparison_dynamic_v2.csv      — dynamic-v2 (2 obstacles, open space)

Writes:
  results/dynamic_comparison_v2.txt           — formatted comparison
"""
import os
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
MASTER = os.path.join(HERE, "results", "fair_comparison_master.csv")
DYN_V1 = os.path.join(HERE, "results", "fair_comparison_dynamic.csv")
DYN_V2 = os.path.join(HERE, "results", "fair_comparison_dynamic_v2.csv")
OUT    = os.path.join(HERE, "results", "dynamic_comparison_v2.txt")

# ------- load static (deduped) -------
m = pd.read_csv(MASTER)
KEY = ["scenario", "controller", "vmax", "lookahead", "trial"]
m = m.drop_duplicates(subset=KEY, keep="first")
mB_recommended = m[(m["scenario"] == "B")
                   & (m["vmax"] == 1.0)
                   & (m["lookahead"] == 0.40)]
mB_full = m[m["scenario"] == "B"]

d1 = pd.read_csv(DYN_V1)
d2 = pd.read_csv(DYN_V2)


def stats(sub):
    n = len(sub)
    s = int((sub["outcome"] == "success").sum())
    t = int((sub["outcome"] == "timeout").sum())
    c = int((sub["outcome"] == "collision").sum())
    return {"n": n, "s": s, "t": t, "c": c,
            "succ": (s / n) if n else 0.0,
            "safe": ((s + t) / n) if n else 0.0}


name_map = {"apf": "APF", "cbf": "CBF-QP", "mtmpc": "mt-MPC"}

# ------- per-trial detail block -------
detail = ["Per-trial outcomes — Dynamic v2 (2 moving obstacles in open space)",
          "-" * 70]
for ctrl in ["apf", "cbf", "mtmpc"]:
    sub = d2[d2["controller"] == ctrl].sort_values("trial")
    for _, row in sub.iterrows():
        detail.append(
            f"  {name_map[ctrl]:<7}  trial {int(row['trial'])}  "
            f"{row['outcome'].upper():<9}  "
            f"steps={int(row['steps']):4d}  "
            f"clear={row['min_clearance']:.3f} m  "
            f"speed={row['avg_speed']:.3f} m/s  "
            f"path={row['path_length']:.2f} m"
        )
    detail.append("")

# ------- comparison table block -------
hdr = (f"{'Controller':<10}  "
       f"{'Static-full (24)':<24}  "
       f"{'Static @ best (3)':<22}  "
       f"{'Dyn v1 (3)':<18}  "
       f"{'Dyn v2 (3)':<18}  "
       f"{'Δsafe (full→v2)':<14}")
table = ["", "Comparison table", "-" * len(hdr), hdr]

for ctrl in ["apf", "cbf", "mtmpc"]:
    full = stats(mB_full[mB_full["controller"] == ctrl])
    rec  = stats(mB_recommended[mB_recommended["controller"] == ctrl])
    v1   = stats(d1[d1["controller"] == ctrl])
    v2   = stats(d2[d2["controller"] == ctrl])
    delta = v2["safe"] - full["safe"]
    table.append(
        f"{name_map[ctrl]:<10}  "
        f"S={full['s']:2d} T={full['t']:2d} C={full['c']:2d} {full['safe']*100:5.1f}%   "
        f"S={rec['s']} T={rec['t']} C={rec['c']} {rec['safe']*100:5.1f}%    "
        f"S={v1['s']} T={v1['t']} C={v1['c']} {v1['safe']*100:5.1f}%   "
        f"S={v2['s']} T={v2['t']} C={v2['c']} {v2['safe']*100:5.1f}%   "
        f"{delta*100:+6.1f}%"
    )

# ------- headline interpretation -------
summary = ["", "Headline (success rate, success∪timeout = safe rate):"]
for ctrl in ["apf", "cbf", "mtmpc"]:
    rec = stats(mB_recommended[mB_recommended["controller"] == ctrl])
    v2  = stats(d2[d2["controller"] == ctrl])
    summary.append(
        f"  {name_map[ctrl]:<7}  "
        f"static@best  succ {rec['succ']*100:5.1f}%  safe {rec['safe']*100:5.1f}%  "
        f"|  dyn v2  succ {v2['succ']*100:5.1f}%  safe {v2['safe']*100:5.1f}%  "
        f"|  Δsucc {(v2['succ']-rec['succ'])*100:+6.1f}%   "
        f"Δsafe {(v2['safe']-rec['safe'])*100:+6.1f}%"
    )

text = "\n".join(detail + table + summary)
print(text)
with open(OUT, "w") as f:
    f.write(text + "\n")
print(f"\nWritten: {OUT}")
