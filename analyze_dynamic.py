"""Compare static-vs-dynamic Scenario B outcomes.

Reads:
  results/fair_comparison_master.csv     (76-row canonical)
  results/fair_comparison_dynamic.csv    (9-row dynamic sweep)

Writes:
  results/dynamic_comparison.txt         (formatted comparison tables)
"""
import os
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
MASTER = os.path.join(HERE, "results", "fair_comparison_master.csv")
DYN = os.path.join(HERE, "results", "fair_comparison_dynamic.csv")
OUT = os.path.join(HERE, "results", "dynamic_comparison.txt")

# ---- load + dedupe master (it sometimes accumulates duplicate runs) ----
m = pd.read_csv(MASTER)
KEY = ["scenario", "controller", "vmax", "lookahead", "trial"]
m = m.drop_duplicates(subset=KEY, keep="first")

mB = m[m["scenario"] == "B"]
mB_recommended = mB[(mB["vmax"] == 1.0) & (mB["lookahead"] == 0.40)]
d = pd.read_csv(DYN)


def stats(sub, label):
    n = len(sub)
    s = int((sub["outcome"] == "success").sum())
    t = int((sub["outcome"] == "timeout").sum())
    c = int((sub["outcome"] == "collision").sum())
    safe = (s + t) / n if n else 0.0
    succ = s / n if n else 0.0
    return {"label": label, "n": n, "s": s, "t": t, "c": c,
            "success_rate": succ, "safe_rate": safe}


def fmt_table(rows, title):
    """Render a single comparison table block."""
    lines = []
    lines.append(title)
    lines.append("-" * len(title))
    header = (f"{'Controller':<10}  "
              f"{'Static-full (24 trials)':<28}  "
              f"{'Static @ v=1.0, la=0.40 (3)':<32}  "
              f"{'Dynamic (3 trials)':<22}  "
              f"{'ΔSafe (full→dyn)':<18}")
    lines.append(header)
    for r in rows:
        full, cell, dyn = r["full"], r["cell"], r["dyn"]
        delta = dyn["safe_rate"] - full["safe_rate"]
        lines.append(
            f"{r['name']:<10}  "
            f"S={full['s']:2d} T={full['t']:2d} C={full['c']:2d}  "
            f"safe={full['safe_rate']*100:5.1f}%   "
            f"S={cell['s']} T={cell['t']} C={cell['c']}    "
            f"safe={cell['safe_rate']*100:5.1f}%      "
            f"S={dyn['s']} T={dyn['t']} C={dyn['c']}   "
            f"safe={dyn['safe_rate']*100:5.1f}%   "
            f"{delta*100:+6.1f}%"
        )
    return "\n".join(lines)


rows = []
name_map = {"apf": "APF", "cbf": "CBF-QP", "mtmpc": "mt-MPC"}
for ctrl in ["apf", "cbf", "mtmpc"]:
    rows.append({
        "name": name_map[ctrl],
        "full": stats(mB[mB["controller"] == ctrl],            "static-full"),
        "cell": stats(mB_recommended[mB_recommended["controller"] == ctrl], "static-cell"),
        "dyn":  stats(d[d["controller"] == ctrl],              "dynamic"),
    })

block_main = fmt_table(rows, "Scenario B — static vs dynamic obstacle (single moving cylinder)")

# Per-trial outcomes block
detail_lines = ["", "Per-trial dynamic outcomes:", "-" * 40]
for ctrl in ["apf", "cbf", "mtmpc"]:
    sub = d[d["controller"] == ctrl].sort_values("trial")
    for _, row in sub.iterrows():
        detail_lines.append(
            f"  {name_map[ctrl]:<7}  trial {int(row['trial'])}  "
            f"{row['outcome'].upper():<9}  "
            f"steps={int(row['steps']):4d}  "
            f"clear={row['min_clearance']:.3f} m  "
            f"speed={row['avg_speed']:.3f} m/s"
        )
    detail_lines.append("")

# Summary headline interpretation
summary = []
summary.append("Headline:")
summary.append(f"  APF      dynamic safe-rate {rows[0]['dyn']['safe_rate']*100:.1f}% "
               f"(static full {rows[0]['full']['safe_rate']*100:.1f}% — "
               f"static recommended cell {rows[0]['cell']['safe_rate']*100:.1f}%)")
summary.append(f"  CBF-QP   dynamic safe-rate {rows[1]['dyn']['safe_rate']*100:.1f}% "
               f"(static full {rows[1]['full']['safe_rate']*100:.1f}% — "
               f"static recommended cell {rows[1]['cell']['safe_rate']*100:.1f}%)")
summary.append(f"  mt-MPC   dynamic safe-rate {rows[2]['dyn']['safe_rate']*100:.1f}% "
               f"(static full {rows[2]['full']['safe_rate']*100:.1f}% — "
               f"static recommended cell {rows[2]['cell']['safe_rate']*100:.1f}%)")

text = "\n\n".join([block_main, "\n".join(detail_lines), "\n".join(summary)])
print(text)
with open(OUT, "w") as f:
    f.write(text + "\n")
print(f"\nWritten: {OUT}")
