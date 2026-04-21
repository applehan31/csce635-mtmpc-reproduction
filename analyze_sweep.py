"""analyze_sweep.py — Read fair_comparison_master.csv and produce markdown summary.

Output: experiments/results/fair_comparison_summary.md

Usage:
    python analyze_sweep.py
"""

import os
import csv
import math
import itertools
from collections import defaultdict

_HERE       = os.path.dirname(os.path.abspath(__file__))
RESULTS_DIR = os.path.join(_HERE, "results")
MASTER_CSV  = os.path.join(RESULTS_DIR, "fair_comparison_master.csv")
OUT_MD      = os.path.join(RESULTS_DIR, "fair_comparison_summary.md")

CONTROLLERS = ["apf", "cbf", "mtmpc"]


# ─────────────────────────────────────────────────────────────────────────────
# Load data
# ─────────────────────────────────────────────────────────────────────────────

def _f(x):
    """Parse float; return nan if blank / 'nan'."""
    try:
        v = float(x)
        return v
    except (ValueError, TypeError):
        return float("nan")


def load_rows():
    rows = []
    with open(MASTER_CSV, newline="") as f:
        for r in csv.DictReader(f):
            rows.append(r)
    return rows


def key(row):
    return (row["scenario"], row["controller"],
            float(row["vmax"]), float(row["lookahead"]))


# ─────────────────────────────────────────────────────────────────────────────
# Stats helpers
# ─────────────────────────────────────────────────────────────────────────────

def mean_std(vals):
    vals = [v for v in vals if not math.isnan(v)]
    if not vals:
        return float("nan"), float("nan")
    m = sum(vals) / len(vals)
    if len(vals) == 1:
        return m, 0.0
    v = sum((x - m)**2 for x in vals) / (len(vals) - 1)
    return m, math.sqrt(v)


def safe_mean(vals):
    vals = [v for v in vals if not math.isnan(v)]
    return sum(vals) / len(vals) if vals else float("nan")


def fmt(val, precision=2):
    if isinstance(val, float) and math.isnan(val):
        return "—"
    if isinstance(val, float):
        return f"{val:.{precision}f}"
    return str(val)


def fmt_ms(m, s):
    if math.isnan(m):
        return "—"
    return f"{m:.1f} ± {s:.1f}"


def fmt_ms_val(m, s):
    if math.isnan(m):
        return "—"
    return f"{m:.2f} ± {s:.2f}"


# ─────────────────────────────────────────────────────────────────────────────
# Main analysis
# ─────────────────────────────────────────────────────────────────────────────

def main():
    rows = load_rows()
    if not rows:
        print("ERROR: master CSV is empty or missing.")
        return

    # Group rows by condition key
    groups = defaultdict(list)
    for r in rows:
        groups[key(r)].append(r)

    # Sorted condition list
    conditions = sorted(groups.keys(),
                        key=lambda k: (k[0], CONTROLLERS.index(k[1])
                                       if k[1] in CONTROLLERS else 99,
                                       k[2], k[3]))

    lines = []

    # ── Header ────────────────────────────────────────────────────────────────
    lines += [
        "# Fair Comparison Sweep — Summary",
        "",
        "**Scenario B** — 6-obstacle, mixed radii.  "
        "Start (0,0,1 m) → Goal (3,3,1 m).  "
        "3 trials per condition (seeds 42–44).  ",
        f"Total rows in master CSV: **{len(rows)}**",
        "",
    ]

    # ── Section 1: Success rate table ─────────────────────────────────────────
    lines += [
        "---",
        "",
        "## Section 1 — Success Rate (per condition)",
        "",
        "| Controller | V_MAX | LOOKAHEAD | Successes/3 | Timeouts | Collisions |",
        "|---|---|---|---|---|---|",
    ]

    for cond in conditions:
        grp = groups[cond]
        sc, ctrl, vm, la = cond
        n_succ = sum(1 for r in grp if r["outcome"] == "success")
        n_tout = sum(1 for r in grp if r["outcome"] == "timeout")
        n_coll = sum(1 for r in grp if r["outcome"] == "collision")
        lines.append(
            f"| {ctrl} | {vm} | {la} | {n_succ}/3 | {n_tout} | {n_coll} |"
        )

    lines += [""]

    # ── Section 2: Averaged metrics (successes only) ───────────────────────────
    lines += [
        "---",
        "",
        "## Section 2 — Averaged Metrics (successful trials only)",
        "",
        "| Controller | V_MAX | LOOKAHEAD | Steps (μ±σ) | Path (m) | "
        "MinClear (m) | AvgSpeed (m/s) | Time2Goal (s) | SafetyViol |",
        "|---|---|---|---|---|---|---|---|---|",
    ]

    for cond in conditions:
        grp    = groups[cond]
        sc, ctrl, vm, la = cond
        succ   = [r for r in grp if r["outcome"] == "success"]
        if not succ:
            lines.append(
                f"| {ctrl} | {vm} | {la} | — | — | — | — | — | — |"
            )
            continue
        steps_ms, steps_ss = mean_std([_f(r["steps"])       for r in succ])
        path_m,   path_s   = mean_std([_f(r["path_length"]) for r in succ])
        clear_m,  clear_s  = mean_std([_f(r["min_clearance"]) for r in succ])
        speed_m,  speed_s  = mean_std([_f(r["avg_speed"])   for r in succ])
        tg_m,     tg_s     = mean_std([_f(r["time_to_goal_s"]) for r in succ])
        sv_m               = safe_mean([_f(r["safety_violation_magnitude"]) for r in succ])
        lines.append(
            f"| {ctrl} | {vm} | {la} | "
            f"{steps_ms:.0f} ± {steps_ss:.0f} | "
            f"{path_m:.2f} ± {path_s:.2f} | "
            f"{clear_m:.3f} ± {clear_s:.3f} | "
            f"{speed_m:.3f} ± {speed_s:.3f} | "
            f"{tg_m:.1f} ± {tg_s:.1f} | "
            f"{sv_m:.4f} |"
        )

    lines += [""]

    # ── Section 3: mt-MPC-only metrics ────────────────────────────────────────
    lines += [
        "---",
        "",
        "## Section 3 — mt-MPC Planner Metrics (all trials)",
        "",
        "| V_MAX | LOOKAHEAD | QP Feas Rate | TTS Rate | Fallbacks | "
        "AvgSolve (ms) | MaxSolve (ms) |",
        "|---|---|---|---|---|---|---|",
    ]

    for cond in conditions:
        sc, ctrl, vm, la = cond
        if ctrl != "mtmpc":
            continue
        grp = groups[cond]
        qpr  = safe_mean([_f(r["qp_feasibility_rate"]) for r in grp])
        ttr  = safe_mean([_f(r["tts_rate"])            for r in grp])
        fb   = safe_mean([_f(r["fallback_count"])      for r in grp])
        asm  = safe_mean([_f(r["avg_solve_ms"])        for r in grp])
        msm  = safe_mean([_f(r["max_solve_ms"])        for r in grp])
        lines.append(
            f"| {vm} | {la} | "
            f"{fmt(qpr, 3)} | {fmt(ttr, 3)} | {fmt(fb, 1)} | "
            f"{fmt(asm, 1)} | {fmt(msm, 1)} |"
        )

    lines += [""]

    # ── Section 4: Key findings ────────────────────────────────────────────────
    lines += [
        "---",
        "",
        "## Section 4 — Key Findings (auto-generated)",
        "",
    ]

    # Aggregate by controller across all conditions (successes only)
    by_ctrl = defaultdict(list)
    for cond in conditions:
        sc, ctrl, vm, la = cond
        succ = [r for r in groups[cond] if r["outcome"] == "success"]
        by_ctrl[ctrl].extend(succ)

    total_by_ctrl = defaultdict(int)
    for cond in conditions:
        sc, ctrl, vm, la = cond
        total_by_ctrl[ctrl] += len(groups[cond])

    # Highest success rate
    succ_rates = {}
    for ctrl in CONTROLLERS:
        total = total_by_ctrl[ctrl]
        nsuc  = len(by_ctrl[ctrl])
        succ_rates[ctrl] = nsuc / total if total > 0 else 0.0

    best_sr = max(succ_rates, key=succ_rates.get)
    lines.append(
        f"**Highest overall success rate:** `{best_sr}` "
        f"({succ_rates[best_sr]*100:.0f}% = "
        f"{len(by_ctrl[best_sr])}/{total_by_ctrl[best_sr]} trials succeeded)\n"
    )

    # Lowest mean path length among successes
    path_means = {}
    for ctrl in CONTROLLERS:
        vals = [_f(r["path_length"]) for r in by_ctrl[ctrl]
                if not math.isnan(_f(r["path_length"]))]
        path_means[ctrl] = safe_mean(vals) if vals else float("nan")

    valid_paths = {c: v for c, v in path_means.items() if not math.isnan(v)}
    if valid_paths:
        best_path = min(valid_paths, key=valid_paths.get)
        lines.append(
            f"**Shortest mean path length:** `{best_path}` "
            f"({valid_paths[best_path]:.2f} m average over successes)\n"
        )

    # Highest min_clearance
    clear_means = {}
    for ctrl in CONTROLLERS:
        vals = [_f(r["min_clearance"]) for r in by_ctrl[ctrl]
                if not math.isnan(_f(r["min_clearance"]))]
        clear_means[ctrl] = safe_mean(vals) if vals else float("nan")

    valid_clear = {c: v for c, v in clear_means.items() if not math.isnan(v)}
    if valid_clear:
        best_clear = max(valid_clear, key=valid_clear.get)
        lines.append(
            f"**Highest mean min_clearance:** `{best_clear}` "
            f"({valid_clear[best_clear]:.3f} m average over successes)\n"
        )

    # Best safety-speed tradeoff: min_clearance / time_to_goal_s
    tradeoff = {}
    for ctrl in CONTROLLERS:
        vals = []
        for r in by_ctrl[ctrl]:
            c = _f(r["min_clearance"])
            t = _f(r["time_to_goal_s"])
            if not math.isnan(c) and not math.isnan(t) and t > 0:
                vals.append(c / t)
        tradeoff[ctrl] = safe_mean(vals) if vals else float("nan")

    valid_to = {c: v for c, v in tradeoff.items() if not math.isnan(v)}
    if valid_to:
        best_to = max(valid_to, key=valid_to.get)
        lines.append(
            f"**Best safety-speed tradeoff** (min_clearance / time_to_goal, "
            f"higher = safer & faster): `{best_to}` "
            f"({valid_to[best_to]:.4f} m/s)\n"
        )

    # mt-MPC under fair settings (vmax=0.5, lookahead=0.10)
    fair_cond_mtmpc = ("B", "mtmpc", 0.5, 0.10)
    fair_cond_apf   = ("B", "apf",   0.5, 0.10)
    fair_cond_cbf   = ("B", "cbf",   0.5, 0.10)

    lines.append("**mt-MPC vs. APF/CBF at fair settings (V_MAX=0.5, LOOKAHEAD=0.10):**\n")
    for cond, label in [
        (fair_cond_apf,   "APF"),
        (fair_cond_cbf,   "CBF-QP"),
        (fair_cond_mtmpc, "mt-MPC"),
    ]:
        grp  = groups.get(cond, [])
        succ = [r for r in grp if r["outcome"] == "success"]
        n    = len(grp)
        ns   = len(succ)
        if succ:
            sm = safe_mean([_f(r["steps"]) for r in succ])
            cm = safe_mean([_f(r["min_clearance"]) for r in succ])
            tm = safe_mean([_f(r["time_to_goal_s"]) for r in succ])
            lines.append(
                f"- `{label}`: {ns}/{n} success, "
                f"avg {sm:.0f} steps / {tm:.1f} s, "
                f"min_clear {cm:.3f} m"
            )
        else:
            lines.append(f"- `{label}`: {ns}/{n} success (no data or all timeouts)")

    lines += [""]

    # ── Footer ────────────────────────────────────────────────────────────────
    lines += [
        "---",
        "",
        f"*Generated by `analyze_sweep.py` from `{MASTER_CSV}`.*",
        "",
    ]

    md = "\n".join(lines)
    with open(OUT_MD, "w") as f:
        f.write(md)
    print(f"Saved: {OUT_MD}")
    print(md)


if __name__ == "__main__":
    main()
