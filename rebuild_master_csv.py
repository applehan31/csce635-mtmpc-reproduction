"""Rebuild fair_comparison_master.csv from canonical sources.

Canonical sources:
  - Scenario A mtmpc: 1 row from the contaminated CSV (unchanged by Stage A)
  - Scenario B (all 3 controllers): 72 rows from backup CSV, deduped keep-first
      (backup's first-per-key rows match sweep_log.txt and resweep_log.txt)
  - Scenario C: 3 rows from the contaminated CSV (pristine — unaffected by Stage A)
"""
import pandas as pd

CURRENT = "results/fair_comparison_master.csv"
BACKUP = "results/fair_comparison_master_pre_carrot_clip.csv"
OUT = "results/fair_comparison_master.csv"

cur = pd.read_csv(CURRENT)
bak = pd.read_csv(BACKUP)

KEY = ["scenario", "controller", "vmax", "lookahead", "trial"]

# (1) Scenario A mtmpc: take from current (single row, unchanged)
sA = cur[cur["scenario"] == "A"].copy()

# (2) Scenario B: from backup, dedup keep-first (canonical Stage-4 / sweep)
sB_backup = bak[bak["scenario"] == "B"].copy()
sB = sB_backup.drop_duplicates(subset=KEY, keep="first")

# (3) Scenario C: from current (3 rows, not in backup)
sC = cur[cur["scenario"] == "C"].copy()

out = pd.concat([sA, sB, sC], ignore_index=True)
out.to_csv(OUT, index=False)

print(f"Wrote {len(out)} rows to {OUT}")
print("\nBy (scenario, controller):")
print(out.groupby(["scenario", "controller"]).size().to_string())
print("\nScenario B outcomes:")
print(out[out["scenario"] == "B"].groupby("controller")["outcome"].value_counts().to_string())
print("\nmt-MPC Scenario B by (vmax, la):")
mt = out[(out["scenario"] == "B") & (out["controller"] == "mtmpc")]
print(mt.groupby(["vmax", "lookahead", "outcome"]).size().unstack(fill_value=0).to_string())
