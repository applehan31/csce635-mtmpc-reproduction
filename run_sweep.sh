#!/bin/bash
# run_sweep.sh — Full parameter sweep for fair controller comparison.
#
# Scenario B only (Scenario A already has videos/data from individual runs).
# Controllers : apf, cbf, mtmpc
# V_MAX       : 0.5 (conservative), 1.0 (aggressive)
# LOOKAHEAD   : 0.10, 0.25, 0.40, 0.60
# Trials      : 3 per condition (seeds 42, 43, 44)
#
# Total: 1 × 3 × 2 × 4 × 3 = 72 runs
#
# Usage:
#   cd ~/gym-pybullet-drones/experiments
#   ./run_sweep.sh 2>&1 | tee results/sweep_log.txt

set -e
cd ~/gym-pybullet-drones/experiments
source ~/miniconda3/etc/profile.d/conda.sh
conda activate drones

SCENARIO="B"
TOTAL=72
COUNT=0

for CTRL in apf cbf mtmpc; do
  for VMAX in 0.5 1.0; do
    for LOOKAHEAD in 0.10 0.25 0.40 0.60; do
      for TRIAL in 0 1 2; do
        COUNT=$((COUNT + 1))
        echo ""
        echo "=== [${COUNT}/${TOTAL}] ctrl=${CTRL} vmax=${VMAX} lookahead=${LOOKAHEAD} trial=${TRIAL} ==="
        python fair_comparison.py \
          --scenario  ${SCENARIO} \
          --controller ${CTRL} \
          --vmax      ${VMAX} \
          --lookahead ${LOOKAHEAD} \
          --trial     ${TRIAL} \
          --no-gui
      done
    done
  done
done

echo ""
echo "================================================================"
echo "  Sweep complete (${COUNT} runs)."
echo "  Master CSV : results/fair_comparison_master.csv"
echo "================================================================"
