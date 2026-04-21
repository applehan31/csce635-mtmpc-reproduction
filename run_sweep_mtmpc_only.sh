#!/bin/bash
# run_sweep_mtmpc_only.sh — Re-run Scenario B mt-MPC sweep with Algorithm 2 fix.
# APF and CBF-QP results from original sweep are untouched.
#
# 24 trials: 2 vmax × 4 lookahead × 3 seeds
#
# Usage:
#   cd ~/gym-pybullet-drones/experiments
#   bash run_sweep_mtmpc_only.sh 2>&1 | tee results/resweep_log.txt

set -e
cd ~/gym-pybullet-drones/experiments
source ~/miniconda3/etc/profile.d/conda.sh
conda activate drones

SCENARIO="B"
CTRL="mtmpc"
TOTAL=24
COUNT=0

for VMAX in 0.5 1.0; do
  for LOOKAHEAD in 0.10 0.25 0.40 0.60; do
    for TRIAL in 0 1 2; do
      COUNT=$((COUNT + 1))
      echo ""
      echo "=== [${COUNT}/${TOTAL}] mtmpc vmax=${VMAX} lookahead=${LOOKAHEAD} trial=${TRIAL} ==="
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

echo ""
echo "================================================================"
echo "  Re-sweep complete (${COUNT} mt-MPC trials)."
echo "  Master CSV : results/fair_comparison_master.csv"
echo "================================================================"
