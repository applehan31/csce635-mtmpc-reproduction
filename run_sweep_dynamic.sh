#!/bin/bash
set -e
cd ~/gym-pybullet-drones/experiments
source ~/miniconda3/etc/profile.d/conda.sh
conda activate drones

TOTAL=9
COUNT=0

for CTRL in apf cbf mtmpc; do
  for TRIAL in 0 1 2; do
    COUNT=$((COUNT + 1))
    echo ""
    echo "=== [${COUNT}/${TOTAL}] dynamic ctrl=${CTRL} trial=${TRIAL} ==="
    python fair_comparison_dynamic.py \
      --controller "${CTRL}" \
      --vmax      1.0 \
      --lookahead 0.40 \
      --trial     "${TRIAL}" \
      --no-gui
  done
done

echo ""
echo "Dynamic sweep complete (${COUNT} runs)"
