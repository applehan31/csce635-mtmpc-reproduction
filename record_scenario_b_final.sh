#!/bin/bash
set -e
cd ~/gym-pybullet-drones/experiments
source ~/miniconda3/etc/profile.d/conda.sh
conda activate drones

mkdir -p results/videos

# All three use identical settings (vmax=1.0, la=0.40, trial=0) for
# direct side-by-side comparison. Camera: top-down, yaw=240.6, pitch=-78.6.

# APF collision — 309 steps, clear=0.044m (longest collision run in CSV)
echo "=== Recording Scenario B — APF collision ==="
python fair_comparison.py --scenario B --controller apf \
  --vmax 1.0 --lookahead 0.40 --trial 0

# CBF-QP collision at same config (myopic, 189 steps)
echo "=== Recording Scenario B — CBF-QP collision ==="
python fair_comparison.py --scenario B --controller cbf \
  --vmax 1.0 --lookahead 0.40 --trial 0

# mt-MPC success at same config (630 steps, clear=0.058m)
echo "=== Recording Scenario B — mt-MPC success ==="
python fair_comparison.py --scenario B --controller mtmpc \
  --vmax 1.0 --lookahead 0.40 --trial 0

echo "All Scenario B recordings complete with new top-down camera."
