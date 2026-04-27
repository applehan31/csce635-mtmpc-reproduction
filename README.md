# Multi-trajectory MPC for Safe UAV Navigation — Reproduction

Reproduction of Saccani et al. 2023, "Multi-trajectory Model Predictive
Control for Safe UAV Navigation in an Unknown Environment" (IEEE CDC),
in a 3D PyBullet simulation environment.

CSCE 635: AI Robotics, Texas A&M University, Spring 2026
Reproducibility Track

## Authors
- Yonghee Han (applehan31@tamu.edu)
- Pranav Jose (pranavj@tamu.edu)

## Overview

This project benchmarks three controllers on UAV obstacle-avoidance tasks
in a 3D PyBullet simulator (Crazyflie 2.X model, gym-pybullet-drones 2.1.0):

- APF (Khatib 1986)
- CBF-QP (Singletary 2020)
- mt-MPC (Saccani 2023) — main reproduction target

## Headline Results (Scenario B, 24 trials each)

| Controller | Success | Timeout | Collision | Safe rate |
|------------|---------|---------|-----------|-----------|
| APF        | 10      | 8       | 6         | 75%       |
| CBF-QP     | 0       | 10      | 14        | 42%       |
| mt-MPC     | 11      | 8       | 5         | 79%       |

At the recommended parameters (`vmax=1.0`, `lookahead=0.40`), mt-MPC
attains a 100% success rate over 3 trials with 0.060 m minimum clearance.

## Installation

Requires WSL2 Ubuntu or native Linux, conda.

```bash
git clone https://github.com/utiasDSL/gym-pybullet-drones.git
cd gym-pybullet-drones
conda env create -f environment.yml
conda activate drones
pip install -e .
```

Then place this repository's contents under
`gym-pybullet-drones/experiments/`.

## Quick Start

Single trial (Scenario A, mt-MPC):

```bash
cd experiments
python fair_comparison.py --scenario A --controller mtmpc \
    --vmax 1.0 --lookahead 0.40 --trial 0
```

Full Scenario B sweep (72 runs, ~2 hours):

```bash
bash run_sweep.sh
```

## File Structure

- `apf_3d.py`, `cbf_qp_3d.py`, `mtmpc_3d.py` — standalone controller
  implementations (Scenario A demos)
- `fair_comparison.py` — unified runner used for the parameter sweep
- `scenarios.py` — obstacle layouts (Scenario A, B, C)
- `analyze_sweep.py`, `collision_analysis.py` — post-hoc analysis
- `plot_trajectories.py` — generates the trajectory and speed-profile
  figures used in the report
- `rebuild_master_csv.py` — reconstructs the master CSV from canonical
  log files
- `run_sweep.sh`, `run_sweep_mtmpc_only.sh` — batch experiment scripts
- `record_scenario_b_final.sh` — top-down video recording script
- `results/` — CSV logs, sweep logs, and trajectory plots
- `reproducibility_journey.md` — narrative of implementation findings
- `final_comparison_summary.md` — final results tables
- `comparison_summary.md` — Scenario A and the four original
  reproducibility findings

## Key Findings

The reproducibility narrative is documented in
`reproducibility_journey.md`. Five deviations from the original paper
were required to make Algorithm 2 work end-to-end in 3D PyBullet,
including velocity-space MPC reformulation and a from-scratch
implementation of Algorithm 2 stages 1 through 4 (which were absent
from the 2D reference code we built upon).

## Videos

Demonstration videos (Scenario A and B for each controller) are hosted
on Google Drive: [link to be added]

## Citation

Saccani, D., Rosa, P., Cichella, V., and Ferrari-Trecate, G. (2023).
"Multi-trajectory Model Predictive Control for Safe UAV Navigation
in an Unknown Environment." Proc. IEEE 62nd Conference on Decision
and Control (CDC).
