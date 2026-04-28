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

## Prerequisites

- WSL2 Ubuntu 20.04+ (or native Linux)
- Miniconda or Anaconda
- Python 3.10
- ~2 GB free disk space (sweep logs included)
- **GPU is not required** — the simulation runs on CPU

PyBullet GUI is optional. With WSL2, GUI windows require WSLg
(Windows 11) or an X11 forwarder; otherwise pass `--no-gui` to every
command below.

## Installation

Three steps. Copy-paste ready.

**Step 1.** Clone the parent simulator (`gym-pybullet-drones` 2.1.0):

```bash
git clone https://github.com/utiasDSL/gym-pybullet-drones.git
cd gym-pybullet-drones
```

**Step 2.** Clone our reproduction repository into `experiments/`. The
parent repo already contains a folder named `experiments/` with example
scripts; replace it with this repo:

```bash
rm -rf experiments
git clone https://github.com/applehan31/csce635-mtmpc-reproduction.git experiments
cd experiments
```

(If you would rather keep the original `experiments/` folder around,
clone us anywhere else and copy the contents in — the scripts only
require their `results/` subdirectory and the `gym_pybullet_drones`
Python package on the import path.)

**Step 3.** Create the conda environment and install
`gym-pybullet-drones` in editable mode:

```bash
conda env create -f environment.yml
conda activate drones
pip install -e ../          # installs gym-pybullet-drones from the parent
```

Verification: `python -c "import gym_pybullet_drones; import cvxpy; print('ok')"`
should print `ok`.

## Quick Start

All commands below are run from the `experiments/` directory with the
`drones` conda env active.

**Single trial — Scenario A, mt-MPC** (PyBullet GUI on; ~5 s wall time):

```bash
python fair_comparison.py --scenario A --controller mtmpc \
    --vmax 1.0 --lookahead 0.40 --trial 0
```

**Single trial — Scenario B, mt-MPC headless** (recommended config;
~12 s wall time):

```bash
python fair_comparison.py --scenario B --controller mtmpc \
    --vmax 1.0 --lookahead 0.40 --trial 0 --no-gui
```

**Full Scenario B sweep** — all 3 controllers × 2 vmax × 4 lookahead × 3
trials = **72 runs**, ~2 hours total:

```bash
bash run_sweep.sh
```

**mt-MPC re-sweep only** — Stage-4 mt-MPC × 24 configs, ~30 minutes:

```bash
bash run_sweep_mtmpc_only.sh
```

CLI flags accepted by `fair_comparison.py`:

| flag           | values                       | default |
|----------------|------------------------------|---------|
| `--scenario`   | `A` / `B` / `C`              | `A`     |
| `--controller` | `apf` / `cbf` / `mtmpc`      | (required) |
| `--vmax`       | float, m/s                   | 0.5     |
| `--lookahead`  | float, m                     | 0.10    |
| `--trial`      | int (used as random seed)    | 0       |
| `--seed`       | int (overrides trial seed)   | None    |
| `--no-gui`     | flag — disables PyBullet GUI | off     |

## Expected Outputs

After any run completes:

- `results/fair_comparison_master.csv` is appended with one row carrying
  outcome (`success` / `timeout` / `collision`), step count, path
  length, minimum clearance, average speed, QP feasibility rate, etc.
- The console prints a one-line summary, e.g.
  `[B|mtmpc|vmax=1.0|la=0.4|t0] SUCCESS in 630 steps | clear=0.058m | speed=0.683 m/s`.
- If GUI was on, an MP4 video is written to `results/` (or
  `results/videos/` for fair-comparison runs).

After a full sweep, `results/sweep_log.txt` contains every per-trial
summary line, and `python rebuild_master_csv.py` can rebuild the master
CSV from the canonical logs (used to recover from accidental row
duplications).

## Headline Results (Scenario B, 24 trials each)

| Controller | Success | Timeout | Collision | Safe rate |
|------------|---------|---------|-----------|-----------|
| APF        | 10      | 8       | 6         | 75%       |
| CBF-QP     | 0       | 10      | 14        | 42%       |
| mt-MPC     | 11      | 8       | 5         | 79%       |

At the recommended parameters (`vmax=1.0`, `lookahead=0.40`), mt-MPC
attains a 100% success rate over 3 trials with 0.060 m minimum
clearance. Full tables, the LOOKAHEAD ablation, and the implementation
narrative are in `final_comparison_summary.md` and
`reproducibility_journey.md`.

## Troubleshooting

**`ModuleNotFoundError: No module named 'gym_pybullet_drones'`**
You ran a script before `pip install -e ../` from inside `experiments/`.
Re-run that step with the `drones` env active.

**`ModuleNotFoundError: No module named 'cvxpy'` (or `osqp`, `scipy`)**
The conda environment is not activated, or `environment.yml` was not
applied. Re-run `conda env create -f environment.yml; conda activate drones`.

**`scipy.optimize.minimize` SLSQP warnings or errors**
Ensure `scipy >= 1.11`. The CBF-QP and mt-MPC fallback paths rely on
SLSQP behaviour from that version onward. `pip install --upgrade scipy`
inside the env resolves it.

**PyBullet GUI window does not appear (WSL2)**
You need WSLg (Windows 11) or an X11 server (e.g. VcXsrv). Verify
`echo $DISPLAY` returns a non-empty value before running. If you cannot
get the GUI working, every script accepts `--no-gui` to run headless;
all CSV outputs are produced identically.

**A run hangs at the start of step 0 with no output**
PyBullet is waiting for the GUI window to be acknowledged. Press any
key in the GUI window, or re-run with `--no-gui`.

**Master CSV has duplicate rows**
Running the same `(scenario, controller, vmax, lookahead, trial)`
combination twice appends a duplicate row. Run
`python rebuild_master_csv.py` to dedupe from the canonical logs.

## File Structure

- `apf_3d.py`, `cbf_qp_3d.py`, `mtmpc_3d.py` — standalone controller
  implementations (Scenario A demos)
- `fair_comparison.py` — unified runner used for the parameter sweep
- `scenarios.py` — obstacle layouts (Scenario A, B, C)
- `analyze_sweep.py`, `collision_analysis.py` — post-hoc analysis
- `plot_trajectories.py`, `make_report_figures.py` — figure generators
- `rebuild_master_csv.py` — rebuild master CSV from canonical logs
- `run_sweep.sh`, `run_sweep_mtmpc_only.sh` — batch experiment scripts
- `record_scenario_b_final.sh` — top-down video recording script
- `results/` — CSV logs, sweep logs, trajectory plots, report figures
- `reproducibility_journey.md` — narrative of implementation findings
- `final_comparison_summary.md` — final results tables
- `comparison_summary.md` — Scenario A and four original findings

## Videos

Demonstration videos (Scenario A and B for each controller) are hosted
on Google Drive: **[link to be added]**

## Citation

Saccani, D., Rosa, P., Cichella, V., and Ferrari-Trecate, G. (2023).
"Multi-trajectory Model Predictive Control for Safe UAV Navigation
in an Unknown Environment." Proc. IEEE 62nd Conference on Decision
and Control (CDC).
