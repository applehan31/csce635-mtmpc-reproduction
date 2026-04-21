# mt-MPC 3D Reproducibility Study
*CSCE 635 AI Robotics — Spring 2026*

Reproduction of Saccani 2023 **Multi-trajectory Model Predictive Control** in
PyBullet for safe UAV navigation, with a fair side-by-side comparison against
Artificial Potential Field (APF) and Control Barrier Function QP (CBF-QP)
baselines on a Crazyflie 2.X model.

## Authors

Yonghee Han, Pranav Jose

## Paper Reference

Saccani, Rosa, Cichella & Ferrari-Trecate (2023), *"Multi-trajectory Model
Predictive Control for Safe UAV Navigation in an Unknown Environment"*,
IEEE Conference on Decision and Control (CDC).

## Key Findings

Three-controller comparison across two obstacle scenarios.

### Scenario B (6 cylindrical obstacles, 24-trial sweep per controller)

| Controller | Success | Timeout | Collision | Safe rate |
|------------|--------:|--------:|----------:|----------:|
| APF        |      10 |       8 |         6 |      75 % |
| CBF-QP     |       0 |      10 |        14 |      42 % |
| **mt-MPC** |  **11** |   **8** |     **5** |  **79 %** |

At the recommended parameters (`vmax=1.0, lookahead=0.40`), mt-MPC achieves
**3/3 successes** with **0.060 m minimum clearance** — demonstrating the
paper's recursive-feasibility guarantee while operating at the
Minkowski-tightened corridor boundary.

See `results/final_comparison_summary.md` for the full results tables and
`results/reproducibility_journey.md` for the complete narrative.

## Environment Setup

> **Note:** This repository contains only our experimental code. To run the
> experiments, first install `gym-pybullet-drones` 2.1.0 from
> <https://github.com/utiasDSL/gym-pybullet-drones>, then place this folder
> as `experiments/` inside the cloned `gym-pybullet-drones` directory. The
> `environment.yml` / `requirements.txt` here supplement the base
> installation with the Python packages our controllers need (CVXPY, OSQP,
> control, etc.).

**Required**
- Ubuntu 20.04+ (native or WSL2)
- Python 3.10
- `conda` (Miniconda or Anaconda)

**Installation**

```bash
# 1. Install the base simulator
git clone https://github.com/utiasDSL/gym-pybullet-drones.git
cd gym-pybullet-drones
pip install -e .

# 2. Drop this repo in as the experiments/ folder
git clone https://github.com/applehan31/csce635-mtmpc-reproduction.git experiments

# 3. Activate the environment (conda path)
cd experiments
conda env create -f environment.yml
conda activate drones
# ...or pip-only fallback: pip install -r requirements.txt
```

GUI runs (video recording) require an X server. On WSL2 use WSLg (Windows 11)
or an X11 forwarding server; verify with `echo $DISPLAY`.

## Running Experiments

### Single trial (fast, headless)

```bash
cd experiments
python fair_comparison.py --scenario A --controller mtmpc \
    --vmax 1.0 --lookahead 0.40 --trial 0 --no-gui
```

Controllers: `apf` | `cbf` | `mtmpc`. Scenarios: `A` (3 obstacles) |
`B` (6 obstacles) | `C` (vertical wall, experimental).

### Full 24-trial sweep for Scenario B

```bash
bash run_sweep_mtmpc_only.sh
```

Output is appended to `results/fair_comparison_master.csv`.

### With GUI (video recording)

```bash
python fair_comparison.py --scenario B --controller mtmpc \
    --vmax 1.0 --lookahead 0.40 --trial 0
```

The three matched-setting Scenario-B videos are produced by:

```bash
bash record_scenario_b_final.sh
```

## File Structure

```
experiments/
├── fair_comparison.py          # main experiment runner (all 3 controllers)
├── scenarios.py                # Scenario A, B, C definitions
├── apf_3d.py                   # standalone APF controller (Scenario A only)
├── cbf_qp_3d.py                # standalone CBF-QP controller (Scenario A only)
├── mtmpc_3d.py                 # standalone mt-MPC controller (Scenario A only)
├── run_sweep.sh                # full 3-controller Scenario B sweep
│                               #   (produces results/sweep_log.txt — canonical
│                               #    APF/CBF/baseline-mt-MPC data source)
├── run_sweep_mtmpc_only.sh     # Stage-4 mt-MPC resweep (produces resweep_log.txt)
├── record_scenario_b_final.sh  # video recording script (top-down camera)
├── rebuild_master_csv.py       # rebuild master CSV from canonical logs
├── analyze_sweep.py            # post-hoc sweep analysis (Table 3.2 statistics)
├── collision_analysis.py       # Scenario B collision mode taxonomy (§6 Mode A/B/C)
├── requirements.txt            # pip-freeze fallback env spec
└── results/
    ├── fair_comparison_master.csv                   # canonical experiment data (76 rows)
    ├── fair_comparison_master_pre_carrot_clip.csv   # pre-Stage-A backup
    ├── fair_comparison_master_contaminated_*.csv    # Stage-A merge artefact (audit)
    ├── final_comparison_summary.md                  # final results summary
    ├── reproducibility_journey.md                   # reproducibility narrative
    ├── comparison_summary.md                        # Scenario A + original 4 findings
    ├── sweep_log.txt                                # canonical APF+CBF+baseline log
    │                                                #   (produced by run_sweep.sh)
    ├── resweep_log.txt                              # canonical Stage-4 mt-MPC log
    │                                                #   (produced by run_sweep_mtmpc_only.sh)
    ├── apf_3d_log.csv                               # Scenario A APF trajectory (raw per-step)
    ├── cbf_qp_3d_log.csv                            # Scenario A CBF trajectory (raw per-step)
    ├── mtmpc_3d_log.csv                             # Scenario A mt-MPC trajectory (raw per-step)
    ├── apf_3d.mp4 / cbf_qp_3d.mp4 / mtmpc_3d.mp4    # Scenario A videos
    └── videos/
        └── fc_B_{apf,cbf,mtmpc}_vmax1.0_la0.4_t0.mp4    # Scenario B matched-setting
```

`sweep_log.txt` is the canonical APF + CBF + pre-Stage-4 mt-MPC data source,
produced end-to-end by `bash run_sweep.sh`. `resweep_log.txt` is the
Stage-4-only mt-MPC re-run, produced by `bash run_sweep_mtmpc_only.sh`. Both
feed `rebuild_master_csv.py`, which reconstructs the 76-row master CSV.

## Reproducibility

Full implementation journey and deviations from the paper are in
`results/reproducibility_journey.md`. Key deviations (with rationale):

1. **Double-integrator → velocity-space MPC** — the paper's
   double-integrator formulation diverged in PyBullet; a velocity-space
   reformulation on `[px, py]` state / `[vx, vy]` input reproduces the
   published behaviour.
2. **Ray-casting TTS projection** — paper unspecified; we use Euclidean
   per-cylinder distance, negative-inside.
3. **ZETA corridor-width constraint** — decoupled from step time
   (`DT_MPC=0.10 s` independent of `ZETA=0.003 m`).
4. **Algorithm 2 full 4-stage implementation** — Stage 1 joint QP, Stage 2
   terminal-hover relaxation, Stage 3 shifted-V_II fallback, Stage 4
   warm-start routing via `_prev_feasible` flag.
5. **Hover-target fix for zero-velocity carrot** — prevents NaN when
   `‖v_safe‖ ≈ 0` after a Stage-3 fallback.

## Known Limitations

- **5/24 collisions** in the Scenario B mt-MPC sweep occur at
  non-recommended LOOKAHEAD settings: 4 at `lookahead=0.60`, 1 at
  (`vmax=0.5`, `lookahead=0.40`). None at the recommended
  (`vmax=1.0, la=0.40`) configuration.
- **Stage A carrot clipping** — 3 surgical-clipping attempts were
  documented and rolled back; the underlying QP infeasibility in tight
  obstacle clusters is not fixable at the carrot layer. See
  `results/reproducibility_journey.md` §6.
- **Scenario C (vertical-wall doorway) abandoned** — CBF-QP's safety
  margin (0.15 m) is strictly less than mt-MPC's (0.20 m), so no single
  doorway width simultaneously fails CBF and passes mt-MPC. The scenario
  geometry is preserved in `scenarios.py` for future work. See §7.

## License

Academic use only — CSCE 635 course project, Texas A&M University,
Spring 2026.
