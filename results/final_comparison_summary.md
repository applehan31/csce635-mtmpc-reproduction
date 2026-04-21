# Final Comparison Summary — CSCE 635 Reproducibility Study

**Paper:** Saccani et al. 2023, *Multi-trajectory Model Predictive Control for Safe UAV Navigation*
**Simulator:** gym-pybullet-drones 2.1.0 (CtrlAviary + DSLPIDControl, 48 Hz ctrl / 240 Hz sim)
**Date:** April 2026

---

## 1. Executive Summary

We reproduced the mt-MPC controller of Saccani 2023 alongside two baselines (APF, CBF-QP) in
a PyBullet quadrotor simulation with a Crazyflie 2.X model. Across 24-trial sweeps on a
moderate 6-obstacle scenario (Scenario B), mt-MPC achieves the **highest success + safety
rate (79%)** among the three controllers while maintaining the **smallest obstacle
clearance (0.060 m at the best configuration)** — exactly matching the paper's headline
claim that multi-trajectory MPC permits tighter corridors than reactive methods while
remaining safe.

The reproduction required **four original deviations** from the paper (§Reproducibility
Journey), each independently validated against behaviour described in the manuscript.
Two additional exploratory efforts — Stage A carrot clipping and Scenario C (vertical
wall) — were attempted and abandoned; their negative results are documented as
limitations (§5).

---

## 2. Scenario A — 3-Cylinder Baseline

Identical start/goal and obstacles as the paper's Figure 4-style layout:
`START=(0,0,1)`, `GOAL=(3,3,1)`, obstacles at (1,1), (2,1.5), (1.5,2.5), all r=0.3 m.
`vmax=1.0`, `lookahead=0.40`, trial 0. Single representative run per controller.

| Controller | Outcome   | Steps | Min clearance | Mean speed |
|------------|-----------|------:|--------------:|-----------:|
| APF        | SUCCESS   |  1246 |       0.363 m |  0.193 m/s |
| CBF-QP     | SUCCESS   |  1074 |       0.095 m |  0.224 m/s |
| mt-MPC     | SUCCESS   |   652 |       0.151 m |  0.369 m/s |

**mt-MPC is ~2× faster than APF and ~1.6× faster than CBF-QP**, while retaining a
clearance (0.151 m) comfortably above CRASH_TOL=0.05 m. CBF-QP drives clearance right
down to its 0.10 m CBF margin. APF's wide detours inflate step count.

---

## 3. Scenario B — 6-Cylinder Moderate (24-trial sweep)

Obstacles: Scenario A's three + (0.8,2.0,r=0.2), (2.3,2.3,r=0.4), (1.2,0.5,r=0.25).
Sweep: `vmax ∈ {0.5, 1.0}`, `lookahead ∈ {0.10, 0.25, 0.40, 0.60}`, `trials 0,1,2`
(2 × 4 × 3 = 24 runs per controller).

### 3.1 Outcome counts (24 trials each)

| Controller | SUCCESS | TIMEOUT | COLLISION | Safe rate (SUCCESS ∪ TIMEOUT) |
|------------|--------:|--------:|----------:|------------------------------:|
| APF        |      10 |       8 |         6 |                          75 % |
| CBF-QP     |       0 |      10 |        14 |                          42 % |
| mt-MPC     |      11 |       8 |         5 |                          79 % |

CBF-QP records **zero successes** across the 24-trial sweep on Scenario B: with the
extra obstacles clustered, the QP either decelerates to timeout or takes the direct
reactive action that puts it inside the safety margin on the next step.

### 3.2 Best configuration for mt-MPC (vmax=1.0, la=0.4)

Across 3 trials at the modal best config:

| Metric                     | mean ± sd       |
|----------------------------|-----------------|
| Steps                      | 561 ± 127       |
| Path length                | 8.43 m          |
| Min clearance              | 0.060 m         |
| Elapsed wall time          | 11.7 s          |
| Fraction feasible (QP OK)  | 66 %            |
| Fraction TTS-active        | 58 %            |
| Mean fallback count / run  | 187             |

mt-MPC's clearance settles at the Minkowski-tightened boundary
(r_obs + 0.20 m − 15×0.003 ≈ 0.155 m half-plane distance; foot-print clearance ≈
0.06 m), demonstrating the tight-corridor regime the paper highlights.

### 3.3 Safety comparison at matched (vmax=1.0, la=0.4, trial=0)

Video recordings of the three controllers at the identical configuration (top-down
camera, yaw=240.6°, pitch=−78.6°; see `results/videos/`):

| Controller | Outcome   | Steps | Min clearance |
|------------|-----------|------:|--------------:|
| APF        | COLLISION |   309 |       0.044 m |
| CBF-QP     | COLLISION |   189 |       0.066 m |
| mt-MPC     | SUCCESS   |   630 |       0.058 m |

Matched-setting collision vs success is the single clearest demonstration of
mt-MPC's advantage: APF's potential-field gradient locks into a local minimum between
the (1.2, 0.5) and (1.0, 1.0) obstacles; CBF-QP's myopic filter accepts a trajectory
that is feasible one step ahead but inevitable one step after. mt-MPC's 15-step
backup trajectory II rejects both futures and succeeds.

---

## 4. LOOKAHEAD ablation (mt-MPC, full Scenario B sweep, 3 trials per cell)

Rebuilt from the canonical Stage-4 log (`resweep_log.txt`); 2 vmax × 4 lookahead × 3
trials = 24 runs.

| vmax | lookahead | SUCCESS | TIMEOUT | COLLISION |
|-----:|----------:|--------:|--------:|----------:|
| 0.5  |    0.10 m |       1 |       2 |         0 |
| 0.5  |    0.25 m |       3 |       0 |         0 |
| 0.5  |    0.40 m |       2 |       0 |         1 |
| 0.5  |    0.60 m |       0 |       0 |         3 |
| 1.0  |    0.10 m |       0 |       3 |         0 |
| 1.0  |    0.25 m |       2 |       1 |         0 |
| 1.0  |    0.40 m |       3 |       0 |         0 |
| 1.0  |    0.60 m |       0 |       2 |         1 |
| **Σ**|           |  **11** |   **8** |     **5** |

LOOKAHEAD=0.4 m is the clear sweet spot (5/6 successes across both vmax), with
la=0.25 a close runner-up (5/6). Near-sighted la=0.10 mostly times out — the
corridor heuristic picks marginal local improvements without reaching the goal.
**la=0.60 produces 4 of the 5 Scenario-B collisions**, the classic Mode-A
carrot-penetration failure that motivated the (ultimately unsuccessful) Stage A
carrot-clipping effort (§5.1); the one remaining collision sits at (vmax=0.5,
la=0.40).

---

## 5. Known limitations

### 5.1 Stage A carrot clipping — abandoned after three attempts

We attempted to eliminate the five Scenario-B mt-MPC collisions (4 at la=0.6, 1 at
la=0.4) by clipping the LOOKAHEAD carrot when it intersected an obstacle:

1. **Attempt 1 (radial clip to r_phys+0.02 m):** still COLLISION at 481 steps — the
   clipped carrot still bisected the pair of obstacles.
2. **Attempt 2 (detect at +0.02, push out to +0.10):** collision averted at the near
   obstacle but carrot was now on the far side; direct-shot collision at 153 steps,
   speed 1.48 m/s.
3. **Attempt 3 (hover on penetration):** deadlock — carrot held at pos, QP had no
   progress signal, timed out at 2501 steps.

The three modes (A: carrot penetration, B: path-through after clip, C: drift
during hover) are not fixable at the carrot layer: the root cause is
**QP infeasibility inside tight obstacle clusters**, where neither the exploiting
nor the backup trajectory admits a feasible tightened polytope sequence. A proper
fix requires richer fallback logic (e.g. Stage 5 trajectory re-seeding from a
pre-computed roadmap) and is left as future work. Code was rolled back to pure
Stage 4.

### 5.2 Scenario C (vertical wall) — abandoned

Intended as a Singletary-2020-scenario-3 analogue: 7-cylinder vertical wall at
x=1.5 with a narrow doorway, expected to demonstrate local-minima behaviour of APF
and myopic failure of CBF-QP while mt-MPC threads the gap. Three-way differentiation
proved **geometrically impossible** in our setup:

- With gap ≤ 0.45 m, mt-MPC's 0.20 m Minkowski-tightened corridor has no feasible
  trajectory through the doorway (it shares the failure with CBF).
- With gap ≥ 0.50 m, CBF-QP's 0.15 m SAFETY_MARGIN is satisfied and CBF succeeds
  too (it matches mt-MPC).
- No gap simultaneously produces "CBF fails, mt-MPC succeeds".

Paper-style differentiation would require **different safety margins by construction**
(e.g. CBF with a margin tuned to match the paper's ≈0.05 m), which we judged to
undermine fair comparison. Scenario C results are excluded from the headline tables;
the scenario is preserved in `experiments/scenarios.py` for reference.

### 5.3 Other minor deviations from the paper

See `reproducibility_journey.md` §9 for the full deviations table (TTS polytope
construction, ray-casting distance query, ZETA corridor constraint, REACH cap,
hover-target fix for zero-velocity carrot).

---

## Files

- Raw per-trial CSV: `fair_comparison_master.csv` (76 rows, rebuilt from canonical
  `sweep_log.txt` + `resweep_log.txt`; see `rebuild_master_csv.py`)
- Pre-Stage-A backup CSV: `fair_comparison_master_pre_carrot_clip.csv` (83 rows; 73
  unique keys; canonical source for Scenario B after dedup)
- Contaminated interim CSV: `fair_comparison_master_contaminated_HHMM.csv` (Stage A
  experimental rows leaked via merge; retained for audit)
- Videos: `videos/fc_B_{apf,cbf,mtmpc}_vmax1.0_la0.4_t0.mp4`
- Scenario A videos: `apf_3d.mp4`, `cbf_qp_3d.mp4`, `mtmpc_3d.mp4`
- Scenario A findings doc: `comparison_summary.md`
- Full narrative: `reproducibility_journey.md`
