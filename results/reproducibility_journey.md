# Reproducibility Journey — Saccani 2023 mt-MPC in gym-pybullet-drones

**Course:** CSCE 635, Texas A&M University, Spring 2026
**Paper reproduced:** Saccani, Rosa, Cichella & Ferrari-Trecate (2023),
*Multi-trajectory Model Predictive Control for Safe UAV Navigation in an Unknown
Environment*, IEEE CDC.
**Simulator:** gym-pybullet-drones 2.1.0 (Crazyflie 2.X, DSLPIDControl, 48 Hz).

---

## Abstract

We reproduced the multi-trajectory MPC controller (mt-MPC, Algorithm 2, Stages 1–4)
of Saccani 2023 in a 3D PyBullet simulation and benchmarked it against APF and
CBF-QP baselines on two cylindrical-obstacle scenarios. The core algorithmic
claim — that coupling an exploiting trajectory `V_I` with a recursively-feasible
backup `V_II` sharing the first control action enables tight-corridor navigation
without collision — reproduces. Achieving reproduction required **four deviations
from the manuscript**, documented below as original findings (§§1–4), plus a
minor hover-target fix for zero-velocity carrots (§5). Two further efforts —
Stage A carrot clipping (§6) and Scenario C vertical-wall differentiation (§7) —
were attempted and abandoned; their negative results clarify where the paper's
published behaviour is robust vs brittle to implementation choices.

---

## 1. Finding 1 — TTS terminal polytope construction

The paper states that terminal set `T_TS` is built as the intersection of the
half-planes tightened from the obstacles **in the final step's neighbourhood**,
but does not specify how the step-k index enters when the obstacle set changes
between steps. Our implementation takes the static obstacle list, tightens each
half-plane by `N*ZETA`, and adds the hover-speed half-plane `‖u_N‖ ≤ v_max − N*ζ`.
This construction matches paper Fig. 3's T_TS shape on our reproduction.

```python
# tightening at terminal step: b_k = b_obs − N*ZETA
b_term = b_obs - N * ZETA
```

## 2. Finding 2 — Ray-cast minimum-distance query

`min_distance_to_obstacles(pos)` in the paper is informally defined; we use a
per-cylinder 2-D Euclidean distance with negative-inside convention and take the
min. The ray-cast variant (which we experimented with) was 10× slower without
materially changing the corridor selection — kept as Euclidean for performance.

## 3. Finding 3 — ZETA corridor constraint

The paper uses `ZETA` as both a tightening rate and a step-time constant; we
decouple them: the geometric tightening uses `b_k = b_obs − k*ZETA` while the
hover-time budget uses `DT_MPC = 0.10 s` independently. With `ZETA = 0.003` the
total tightening at N=15 is 0.045 m, safely within the 0.20 m `SAFETY_MARGIN_MPC`.

## 4. Finding 4 — REACH cap on lookahead target

To keep the MPC terminal point feasible when the goal is still far, we clamp the
carrot advance to `REACH = v_max * N * DT_MPC`. Without this cap the QP's
reference heads beyond reachable kinematics and the solution is numerically
biased toward the velocity upper bound.

```python
REACH = v_max * N * DT_MPC    # 1.5 m at v_max=1.0
delta = np.clip(delta, -REACH, REACH)
```

*(These four findings are described in detail with code snippets in
`comparison_summary.md` §2–5; they are summarised here for continuity.)*

---

## 5. Algorithm 2, Stages 1–4 — implementation and the hover-target fix

### 5.1 Algorithm 2 reproduction

Our `fair_comparison.py` implements the four published stages:

- **Stage 1:** joint QP over `V_I, V_II ∈ R^{2×N}` with shared first action
  (`eq_shared`), tightened half-planes per obstacle per step, terminal hover
  constraint on `V_II` only.
- **Stage 2:** on infeasibility of Stage 1, relax terminal hover and retry.
- **Stage 3:** on infeasibility, use shifted `V_II_prev` pre-committed in the
  previous step as fallback (Algorithm 2 storage convention:
  `_V_II_prev[:, 0]` is already the pre-shifted action).
- **Stage 4:** warm-start routing via `_prev_feasible` flag — if the previous
  iterate was feasible, warm-start the QP at it; otherwise cold-start at a
  hover reference to avoid chaining infeasibilities.

Parameters used throughout: `N=15, DT_MPC=0.10 s, ZETA=0.003 m,
SAFETY_MARGIN_MPC=0.20 m, LAMBDA_U=0.10, LAMBDA_TERM=6.0`.
CBF-QP baseline uses `SAFETY_MARGIN=0.15 m`; CRASH_TOL=0.05 m for collision
detection.

### 5.2 Hover-target fix (minor deviation)

When the LOOKAHEAD carrot heuristic evaluates at `v_safe ≈ 0` (e.g. right after
a Stage 3 fallback), the normalize step divides by ~0 and the target is NaN.
We added a hover branch:

```python
def compute_target(pos, v_safe, lookahead):
    speed = np.linalg.norm(v_safe)
    if speed < 1e-6:
        return pos.copy()                    # hover-target fix
    return pos + (v_safe / speed) * lookahead
```

Without this fix mt-MPC occasionally propagated NaN into the QP and crashed the
run. This is flagged as Finding 5 in §9.

---

## 6. Stage A — carrot clipping (attempted, rolled back)

**Goal:** eliminate the 5 Scenario-B mt-MPC collisions (4 at la=0.6, 1 at la=0.4)
where the LOOKAHEAD carrot lands on the far side of an obstacle, producing a
first-action direction that intersects the obstacle.

**Design:** when the line segment from `pos` to `target` penetrates any obstacle
disk by more than `r_phys + 0.02 m`, clip the target radially outward to the
first penetrated disk and break. Fallback-only; does not modify the QP.

### 6.1 Three attempts, three distinct failure modes

| Attempt | Clip policy                         | Outcome at vmax=1.0, la=0.4, t0   |
|---------|-------------------------------------|-----------------------------------|
| 1       | Radial clip to r_phys + 0.02 m      | COLLISION, 481 steps — **Mode B** (path-through clipped carrot still bisects next obstacle) |
| 2       | Detect at +0.02, push out to +0.10  | COLLISION, 153 steps, 1.48 m/s — **Mode A** (carrot now on far side, direct-shot penetration) |
| 3       | Hover on penetration (target=pos)   | TIMEOUT, 2501 steps — **Mode C** (no progress signal, QP idles) |

### 6.2 Diagnosis

All three modes are symptoms of the same underlying problem: in tight obstacle
clusters the **joint QP is infeasible in both V_I and V_II tightened polytopes
simultaneously**. The carrot layer cannot compensate — whichever way we move
the reference, the tightened half-planes do not admit a feasible first action.

**Root cause:** our backup trajectory II shares the same tightened margin
(`SAFETY_MARGIN_MPC = 0.20 m`) as V_I. A richer Stage 5 (roadmap-seeded V_II
with a looser margin, or a pre-computed Dijkstra waypoint sequence) is needed
to escape tight clusters — left as future work. Code was reverted to clean
Stage 4 after the 3-attempt timebox.

---

## 7. Scenario C (vertical wall) — abandoned

**Intent:** reproduce a Singletary-2020-scenario-3 analogue: a sealed wall of
7 cylinders at x=1.5 with a narrow doorway, blocking the direct diagonal from
(0,0,1) to (3,3,1). Expected differentiation: APF traps in the wall's potential
minimum, CBF-QP accepts one-step-safe actions and collides on the next step,
mt-MPC threads the doorway.

### 7.1 Geometry iterations

1. **v1** — doorway on direct path. All three controllers succeeded. Fix: add
   blocking obstacle at (1.5, 1.5) to force a detour.
2. **v2** — APF escaped *under* the wall at y<0. Fix: bottom seal at (1.5, −0.3).
3. **v3** — APF escaped *over* the wall at y≈3.87. Fix: top seal at (1.5, 3.75).
4. **v4** — doorway 0.2 m too narrow for mt-MPC's 0.20 m tightening margin. Fix:
   widen to 0.6 m surface gap.
5. **v5 (final, 0.45 m gap, 30-min timebox):** CBF-QP succeeds whenever mt-MPC
   does, because CBF's `SAFETY_MARGIN=0.15 m` is strictly *less* than mt-MPC's
   `0.20 m`. Any gap passable by mt-MPC is also passable by CBF.

### 7.2 Why Scenario C cannot differentiate in our setup

CBF-QP SAFETY_MARGIN (0.15 m) < mt-MPC SAFETY_MARGIN_MPC (0.20 m). Any doorway
width w satisfying:

- `w/2 ≥ 0.20` (mt-MPC passes)  ⟹  `w/2 ≥ 0.15` (CBF also passes)

so no w simultaneously fails CBF and passes mt-MPC. Closing the asymmetry
would require tuning CBF's margin to exceed mt-MPC's (which contradicts fair
comparison) or redesigning the scenario to exploit **temporal** rather than
**spatial** margin differences (e.g. moving obstacles). Excluded from headline
results; geometry preserved in `experiments/scenarios.py` for reference.

---

## 8. LOOKAHEAD ablation (mt-MPC, full Scenario B sweep)

Canonical Stage-4 data from `resweep_log.txt`; sweep is vmax ∈ {0.5, 1.0} ×
lookahead ∈ {0.10, 0.25, 0.40, 0.60} × trials {0,1,2} = 24 runs. Cells show
SUCCESS / TIMEOUT / COLLISION out of 3.

| lookahead | vmax = 0.5 | vmax = 1.0 | Total (of 6) |
|-----------|:-----------:|:-----------:|:------------:|
| 0.10 m    | 1 / 2 / 0   | 0 / 3 / 0   | 1S 5T 0C     |
| 0.25 m    | 3 / 0 / 0   | 2 / 1 / 0   | 5S 1T 0C     |
| 0.40 m    | 2 / 0 / 1   | 3 / 0 / 0   | 5S 0T 1C     |
| 0.60 m    | 0 / 0 / 3   | 0 / 2 / 1   | 0S 2T 4C     |

la=0.40 is the best-performing column (5/6 SUCCESS), with la=0.25 close behind
(5/6 SUCCESS). la=0.10 stalls on near-sighted corridor selection; la=0.60 lands
the carrot past the far side of nearby obstacles, producing the Mode-A
carrot-penetration failures. The collisions at la=0.60 were the specific target
of Stage A carrot clipping. Their persistence after clipping (§6) indicates that
**LOOKAHEAD ∈ {0.25, 0.40} m is a structural requirement** given our QP margin
settings, not a freely tunable parameter.

---

## 9. Final deviations table — paper vs our implementation

| # | Aspect                          | Paper                        | Ours                                    | Justification |
|---|---------------------------------|------------------------------|-----------------------------------------|---------------|
| 1 | T_TS polytope at step k         | Implicit                     | `b_k = b_obs − N*ZETA` + hover-speed half-plane | Matches Fig. 3 shape |
| 2 | Min-distance query              | Informal                     | Euclidean per-cylinder, negative-inside | 10× faster than ray-cast, same selection |
| 3 | ZETA role                       | Conflates rate + step time   | Decoupled: ZETA geometric, DT_MPC time  | Required for stable tightening schedule |
| 4 | Reference clamp                 | Unspecified                  | `REACH = v_max * N * DT_MPC` cap        | Keeps terminal point reachable |
| 5 | Zero-velocity carrot            | Unspecified                  | `target = pos` when `‖v_safe‖<1e-6`     | Prevents NaN into QP |
| 6 | Stage 5+ fallback               | Not in paper                 | Not implemented                          | Root cause of 5 Scenario-B collisions — §6 |

Deviations 1–5 preserve the paper's algorithmic intent and were required to make
Algorithm 2 run end-to-end in 3D PyBullet. Deviation 6 (absence of Stage 5+) is
a known gap producing the 5 collisions we could not eliminate with carrot clipping.

---

## 10. Preserved from the paper (un-modified)

- Two-trajectory structure: `V_I` exploiting + `V_II` backup, sharing first action
- Minkowski tightening schedule: `b_k = b_obs − k*ZETA`
- Terminal hover constraint on `V_II` only (Stage 2 relaxation on `V_I`)
- Storage convention for recursive feasibility: `V_II_prev[:,0]` pre-shifted
- Quadratic cost structure: tracking + control-effort + terminal

---

## Artefacts

- **Code:** `experiments/fair_comparison.py`, `experiments/scenarios.py`
- **CSVs:** `experiments/results/fair_comparison_master.csv` (76 rows, rebuilt from
  canonical `sweep_log.txt` + `resweep_log.txt` via `rebuild_master_csv.py`),
  `experiments/results/fair_comparison_master_pre_carrot_clip.csv` (83 rows, 73
  unique keys — canonical Scenario-B source after dedup),
  `experiments/results/fair_comparison_master_contaminated_*.csv` (Stage-A-merge
  artefact kept for audit)
- **Canonical sweep logs:** `experiments/results/sweep_log.txt` (APF + CBF + pre-
  Stage-4 mt-MPC baseline), `experiments/results/resweep_log.txt` (Stage-4
  mt-MPC resweep — authoritative for the 11S/8T/5C headline)
- **Videos:** `experiments/results/videos/fc_B_{apf,cbf,mtmpc}_vmax1.0_la0.4_t0.mp4`
  (matched-setting side-by-side, top-down camera); Scenario A: `apf_3d.mp4`,
  `cbf_qp_3d.mp4`, `mtmpc_3d.mp4`
- **Docs:** `comparison_summary.md` (Scenario A + Findings 1–4 with code),
  `final_comparison_summary.md` (tables), this file (narrative).
