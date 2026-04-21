# CSCE 635 Reproducibility — 3-Way Controller Comparison

**Course:** CSCE 635, Texas A&M University  
**Paper:** Saccani et al. 2023, "Multi-trajectory Model Predictive Control for Safe UAV Navigation"  
**Simulator:** gym-pybullet-drones 2.1.0 (PyBullet, `CtrlAviary` + `DSLPIDControl`)  
**Scene:** Start (0, 0, 1 m) → Goal (3, 3, 1 m), three cylindrical obstacles (r = 0.5 m) at (1,1), (2,1.5), (1.5,2.5)

---

## 3-Way Quantitative Comparison

| Metric | APF | CBF-QP | **mt-MPC** |
|---|---|---|---|
| **Reference** | Khatib 1986 | Singletary 2020 | Saccani 2023 |
| **Steps to goal** | 1246 | 1074 | **652** |
| **Min clearance (m)** | 0.363 | 0.095 | 0.151 |
| **Max clearance (m)** | 1.799 | 1.590 | 1.141 |
| **QP infeasible steps** | N/A | 0 | **0** |
| **Max solve time (ms)** | N/A | N/A | 291.7 |
| **TTS activation rate** | N/A | N/A | 76.5% (499/652) |
| **Fallback used** | N/A | N/A | **0** |

*Steps counted from liftoff until `dist_to_goal < 0.15 m` threshold. Clearance = distance to nearest obstacle surface = `dist_to_obstacle_center − r_obstacle`.*

---

## Key Findings

### 1. mt-MPC is the fastest controller: 47.5% fewer steps than APF

mt-MPC reaches the goal in **652 steps** versus 1246 for APF and 1074 for CBF-QP — a 47.6% and 39.3% reduction respectively. The gain comes from the MPC horizon planning 1.5 m ahead (N=15, V_MAX=1.0 m/s, DT=0.1 s), allowing the optimizer to commit to efficient curved paths around the obstacle cluster rather than reacting locally.

### 2. Zero QP infeasibilities across all 652 steps

The condensed QP (4·N = 60 variables, linearized safe polytope + Minkowski tightening) remained feasible for every planning step. This validates the tightening parameter choice: ZETA = 0.003 m gives a cumulative tightening of N·ZETA = 0.045 m, which keeps the narrowest corridor (gap = 0.118 m between obs1 and obs2) open at the planning horizon with 0.028 m to spare.

### 3. TTS activated 76.5% — geometrically correct behavior

The Target-Tracking Switching (TTS) mechanism fires when the direct path to the goal is blocked by the linearized obstacle polytope. The obstacle cluster lies directly between start and goal on the diagonal; TTS is expected to stay active for the majority of the trajectory. The 76.5% rate confirms TTS is correctly identifying the blocked-path condition and providing an alternate waypoint via ray-casting.

---

## Reproducibility Findings

The paper (Saccani 2023) leaves four implementation details underspecified. Each required discovery through debugging and geometric analysis.

### Finding 1 — TTS must use the untightened polytope for its feasibility check

**What the paper says:** TTS fires "when the goal lies outside the safe polytope."  
**Ambiguity:** The MPC uses a Minkowski-tightened polytope (`b_obs_N = b_obs − N·ZETA`). It is unspecified which version to use for the TTS check.  
**Bug:** Using the tightened polytope causes TTS to fire 100% of the time even when no real obstacle occludes the goal, because the extra tightening excludes the goal position even from an open corridor.  
**Fix:** Check against the current (untightened) polytope `b_obs`:

```python
# TTS fires when goal violates the CURRENT (untightened) obstacle polytope
if n_obs > 0 and np.any(A_obs @ p_target > b_obs + 1e-6):
    p_eff, tts_activated = self._shift_target_ray(p0, p_target, A_obs, b_obs_N)
```

### Finding 2 — Ray-casting projection is essential; minimum-distance projection causes limit cycles

**What the paper says:** TTS "projects the goal onto the polytope boundary."  
**Ambiguity:** "Projects" is not defined — nearest-point (minimum distance) projection or farthest-reachable-point (ray-casting) are both valid interpretations.  
**Bug:** Nearest-point projection maps the goal to the closest feasible point, which during an obstacle traversal is often only ~0.1 m ahead of the drone or even behind it. This creates a limit cycle: the drone reaches p_eff, TTS recomputes a new p_eff nearby, and the drone circles indefinitely.  
**Fix:** Ray-cast from the current position toward the goal; find the farthest reachable point along that ray within the tightened polytope:

```python
def _shift_target_ray(self, pos_xy, goal_xy, A_obs, b_obs_N):
    direction = (goal_xy - pos_xy) / np.linalg.norm(goal_xy - pos_xy)
    lambda_max = self._ray_clip(pos_xy, direction, A_obs, b_obs_N)
    # lateral ±45° swing if primary ray blocked within 0.3 m
    if lambda_max < 0.3:
        perp = np.array([-direction[1], direction[0]])
        for side in [+1.0, -1.0]:
            lat_dir = normalize(0.7 * direction + 0.7 * side * perp)
            lam_lat = self._ray_clip(pos_xy, lat_dir, A_obs, b_obs_N)
            if lam_lat > lambda_max:
                lambda_max, direction = lam_lat, lat_dir
    reach = MPC_N * V_MAX * DT_MPC  # = 1.5 m
    lambda_use = min(lambda_max * 0.95, reach, d_norm)
    return pos_xy + lambda_use * direction, (lambda_use < d_norm - 1e-6)
```

### Finding 3 — ZETA must satisfy N·ZETA < corridor\_gap/2 (geometric constraint)

**What the paper says:** ZETA is a "robustness margin" for Minkowski tightening; the paper uses ZETA = 0.02 m.  
**Geometric constraint:** The obstacle layout has minimum inter-center distance sqrt(1.25) ≈ 1.118 m with obstacle radius 0.5 m, giving a physical corridor gap of 0.118 m. The tightened corridor gap at planning horizon N is:

```
gap_N = inter_center_dist − 2 * (r_obs + N * ZETA)
      = 1.118 − 2 * (0.500 + 15 * ZETA)
```

For gap_N > 0 (corridor open): `ZETA < 0.118 / (2 * 15) ≈ 0.00393 m`.  
**Bug:** With ZETA = 0.02 (paper value), gap_N = −0.48 m — the corridor is completely closed in the tightened polytope. The MPC sees no feasible path through the cluster and the drone stalls.  
**Fix:** Use ZETA = 0.003 m, giving gap_N = 0.028 m (open). The general constraint for any layout is:

```
ZETA < min_corridor_gap / (2 * N)
```

### Finding 4 — REACH cap needed on the lateral swing to prevent p\_eff overshoot

**What the paper says:** No mention of bounding the projected target distance.  
**Bug:** When the lateral swing direction has no active tightened constraints (the ray escapes to infinity, `_ray_clip` returns 1e6), the uncapped formula `lambda_use = min(1e6 * 0.95, d_norm)` sets `lambda_use = d_norm` (distance to goal, ~4 m). With a lateral direction, `p_eff = pos + 4 m * lat_dir` is a point ~4 m away in a sideways direction. The MPC chases this wrong waypoint, causing the drone to stall in a different region.  
**Fix:** Cap at the planning horizon's maximum reachable distance:

```python
reach = MPC_N * V_MAX * DT_MPC   # = 15 * 1.0 * 0.1 = 1.5 m
lambda_use = min(lambda_max * 0.95, reach, d_norm)
```

This ensures p_eff is always within the MPC's effective planning range regardless of ray direction.

---

## Implementation Parameters

```python
# MPC formulation
MPC_N         = 15      # planning horizon steps
DT_MPC        = 0.10    # s per horizon step
V_MAX         = 1.0     # m/s velocity limit
SAFETY_MARGIN = 0.20    # m added to obstacle radius for safe circle
ZETA          = 0.003   # m Minkowski tightening per step (N*ZETA = 0.045 m)

# Cost weights
LAMBDA_U      = 0.10    # control effort weight
LAMBDA_TERM   = 6.0     # terminal state weight

# Trajectory controller bridge
LOOKAHEAD     = 0.10    # m — target offset passed to DSLPIDControl
```

**Crash tolerance:** 0.05 m clearance (drone radius ≈ 0.05 m). All three controllers maintain clearance above this threshold.

---

## Video Files

| Controller | Video | Steps |
|---|---|---|
| APF | `experiments/results/apf_3d.mp4` | 1246 |
| CBF-QP | `experiments/results/cbf_qp_3d.mp4` | 1074 |
| mt-MPC | `experiments/results/mtmpc_3d.mp4` | 652 |
