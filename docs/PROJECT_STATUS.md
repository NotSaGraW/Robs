# Project Status

## Architecture

### Stack
- Motor control: Python via ZMQ Remote API (`coppeliasim-zmqremoteapi-client`)
- Sensor reading: Python via ZMQ Remote API
- Pioneer Lua scripts: `sysCall_init` active (detection collection), `sysCall_actuation` commented out
- Physics engine: Bullet V2.78
- Control loop: `sim.setStepping(True)` + `sim.step()` — synchronous

### Cooperative force requirement

The core design requirement for cooperative push is that both robots' forces compose
to produce the desired payload motion — their forces must sum to F_des:

```
f0 * n0 + f1 * n1 = F_des

  F_des    = normalize(payload → rally)   unit vector toward goal
  n0, n1   = push directions per robot    (approach → payload centre, unit)
  f0, f1 ≥ 0                              non-negative: robots push, never pull
```

Solved each planning call as a 2-variable NNLS (non-negative least squares):

```
min ||f0*n0 + f1*n1 - F_des||²   s.t. f0,f1 ≥ 0

Closed-form: 4 candidate points (unconstrained interior + 3 boundary cases).
No library dependency. Exact solution.
```

Why NNLS:
- Contact faces provide fixed normal vectors from approach direction.
- Force scalars must be non-negative (robots push, not pull).
- NNLS finds the minimum-residual decomposition under this constraint.
- When the two normals span F_des perfectly (e.g. NE+SE for NE rally), residual=0.
- When no pair reconstructs F_des exactly, NNLS minimises the error.

Invariants agreed during design:
- **Simultaneity**: both robots push at the same time (no alternating turns).
- **Assignment lock**: face assignments are sticky until force_scale degrades or stall
  reset — prevents oscillation between equivalent configurations.
- **Separation**: MIN_APPROACH_SEP=0.52 m ensures robots do not collide at approach.
- **Symmetry**: both orderings (R1→k0,R2→k1) and (R1→k1,R2→k0) are scored —
  no arbitrary swap heuristic.
- **Non-negativity**: NNLS constraint ensures forces are physically realizable.

### Planner: force decomposition (current)

```
ContactPlanner (src/planner.py)
  - 8-direction contact frames: N, S, E, W, NE, NW, SE, SW
  - Approach positions at PUSH_DIST=0.489 m from payload centre
  - NAV_CLEARANCE=0.45 m (Minkowski sum: PAYLOAD_HALF + ROBOT_HALF_W + margin)
  - NNLS (2-variable, closed-form): min ||f0*n0 + f1*n1 - F_des||²  s.t. f0,f1≥0
  - Score function: residual + progress + alignment + balance + torque + nav_cost - lock
  - Filters: MIN_APPROACH_SEP=0.52 m, approach-on-rally-side guardrail
  - Sticky assignment (locks until force_scale→0 or reset)

RobotAgent (src/strategy.py — _RobotAgent)
  - States: NAVIGATE → PUSH → BACKOFF
  - NAVIGATE: follows safe waypoints then drives to approach position.
    Approach+waypoints are frozen on assignment update while in NAVIGATE
    (without freeze, every replan shifts approach with moving payload,
    causing the robot to chase a moving target indefinitely).
    NAV_TIMEOUT=400 steps — fires reset() if approach not reached in time.
    reset() does NOT call robot.stop() — stop() causes a CoppeliaSim physics
    freeze that collapses velocity from 2.0 m/s to 0.007 m/s for the remainder
    of the run. After reset(), the next drive_to() redirects without freeze.
  - PUSH: applies push_dir with sensor centering correction (d3/d4).
    BACKOFF gate: _contact_stall_ctr only increments when in_contact AND
    both_pushing — solo-push against a stationary heavy payload is not stuck.
    Speed adaptation: speed_scale = clamp(0.70, SPEED_BOOST=1.30,
    required_rate / ema_progress) when both pushing and progress measurable.
  - BACKOFF: reverses at 50% APPROACH_SPD for BACKOFF_STEPS=25 steps when
    CONTACT_STALL_THR=30 consecutive stall ticks reached; then returns to NAVIGATE.
  - Both robots receive own d3/d4 observations (unified sensor routing).

Stall detection: EMA of per-step progress, reset after STALL_STEPS consecutive
steps below MIN_PROGRESS. Triggers full re-assignment with keep_locked=True to
preserve sticky assignments and prevent plan_margin collapse.
Conditional replan: _plan_viable() fires mid-period replan when current
ema_progress rate is insufficient to reach rally within remaining step budget.
```

### NAV_CLEARANCE derivation

```
NAV_CLEARANCE = PAYLOAD_HALF + ROBOT_HALF_W + margin
              = 0.25 + 0.19 + 0.01
              = 0.45 m   (Minkowski sum)

Pioneer P3DX body half-width: 0.381m / 2 ≈ 0.19 m (rounded up)
Payload: 0.5m cube, PAYLOAD_HALF = 0.25 m
margin: 0.01 m (minimum geometric buffer)

Why: any path whose closest point to payload centre is < 0.45 m will result
in physical robot-payload contact during navigation. The original value of
0.35 m was insufficient — discovered when R2 (spawning east of payload at
post-exploration position (0.62, 0.61)) routed directly to SE approach
(-0.346, 0.346) with a clearance of 0.402 m — above the old threshold
(no bypass generated) but below physical requirement (collision).
```

### Sensor map (Pioneer P3DX, verified ✓)

| Index | Direction | Primary use |
|-------|-----------|-------------|
| [3] | ~10° left (near frontal) | Payload contact detection |
| [4] | ~10° right (near frontal) | Payload contact detection |

d3/d4 used for centering correction in PUSH: if both < DETECT_DIST (0.80 m),
applies lateral correction `K_CENTER*(d4-d3)*perp` to push direction.
Both sensors read `inf` during NAVIGATE (robot faces waypoint, not payload) —
this is expected and does not affect control; NAVIGATE→PUSH transition is
distance-based, not sensor-based.

### Handle map (verified ✓)

| Object | Handle |
|--------|--------|
| `/p3dx_1` | 15 |
| `/p3dx_2` | 99 |
| `/payload` | 98 |
| `/Floor` | 13 |
| `/rally_point` | 97 |

### Scene properties

| Object | Mass | Respondable | Dynamic |
|--------|------|-------------|---------|
| /payload | 20 kg | YES | YES |
| /Floor | 500 kg | NO | NO |
| /p3dx_1 | ~9 kg | YES | YES |
| /p3dx_2 | ~9 kg | YES | YES |

Friction: Floor frictionOld=1.0, Payload frictionOld=1.0.

---

## Test architecture

Two simulation tests serve complementary diagnostic purposes. Each has a single
measurable objective and produces structured data to support it. Neither is
a general-purpose runner.

### Design principle

Tests must be single-purpose: one hypothesis per file. All data columns serve
that hypothesis. Adding columns "just in case" is acceptable (data is cheap);
mixing two hypotheses into one runner is not. When a test fails, the CSV data
must make the cause identifiable without re-running in CoppeliaSim.

### `test_planner_benchmark.py` — Route and sequence correctness

**Question:** Can the system move the payload to each rally direction within the step budget?

**Purpose:** Acceptance test across the 5 canonical rally directions. Detects
regression in planner assignment, navigation routing, and step budget adequacy.
Does NOT diagnose why a scenario fails — that diagnosis is done by reading the
CSV and cross-referencing with diag data if needed.

**Start conditions:** Canonical western positions (R1_START, R2_START), payload at origin.

**Pass criterion:** `d_rally < SUCCESS_DIST` within `MAX_STEPS=4000`.

**Output files:**
- `logs/benchmark/benchmark_YYYYMMDD_HHMMSS.log` — full stdout including summary table
- `logs/benchmark/benchmark_<scenario>_YYYYMMDD_HHMMSS.csv` — per-step causal data

### `test_planner_diag.py` — Cooperative force quality

**Question:** Is the force applied by both robots well-directed, balanced, and free of torque?

**Purpose:** Causal analysis of the PUSH phase. Measures whether the NNLS
decomposition produces efficient, torque-minimizing cooperative force. Also
validates NAV routing from non-canonical (post-exploration east-start) positions.

**Start conditions:** Canonical or east-start positions (configurable per SCENARIOS entry).

**Pass criterion:** None — output is a rich CSV for offline analysis.

**Output files:**
- `logs/diag/diag_YYYYMMDD_HHMMSS.log` — stdout with STALL/SUCCESS/TIMEOUT events
- `logs/diag/diag_<scenario>_YYYYMMDD_HHMMSS.csv` — per-step force and torque data

### Distinction between the two tests

| Dimension | benchmark | diag |
|-----------|-----------|------|
| Question  | Does it reach the goal? | Is the cooperative force correct? |
| Focus     | Navigation, routing, step budget | Torque, F_along_goal, contact proxy |
| Key data  | robot positions, yaw, approach target, heading error | tau_net, F_lateral, force_scale |
| Scenarios | 5 canonical rally directions | 3 canonical + 2 east-start |
| Pass/fail | YES (d_rally < threshold) | NO (analysis-only) |

---

## Data schema

### Common to both CSVs

| Column | Type | Description |
|--------|------|-------------|
| `step` | int | Simulation step index (0-based) |
| `scenario` | str | Scenario name (rally direction identifier) |
| `px`, `py` | float | Payload position in world frame (m) |
| `pvx`, `pvy` | float | Payload linear velocity (m/step) |
| `pwz` | float | Payload angular velocity about Z (rad/step) — non-zero indicates torque imbalance |
| `payload_angle` | float | Cumulative payload rotation about Z from initial orientation (rad) — grows if payload spins |
| `d_rally` | float | Distance payload→rally (m) — primary progress metric |
| `r1_phase`, `r2_phase` | str | NAVIGATE / PUSH / IDLE |
| `r1_face`, `r2_face` | str | Assigned contact face (N/S/E/W/NE/NW/SE/SW or -) |
| `r1_fs`, `r2_fs` | float | Force scale from NNLS — how much of unit force this robot contributes |
| `r1_x`, `r1_y` | float | Robot 1 position in world frame (m) |
| `r2_x`, `r2_y` | float | Robot 2 position in world frame (m) |
| `r1_yaw`, `r2_yaw` | float | Robot heading angle (rad, world frame) |
| `r1_fx`, `r1_fy` | float | Effective push direction (unit vector, after centering correction) |
| `r2_fx`, `r2_fy` | float | — |
| `r1_heading_err` | float | Angle between robot yaw and push direction (rad) — 0 = perfectly aligned |
| `r2_heading_err` | float | — |
| `r1_app_x`, `r1_app_y` | float | Current approach position target for R1 (m) |
| `r2_app_x`, `r2_app_y` | float | — |
| `r1_dist_to_app` | float | Distance R1→approach target (m) — shows navigation convergence |
| `r2_dist_to_app` | float | — |
| `r1_wp_idx`, `r2_wp_idx` | int | Current waypoint index being tracked |
| `r1_wp_total`, `r2_wp_total` | int | Total waypoints in current route (0 = direct path) |
| `r1_nav_steps`, `r2_nav_steps` | int | Steps spent in NAVIGATE for current assignment |
| `r1_vx`, `r1_vy` | float | Actual robot 1 velocity (m/step, from physics engine) |
| `r2_vx`, `r2_vy` | float | — |
| `d3_r1`, `d4_r1` | float | Proximity sensors 3 and 4 for R1 (m or 'inf') |
| `d3_r2`, `d4_r2` | float | — |
| `r1_contact`, `r2_contact` | int | 1 if both d3/d4 < DETECT_DIST (proxy for physical contact) |
| `face_change_r1`, `face_change_r2` | int | 1 if face assignment changed this step — marks instability events |
| `both_pushing` | int | 1 if both robots in PUSH phase simultaneously |
| `Fx_net`, `Fy_net` | float | Net force vector (force_scale-weighted sum of push directions) |
| `F_along_goal` | float | Component of net force along F_des direction — efficiency measure |
| `F_lateral` | float | Component of net force perpendicular to F_des — waste/drift measure |
| `ema_progress` | float | EMA of payload progress (active only when both_pushing) |
| `stall_counter` | int | Steps since last positive EMA progress |

### Benchmark-only columns

| Column | Description |
|--------|-------------|
| `delta_d_rally` | Change in d_rally vs previous step — instantaneous progress rate |

### Diag-only columns

| Column | Description |
|--------|-------------|
| `progress_fixed` | Payload velocity projected onto FIXED F_des (set at step 0) |
| `lateral_fixed` | Payload velocity perpendicular to FIXED F_des |
| `progress_dyn` | Payload velocity projected onto DYNAMIC F_des (current payload→rally) |
| `lateral_dyn` | Payload velocity perpendicular to DYNAMIC F_des |
| `tau1`, `tau2` | Torque proxy per robot: cross(arm, push_dir) × force_scale |
| `tau_net` | Sum of tau1+tau2 — net torque proxy; non-zero causes payload rotation |

### Interpreting key columns

**Diagnosing navigation failure:**
- `r_nav_steps` climbing toward NAV_TIMEOUT with `r_dist_to_app` not decreasing → stuck
- `r_wp_total > 0` → bypass waypoint was generated (NAV_CLEARANCE blocked direct path)
- `face_change_r` = 1 repeatedly → planner oscillating between assignments

**Diagnosing push misalignment:**
- `r_heading_err` large (> 0.3 rad) in PUSH → robot not aligned with push direction
- `F_lateral` large relative to `F_along_goal` → force wasted on drift
- `tau_net` consistently positive or negative → payload spinning in one direction

**Diagnosing cooperative breakdown:**
- `both_pushing` = 0 for many steps → one robot in NAVIGATE while other pushes alone
- Solo push with F_des≠push_dir → payload drifts laterally during that interval
- `r_fs` → 0 for one robot → NNLS found one-robot-dominant solution

**Diagnosing payload rotation:**
- `pwz` sustained non-zero → torque being applied each step
- `payload_angle` drifting → cumulative rotation — face normals now misaligned with world

---

## Benchmark results (2026-05-26)

### Run parameters
- Start: R1=[-1.725,-1.475,0.195], R2=[-1.750,1.850,0.195], Payload=[0,0,0.250]
- MAX_STEPS=4000, PUSH_SPD=APPROACH_SPD=2.0
- NAV_TIMEOUT=400, frozen approach, BACKOFF, keep_locked=True, speed adaptation

| Scenario      | Result    | Steps | Stalls |
|---------------|-----------|-------|--------|
| NE (baseline) | SUCCESS ✓ | 1327  | 0      |
| E (east)      | SUCCESS ✓ | 2420  | 0      |
| N (north)     | SUCCESS ✓ | 2928  | 0      |
| NW (northwest)| SUCCESS ✓ | 2503  | 0      |
| SE (southeast)| SUCCESS ✓ | 1699  | 0      |

### Fixes applied

**Frozen approach (N north fix)**
Root cause: `update_assignment()` always reset `wp_idx=0` and overwrote the approach
with the current payload position. Every REPLAN_EVERY=10 steps, the approach shifted
with the moving payload — R2 was always chasing a target that had just moved away.
Fix: in NAVIGATE phase, freeze `approach` and `waypoints` from the previous assignment.
Only face_changed events or PUSH/BACKOFF transitions allow the approach to update.
Verified with CSV data: r2_app_x/y constant from step 1098 to 1199 across 10+ replans
while payload moved from py=0.0842 to py=0.0864 (N_north run).

**NAV_TIMEOUT=400 + no robot.stop() (physics freeze fix)**
Root cause: in diag N_north, R2 hit NAV_TIMEOUT with approach still 0.36m away.
`robot.stop()` in `reset()` caused CoppeliaSim to freeze the robot in a near-static
physics state: velocity dropped from 0.038 m/step to 0.007 m/step, closure rate from
0.038 to 0.0003 m/step. Repeated timeouts with no convergence.
Fix: NAV_TIMEOUT triggers reset() without calling robot.stop(). Robot continues at
current velocity; next drive_to() redirects without triggering the freeze artifact.

**BACKOFF state (NW, SE deadlock fix)**
Root cause: static friction deadlock — both robots physically stuck against payload,
EMA progress → 0, stall reset fires but keep_locked=False caused plan_margin collapse
and 70-100 face changes per 100 steps post-reset.
Fix: BACKOFF gate counts stall only when `in_contact AND both_pushing` (solo-push
stall against heavy payload is expected, not stuck). After CONTACT_STALL_THR=30 ticks,
robot backs away for BACKOFF_STEPS=25 steps then re-approaches.
keep_locked=True on stall reset preserves sticky assignments, preventing oscillation.

**Speed adaptation**
When both_pushing and ema_progress measurable: speed_scale = clamp(0.70, 1.30,
required_rate / ema_progress). Prevents timeout when closing a long rally at
insufficient natural push speed. Verified: speed_scale ranged 0.70–1.128 in N_north run.

---

## Verified results

### Single robot push — reference baseline
**File:** `simulation_tests/test_single_push.py`

- 826 steps, 0 repositions, final dist=0.3293 m
- Approach: GT navigation to push_target position
- Push: continuous direction = GT vector + sensor centering `K_CENTER*(d4-d3)*perp`
- Deterministic and reproducible

### Two-robot cooperative push — evolution

| Version | Approach | Steps | Stall resets | f ratio |
|---------|----------|-------|-------------|---------|
| Cardinal N+E | Fixed normals, greedy face assign | 4987 | 0 | 0.33 |
| Cardinal N+E + NNLS | Force decomposition, lock | 4987 | 0 | 0.33 |
| 8-dir NE+SE | 8-direction frames, sep filter | **1466** | 0 | **0.50→0.17** |

**Current best:** `simulation_tests/test_planner.py` — 1466 steps, no stall resets.

Assignment for rally at (1.125, 0.225) / F_des ≈ (0.949, 0.316):
- R1 → NE face, app=(-0.346, -0.346), push=(+0.707,+0.707), f≈0.83–0.99
- R2 → SE face, app=(-0.346, +0.346), push=(+0.707,−0.707), f≈0.55→0.17

NE+SE chosen because: approach separation=0.692 m (safe), residual=0 (perfect
reconstruction of F_des), balance=0.50 at t=0.

---

## Planner design decisions

| Decision | Rationale |
|----------|-----------|
| 8-direction frames over 4 | Cardinal-only collapses to N+E for diagonal rally; ratio=0.33 |
| MIN_APPROACH_SEP=0.52 m | Pioneer width=0.415 m; adjacent 8-dir approaches=0.374 m → collision |
| Approach-on-rally-side guardrail | Eliminates S-survival bias: S approach has positive dot with NE F_des |
| NNLS closed-form, 2 variables | No library dependency; 4 candidate cases, exact solution |
| Both orderings scored (no swap heuristic) | Eliminates assignment oscillation |
| Soft alignment penalty | Allows NNLS to correct poor alignment without hard pruning |
| Balance penalty (abs(f0-f1)) | Discourages one-robot-dominant solutions |
| Torque penalty (cross product) | Penalises rotation-inducing force configurations |
| Sticky assignment | Robot stays on assigned face until score degrades → avoids navigation thrash |
| wp_idx always reset on replan | Waypoints shift with payload; stale index causes wrong navigation |
| Unified sensor routing | Both agents call read_sensors(sim, robot) independently → symmetric observation |
| NAV_CLEARANCE=0.45 (Minkowski) | PAYLOAD_HALF+ROBOT_HALF_W+margin=0.25+0.19+0.01; old 0.35 caused collision from east-start positions |
| Frozen approach during NAVIGATE | update_assignment() preserves approach+waypoints when face unchanged and phase=NAVIGATE; without this, every replan shifts approach with payload and robot chases moving target |
| No robot.stop() in reset() | stop() causes CoppeliaSim physics freeze: velocity drops from 2.0 to 0.007 m/s, closure rate from 0.038 to 0.0003 m/step. Next drive_to() redirects without freeze if stop() is omitted |
| BACKOFF gate: in_contact AND both_pushing | Solo robot pressing against heavy payload naturally stays still — that is not stuck. Gate prevents false BACKOFF during solo-push intervals |
| keep_locked=True on stall reset | Preserves sticky face assignments after reset; without this, all locks clear and NNLS restarts from scratch → plan_margin collapses → 70-100 face changes per 100 steps |
| Speed adaptation: clamp(0.70, 1.30, required/actual) | Prevents timeout when natural push rate is insufficient for remaining step budget; floor 0.70 avoids reducing speed below stable contact threshold |

---

## Known limitations

### Structural (requires redesign)
1. **Basis degeneracy**: fixed 8-direction frames give good NNLS balance at t=0
   but degrade as payload moves (SE contribution falls from 0.55→0.17 for NE rally).
   Fix: rally-frame dynamic basis — compute approach at ±45° from F_des each tick.

2. **Discrete action space**: with only 8 directions, there are always scenarios
   where no pair gives ratio > 0.5. Continuous approach angles would eliminate this.

3. **No environment exploration**: payload position obtained via `getObjectPosition`
   (ground truth). The exploration/detection phase from `src/main.py` drives toward
   rally_point until a proximity sensor detects the payload, then switches to
   ContactPlanner. True exploration (unknown environment) is not implemented.

### Operational (tunable / in progress)
4. ~~**Benchmark passes only 1/5 scenarios**~~ — RESOLVED: all 5 scenarios pass
   (see Benchmark results above). Frozen approach, NAV_TIMEOUT, BACKOFF, keep_locked,
   and speed adaptation collectively eliminated all five failure modes.

5. ~~**Stall detection inactive during NAVIGATE**~~ — RESOLVED: NAV_TIMEOUT=400
   added to benchmark and strategy.py. Fires reset() without robot.stop() to avoid
   CoppeliaSim physics freeze artifact.

6. **Waypoint generator uses column routes only**: simple 2-waypoint column route.
   Does not handle scenarios where column route is also blocked (moved payload).

7. **Face reassignment instability near goal**: as d_rally decreases, small
   perturbations in payload position change the NNLS score ordering, causing
   repeated face reassignments. Each reassignment sends a robot to NAVIGATE,
   potentially creating a deadlock at close range.

---

## Simulation tests status

| Script | Status | Result |
|--------|--------|--------|
| `test_sensor_360.py` | PASS ✓ | Position formula verified, avg error 0.025 m |
| `test_handle_mapping.py` | PASS ✓ | All object handles confirmed |
| `test_single_push.py` | PASS ✓ | 826 steps, 0.3293 m final dist |
| `test_planner.py` | PASS ✓ | 1466 steps, NE+SE, 0 stall resets |
| `test_planner_benchmark.py` | PASS ✓ | 5/5 scenarios pass; see benchmark results above |
| `test_planner_diag.py` | IN USE | CSV data for force/torque/nav analysis |
| `test_two_robots.py` | SUPERSEDED | Replaced by test_planner.py |
| `test_exploring.py` | PASS (partial) | Outer waypoints OK, inner oscillate |
| `test_wall_follower.py` | PASS (partial) | PID OK, requires wall in range at start |
| `test_wall_approach.py` | PENDING | |
| `test_push_alignment.py` | PENDING | |

---

## Pending work (priority order)

1. **Face reassignment stability near goal** — add hysteresis or assignment lock
   once d_rally < threshold to prevent oscillation at close range.

2. **Rally-frame dynamic basis** — compute ±45° approach from F_des each tick
   to eliminate basis degeneracy and keep NNLS balance high throughout the push.

3. **Repeatability tests** — 5+ runs with same parameters to verify determinism.

4. ~~**Add NAV_TIMEOUT to benchmark and strategy.py**~~ — DONE: NAV_TIMEOUT=400 in
   all three (diag, benchmark, strategy.py); reset() omits robot.stop().

5. ~~**Extend stall detection to NAVIGATE phase**~~ — DONE: NAV_TIMEOUT covers this.

6. ~~**Run test_planner_diag.py east-start scenarios**~~ — DONE: NE_east_start SUCCESS
   2223 steps, SE_east_start SUCCESS 2618 steps; NAV_CLEARANCE=0.45 validated.

7. ~~**Integration with exploration pipeline**~~ — DONE: ContactPlanner integrated into
   `src/strategy.py` ACTIVE phase; `_RobotAgent` (NAVIGATE→PUSH→BACKOFF) replaces
   fixed-role logic.

8. ~~**Multi-scenario benchmarking**~~ — DONE: 5/5 benchmark scenarios pass.
