# Project Status

## Architecture

### Stack
- Motor control: Python via ZMQ Remote API (`coppeliasim-zmqremoteapi-client`)
- Sensor reading: Python via ZMQ Remote API
- Pioneer Lua scripts: `sysCall_init` active (detection collection), `sysCall_actuation` commented out
- Physics engine: Bullet V2.78
- Control loop: `sim.setStepping(True)` + `sim.step()` — synchronous

### Planner: force decomposition (current)

```
ContactPlanner (src/planner.py)
  - 8-direction contact frames: N, S, E, W, NE, NW, SE, SW
  - Approach positions at PUSH_DIST=0.489 m from payload centre
  - NNLS (2-variable, closed-form): min ||f0*n0 + f1*n1 - F_des||²  s.t. f0,f1≥0
  - Score function: residual + progress + alignment + balance + torque + nav_cost - lock
  - Filters: MIN_APPROACH_SEP=0.52 m, approach-on-rally-side guardrail
  - Sticky assignment (locks until force_scale→0 or reset)

RobotAgent (simulation_tests/test_planner.py)
  - States: NAVIGATE → PUSH
  - NAVIGATE: follows safe waypoints to approach position
  - PUSH: applies push_dir with sensor centering correction (d3/d4)
  - Both robots receive own d3/d4 observations (unified sensor routing)

Stall detection: EMA of per-step progress, reset after STALL_STEPS consecutive
steps below MIN_PROGRESS. Triggers full re-assignment.
```

### Sensor map (Pioneer P3DX, verified ✓)

| Index | Direction | Primary use |
|-------|-----------|-------------|
| [3] | ~10° left (near frontal) | Payload contact detection |
| [4] | ~10° right (near frontal) | Payload contact detection |

d3/d4 used for centering correction in PUSH: if both < DETECT_DIST (0.80 m),
applies lateral correction `K_CENTER*(d4-d3)*perp` to push direction.

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

## Verified results

### Single robot push (v7) — reference baseline
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

**Current best:** `simulation_tests/test_planner.py` v3 — 1466 steps, no stall resets.

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

---

## Known limitations

### Structural (requires redesign)
1. **Basis degeneracy**: fixed 8-direction frames give good NNLS balance at t=0
   but degrade as payload moves (SE contribution falls from 0.55→0.17 for NE rally).
   Fix: rally-frame dynamic basis — compute approach at ±45° from F_des each tick.

2. **Discrete action space**: with only 8 directions, there are always scenarios
   where no pair gives ratio > 0.5. Continuous approach angles would eliminate this.

3. **No integration with exploration**: `src/main.py` pipeline (EXPLORING→CONVERGING→
   PUSHING phases) is not connected to ContactPlanner. The planner uses GT positions
   directly via `getObjectPosition`.

### Operational (tunable)
4. **Single scenario benchmarked**: all results are for rally at (1.125, 0.225).
   Other rally positions (N, S, NW) not tested.

5. **Stall detection inactive during NAVIGATE**: EMA progress only runs when
   `both_pushing=True`. If one robot gets stuck in NAVIGATE, no recovery fires.

6. **Waypoint generator uses column routes only**: simple 2-waypoint column route.
   Does not handle scenarios where column route is also blocked.

---

## Simulation tests status

| Script | Status | Result |
|--------|--------|--------|
| `test_sensor_360.py` | PASS ✓ | Position formula verified, avg error 0.025 m |
| `test_handle_mapping.py` | PASS ✓ | All object handles confirmed |
| `test_single_push.py` (v7) | PASS ✓ | 826 steps, 0.3293 m final dist |
| `test_planner.py` (v3) | PASS ✓ | 1466 steps, NE+SE, 0 stall resets |
| `test_two_robots.py` | SUPERSEDED | Replaced by test_planner.py |
| `test_exploring.py` | PASS (partial) | Outer waypoints OK, inner oscillate |
| `test_wall_follower.py` | PASS (partial) | PID OK, requires wall in range at start |
| `test_wall_approach.py` | PENDING | |
| `test_push_alignment.py` | PENDING | |

---

## Pending work (priority order)

1. **Rally-frame dynamic basis** — compute ±45° approach from F_des each tick
2. **Multi-scenario benchmarking** — rally N, S, NW positions
3. **Stall recovery for NAVIGATE state** — timeout + replan if robot stuck
4. **Integration with exploration pipeline** — connect ContactPlanner to main.py
5. **Repeatability tests** — 5+ runs with same parameters to verify determinism