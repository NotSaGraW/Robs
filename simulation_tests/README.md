# Simulation Tests

Verification scripts that require CoppeliaSim running with the scene loaded.
Not automated unit tests — require simulated hardware.

## How to run

```powershell
# From project root, with CoppeliaSim open and scene loaded
python -m simulation_tests.test_planner
python -m simulation_tests.test_single_push
```

## Status

### Completed

| Script | Date | Result |
|--------|------|--------|
| `test_sensor_360.py` | 2026-05 | PASS — position formula verified, avg error 0.025 m |
| `test_handle_mapping.py` | 2026-05 | PASS — all object handles confirmed |
| `test_single_push.py` (v7) | 2026-05 | PASS — 826 steps, 0.3293 m, deterministic |
| `test_planner.py` (v3) | 2026-05 | PASS — 1466 steps, NE+SE faces, 0 stall resets |
| `test_exploring.py` | 2026-05 | Partial — outer waypoints OK, inner waypoints oscillate |
| `test_wall_follower.py` | 2026-05 | Partial — PID OK, requires wall in range at start |

### Active development

| Script | Purpose |
|--------|---------|
| `test_planner.py` | Two-robot cooperative push using ContactPlanner |

### Pending

| Script | Purpose |
|--------|---------|
| `test_wall_approach.py` | Robot advances until payload detected from open space |
| `test_push_alignment.py` | Optimal approach geometry for two robots |

### Superseded

| Script | Replaced by | Notes |
|--------|-------------|-------|
| `test_two_robots.py` | `test_planner.py` | FSM approach, no force decomposition |

## Key results summary

### Single robot (v7) — `test_single_push.py`
Best reference for single-agent performance.
- **826 steps**, 0 repositions, final dist 0.3293 m
- Deterministic across multiple runs from same start position (-1.725, -1.475)
- Architecture: GT navigation to push_target + sensor centering in PUSH phase

### Two robots — `test_planner.py` evolution

```
Version        Steps   Ratio   Notes
N+E cardinal   4987    0.33    First cooperative success; R1 dominates
NE+SE 8-dir    1466    0.50    Current best; 3.4× faster than N+E
```

**NE+SE assignment** for rally at (1.125, 0.225):
- R1 → NE: approach (-0.346, -0.346), push (+0.707, +0.707), f≈0.83
- R2 → SE: approach (-0.346, +0.346), push (+0.707, -0.707), f≈0.55
- Approach separation: 0.692 m (no collision)
- Force residual: 0 (perfect F_des reconstruction)

## Architecture note

`test_planner.py` v3 uses:
- `src/planner.py` — ContactPlanner with 8-direction frames, NNLS, sticky locks
- `src/robot.py` — Robot class (sensors, motors, drive_to)
- Unified sensor routing: each robot reads its own d3/d4 independently
- EMA stall detection (active only when both robots in PUSH)