# Multi-Robot Cooperative Push — CoppeliaSim + Python

## Overview

Two Pioneer P3DX robots cooperate to push a payload to a rally point.
The system uses a centralised **force-decomposition planner** that solves
a 2-variable NNLS problem each tick to assign contact faces and
coordinate push forces.

## Agents

| Agent     | Type         | Role |
|-----------|--------------|------|
| `/p3dx_1` | Pioneer P3DX | Pusher 1 |
| `/p3dx_2` | Pioneer P3DX | Pusher 2 |

## Scene objects

| Object         | Description |
|----------------|-------------|
| `/payload`     | Object to transport (20 kg, 0.5 m cube) |
| `/rally_point` | Extraction point — only position known at mission start |

## Setup

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -U pip
pip install -e .
pip install coppeliasim-zmqremoteapi-client Pillow
```

Run with CoppeliaSim open and scene loaded:

```powershell
python -m simulation_tests.test_planner
```

## Scene geometry

| Object       | Position |
|--------------|----------|
| `/p3dx_1`    | (-1.725, -1.475) |
| `/p3dx_2`    | (-1.750, 1.850) |
| `/payload`   | (0, 0) — unknown to agents |
| `/rally_point` | (1.125, 0.225) |

Playfield: 5×5 m. Verified handles: p3dx_1=15, p3dx_2=99, payload=98, rally=97.

## Project structure

```
src/
  planner.py   — ContactPlanner: NNLS force decomposition, 8-direction frames
  robot.py     — Robot class: sensors, motors, drive_to, get_yaw
  scene.py     — geometry: dist2d, push_vector, payload_reached_rally_point
  grid.py      — shared 2D occupancy map (50×50, 0.1 m/cell)

simulation_tests/
  test_planner.py    — two-robot cooperative push (ACTIVE development)
  test_single_push.py — single-robot baseline (v7, reference)
  test_sensor_360.py  — sensor position formula verification (PASS)
  test_handle_mapping.py — object handle confirmation (PASS)

docs/
  PROJECT_STATUS.md  — architecture decisions, results, pending work
```

## Planner architecture

```
Each tick:
  F_des = normalize(payload → rally)

  For each unordered face pair {k0, k1} from 8 directions (N,S,E,W,NE,NW,SE,SW):
    For both robot assignments (R1→k0,R2→k1) and (R1→k1,R2→k0):
      1. Reject if approach separation < 0.52 m  (robot collision prevention)
      2. Reject if approach is on rally side of payload (wrong direction)
      3. Solve NNLS: min ||f0*n0 + f1*n1 - F_des||²  s.t. f0,f1 ≥ 0
      4. Score = residual + progress_penalty + alignment_penalty
                + balance_penalty + torque_penalty + nav_cost - lock_bonus
  
  Select minimum score → assign faces → compute safe waypoints
  Lock assignment (sticky) until force_scale → 0 or stall reset
```

Push direction per face: dynamic (approach → payload centre), not fixed cardinal.

## Verified results

| Test | Steps | Stall resets | Notes |
|------|-------|-------------|-------|
| Single robot (v7) | 826 | 0 | Baseline reference |
| Two robots, cardinal N+E | 4987 | 0 | First cooperative success |
| Two robots, 8-dir NE+SE | **1466** | 0 | **Current best** |

Force balance NE+SE: f=(0.83, 0.55), ratio=0.50 at t=0 → degrades to 0.17 as
payload approaches rally (geometric: SE contribution decreases along NE trajectory).
No S-survival bias: approach-on-rally-side guardrail filters it.
Both robots now receive their own d3/d4 sensor observations (centering correction active).

## Known limitations

- **Basis degeneracy**: with fixed 8-direction frames, NNLS balance degrades
  as payload moves and angles shift. Rally-frame basis (dynamic ±45° from
  payload→rally vector) would give f0≈f1 throughout. Not yet implemented.
- **Single test scenario**: all results are for rally at (1.125, 0.225).
  Different rally positions not yet benchmarked.
- **No environment exploration**: payload and environment are known via
  `getObjectPosition` GT. The exploration/detection phase from `src/main.py`
  is not integrated with the planner.

## Next steps

1. Rally-frame dynamic basis: compute approach directions as ±45° from F_des
2. Multi-scenario benchmarking: rally N, S, NW
3. Integration with `src/main.py` exploration pipeline