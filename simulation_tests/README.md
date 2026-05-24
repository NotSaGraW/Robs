# Multi-Robot Cooperative System — CoppeliaSim + Python

## Overview
Multi-agent cooperative robotics system using two Pioneer P3DX robots and a
quadcopter to move a payload to a rally point in a completely unknown environment.
The only prior knowledge is the rally point position.

## Agents
| Agent     | Type         | Role |
|-----------|--------------|------|
| `/p3dx_1` | Pioneer P3DX | Exploration, pusher 1 |
| `/p3dx_2` | Pioneer P3DX | Exploration, pusher 2 |
| `/drone`  | Quadcopter   | Aerial exploration, payload detection, corridor surveillance, communication hub |

## Scene objects
| Object          | Description |
|-----------------|-------------|
| `/payload`      | Object to transport (20kg, 0.5m cube) |
| `/rally_point`  | Extraction point — only prior known position |

## Setup
1. Create and activate a virtual environment:
```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
```
2. Install dependencies:
```powershell
python -m pip install -U pip
python -m pip install -e .
pip install coppeliasim-zmqremoteapi-client numpy Pillow
```
3. Run unit tests:
```powershell
python -m pytest
```
4. Run full system (CoppeliaSim must be open with scene loaded):
```powershell
python -m src.main
```

## Scene geometry
- Playfield: 5×5m, usable area ~±2.4m in x and y (external walls excluded)
- `/p3dx_1` start: (-1.75, -1.425), `/p3dx_2` start: (-1.75, 1.85)
- `/payload` start: (0, 0) — unknown to agents until detected
- `/rally_point`: (1.5, 0.5) — known from mission start
- Required push vector: (1.5, 0.5) normalised ≈ (0.949, 0.316)

## Knowledge model
```
KNOWN at t=0 (given by mission):
  - rally_point position  ← sole absolute anchor

UNKNOWN, must be discovered:
  - environment dimensions
  - wall positions
  - payload position
  - other robot positions (until drone communication)
  - obstacles

NEVER ASSUMED:
  - any position not yet confirmed by sensor
```

## Project structure
```
src/              → main system code
  main.py         → main loop, state machine, timing, logs
  robot.py        → Robot class (handle, motors, sensors, navigation)
  drone.py        → Drone class (patrol, payload detection, corridor mapping)
  scene.py        → geometry, vectors, success condition
  strategy.py     → multi-agent coordination, phase machine
  grid.py         → shared 2D occupancy map
tests/            → automated unit tests (pytest, no CoppeliaSim required)
experiments/      → experimental verification scripts (CoppeliaSim required)
  logs/           → structured output from each experiment
  results/        → analysis and conclusions from experiments
docs/             → technical documentation and design decisions
  PROJECT_STATUS.md → design decisions, verified data, known issues
```

## System phases
| Phase      | Description | Exit condition |
|------------|-------------|----------------|
| EXPLORING  | Wall following builds shared occupancy map | Payload detected by any agent |
| CONVERGING | Agents navigate to push positions | Both confirmed by frontal sensors |
| PUSHING    | Both robots push with sensor-confirmed contact | Payload reaches rally point |
| SUCCESS    | Mission complete | — |

## Communication architecture
- **Regular channel — 2Hz heartbeat:** robots → drone (partial map, position, state) / drone → robots (global map, other robot position)
- **Urgent channel — event-driven:** robot detects unknown object → drone identifies (KNOWN_AGENT / KNOWN_OBJECT / KNOWN_STATIC / UNKNOWN)
- Drone acts as data hub: elevated position avoids interference, has global view

## Current status
See `docs/PROJECT_STATUS.md` for verified hardware data, design decisions,
known issues and pending experiments.