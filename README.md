# Multi-Robot Cooperative System — CoppeliaSim + Python

## Overview
Multi-agent cooperative robotics system using two Pioneer P3DX robots and a quadcopter
to move a box to a target position in an initially unknown environment.

## Agents
| Agent  | Type         | Role |
|--------|--------------|------|
| `/roba`| Pioneer P3DX | Left sector exploration, pusher A |
| `/robo`| Pioneer P3DX | Right sector exploration, pusher B |
| `/qua` | Quadcopter   | Aerial exploration, box detection, corridor surveillance |

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
- `/roba` start: (-1.75, -1.425), `/robo` start: (-1.75, 1.85)
- `/box` start: (0, 0), `/target`: (1.5, 0.5)
- Required push vector: (1.5, 0.5) normalised ≈ (0.949, 0.316)
- Optimal push position: opposite side of box from target → direction (-0.949, -0.316)

## Project structure
```
src/              → main system code
  main.py         → main loop, state machine, timing, logs
  robot.py        → Robot class (handle, motors, sensors, navigation)
  drone.py        → Drone class (patrol, box detection, corridor mapping)
  scene.py        → geometry, vectors, success condition
  strategy.py     → multi-agent coordination, phase machine
  grid.py         → shared 2D occupancy map
tests/            → automated unit tests (pytest, no CoppeliaSim required)
experiments/      → experimental verification scripts (CoppeliaSim required)
  logs/           → structured output from each experiment
  results/        → analysis and conclusions from experiments
docs/             → technical documentation and design decisions
  project_status.md → design decisions, verified data, known issues, pending experiments
```

## System phases
| Phase | Description | Exit condition |
|-------|-------------|----------------|
| ORIENTATION | Each robot reads all sensors and identifies free space | 360° scan complete |
| EXPLORING | Wall following builds shared occupancy map | Box detected by any agent |
| DETECTING | All agents stop, box position confirmed | Position verified |
| PLANNING | BFS over map computes safe approach routes | Routes computed for A and B |
| POSITIONING | A positions first (B idle), then B (A idle) | Both confirmed by sensors [3,4] |
| PUSHING | Both robots push with sensor-confirmed contact | Box reaches target |
| SUCCESS | Mission complete | — |

## Current status
See `docs/project_status.md` for verified hardware data, design decisions,
known issues and pending experiments.