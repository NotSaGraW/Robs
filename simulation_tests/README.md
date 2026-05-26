# Simulation Tests

Verification scripts that require CoppeliaSim running with the scene loaded.
Not automated unit tests — require simulated hardware.

## How to run

```powershell
# From project root, with CoppeliaSim open and scene loaded
python -m simulation_tests.test_planner
python -m simulation_tests.test_planner_diag
python -m simulation_tests.test_planner_benchmark
```

---

## Test descriptions

### `test_planner_benchmark.py` — Multi-scenario acceptance test

**Status:** PASS ✓ 5/5  
**Question:** Can the system move the payload to each rally direction within the step budget?

Runs the full ContactPlanner + _RobotAgent (NAVIGATE→PUSH→BACKOFF) pipeline across 5 canonical rally directions. Both robots start at canonical western positions; the planner assigns faces via NNLS and each robot executes its state machine independently.

- **Pass criterion:** `d_rally < SUCCESS_DIST` within `MAX_STEPS = 4000`
- **Scenarios:** NE (baseline), E, N, NW, SE
- **Output:**
  - `logs/benchmark/benchmark_YYYYMMDD_HHMMSS.log` — full stdout + summary table
  - `logs/benchmark/benchmark_<name>_YYYYMMDD_HHMMSS.csv` — per-step causal data
- **Key parameters:** `PUSH_SPD = APPROACH_SPD = 2.0`, `NAV_TIMEOUT = 400`, `REPLAN_EVERY = 10`

Results (2026-05):

| Scenario | Result    | Steps |
|----------|-----------|-------|
| NE       | SUCCESS ✓ | 1 327 |
| E        | SUCCESS ✓ | 2 420 |
| N        | SUCCESS ✓ | 2 928 |
| NW       | SUCCESS ✓ | 2 503 |
| SE       | SUCCESS ✓ | 1 699 |

---

### `test_planner_diag.py` — Cooperative force quality diagnostic

**Status:** IN USE (analysis-only, no pass/fail)  
**Question:** Is the force applied by both robots well-directed, balanced, and free of torque?

Runs 3 configurable scenarios and produces a rich CSV for offline causal analysis of the PUSH phase. Also validates NAV routing from non-canonical (post-exploration east-start) positions.

- **No pass/fail criterion** — output is for offline analysis with CSV tools
- **Scenarios:** configurable (default: NE_baseline, N_north, SE_southeast)
- **Output:**
  - `logs/diag/diag_YYYYMMDD_HHMMSS.log` — stdout with STALL/SUCCESS/TIMEOUT events
  - `logs/diag/diag_<name>_YYYYMMDD_HHMMSS.csv` — per-step force, torque, nav data
- **Diag-only CSV columns:** `tau1`, `tau2`, `tau_net` (torque proxy), `progress_fixed`, `lateral_fixed`, `progress_dyn`, `lateral_dyn`, `F_along_goal`, `F_lateral`
- **Key parameters:** `MAX_STEPS = 4000`, `NAV_TIMEOUT = 400`, `CONTACT_STALL_THR = 30`, `BACKOFF_STEPS = 25`

---

### `test_planner.py` — Two-robot cooperative push (interactive)

**Status:** PASS ✓  
**Purpose:** Interactive run of ContactPlanner for development and visual inspection in CoppeliaSim.

Same control loop as benchmark but single-scenario, with more verbose step logging. Used during development to observe robot behaviour in real time before committing changes to the benchmark.

- **Start:** R1 = (−1.725, −1.475), R2 = (−1.750, 1.850), rally = (1.125, 0.225)
- **Output:** console only (no CSV)
- **Result:** 1 466 steps, NE+SE faces, 0 stall resets

NE+SE assignment for rally at (1.125, 0.225):
- R1 → NE: approach (−0.346, −0.346), push (+0.707, +0.707), f ≈ 0.83
- R2 → SE: approach (−0.346, +0.346), push (+0.707, −0.707), f ≈ 0.55
- Approach separation: 0.692 m — no collision
- Force residual: 0 (perfect F_des reconstruction)

---

### `test_single_push.py` — Single-robot push (reference baseline)

**Status:** PASS ✓  
**Purpose:** Reference performance for single-agent push; basis of comparison for two-robot gains.

Phases: SCAN (drive_to push_target via GT) → PUSH (continuous direction = GT vector + sensor centering). Repositions on stall or contact loss.

- **Sensors used:** d3/d4 for lateral centering correction only (`K_CENTER * (d4 − d3)`)
- **Navigation:** ground-truth `getObjectPosition` throughout
- **Result:** 826 steps, 0 repositions, final dist 0.3293 m — deterministic across runs

---

### `test_sensor_360.py` — Sensor world-position formula

**Status:** PASS ✓  
**Purpose:** Prerequisite verification — confirms that the formula for converting sensor readings to world positions is correct before implementing communication architecture.

Formula under test: `detected_world_pos = sensor_world_matrix × detected_point_local`

Robot rotates 360° while scanning walls. Detected positions are compared against ground-truth wall geometry.

- **Pass criterion:** average position error < `ERROR_THRESHOLD = 0.05 m`
- **Result:** avg error 0.025 m — PASS
- **Output:** CSV log in `simulation_tests/logs/`

---

### `test_handle_mapping.py` — Scene object handle verification

**Status:** PASS ✓  
**Purpose:** One-time verification that all scene objects have the expected handles and correct child hierarchies.

Queries `/p3dx_1`, `/p3dx_2`, `/payload`, `/rally_point`, `/Floor` and prints handle, alias, position, and children for each.

- **Verified handles:** p3dx_1=15, p3dx_2=99, payload=98, rally=97, floor=13
- **Prerequisites:** simulation NOT running

---

### `test_sensor_calibration.py` — Sensor geometry calibration

**Status:** PASS ✓  
**Purpose:** Validates the corrected forward offset for sensors [3] and [4] and tests lateral inference and detection range.

Advances robot toward a known wall at multiple distances and compares measured vs expected sensor values.

- **Calibrated constant:** `SENSOR_FORWARD_OFFSET = 0.209 m` (sensors [3],[4] are 5.1 cm behind robot front)
- **Setup required:** place p3dx_1 at (−1.75, 0.0) facing east (0°) manually in CoppeliaSim
- **Tests:** forward offset at 4 distances (0.60, 0.45, 0.30, 0.20 m), lateral sensors [0],[7] range

---

### `test_exploring.py` — EXPLORING phase isolation

**Status:** Partial  
**Purpose:** Isolated test of the exploration sweep waypoint sequence, without interference from detection or push phases.

Both robots follow their respective waypoint lists independently. Tests correctness of the outer perimeter sweep and inner convergence.

- **Result:** outer waypoints OK; inner waypoints oscillate (robot overshoots at low speed near (0,0))
- **Limitation:** inner convergence requires tighter POSITION_THR or speed reduction near centre

---

### `test_wall_follower.py` — PID wall follower

**Status:** Partial  
**Purpose:** Verifies PID-based wall following on `/p3dx_1` using lateral sensor pairs with hysteresis for side switching.

- **Sensor pairs:** right [7,8], left [0,15]
- **Hysteresis:** `HYSTERESIS = 0.15 m` margin + `CONFIRM_CYCLES = 8` consecutive readings before switching side
- **Result:** PID works correctly; requires a wall already in sensor range at start — fails in open space

---

### `test_wall_approach.py` — Wall-referenced navigation (pending)

**Status:** Pending  
**Purpose:** Navigation using a vector field derived from oblique sensor groups. Designed to handle wall-referenced approach from open space without requiring a wall in range at start.

Key improvement over `test_wall_follower.py`: uses oblique sensors [5,6,7,8] (right) and [15,0,1,2] (left) to produce an angular gradient — avoids degenerate vector field (wx=0, wy=±1) that caused oscillation in frontal-only configurations.

- **Sensor angles:** verified 2026-05 via `sim.getObjectMatrix(sensor, robot)`

---

### `test_push_alignment.py` — Approach, contact and alignment sequence (pending)

**Status:** Pending  
**Purpose:** Tests the full single-robot push sequence with explicit contact detection and alignment assessment before committing to a push.

Phases: approach → contact detection via `checkCollision` → alignment check (log_ratio, centroid) → push toward rally for 8 s if aligned, back off and reorient if misaligned.

- **Contact detection:** `checkCollision` API (not proximity sensors)
- **Alignment metric:** `log_ratio` of frontal sensor pair + centroid position

---

### `test_push_speed.py` — Push speed sweep

**Status:** Reference / completed  
**Purpose:** Parametric sweep of `PUSH_SPD` values for single-robot push. Reports steps, time, final heading error, and lateral drift for each speed.

Used to determine the optimal push speed before fixing `PUSH_SPD = 2.0`. Not intended for regression testing.

---

### `test_sensor_compare.py` — Sensor API comparison (diagnostic)

**Status:** Diagnostic tool  
**Purpose:** Compares `readProximitySensor` (legacy) vs `checkProximitySensorEx` for all 16 sensors with the robot stationary near a wall.

Used once to determine which API call to use for sensor readings. Result: `readProximitySensor` was selected as the primary interface.

- **Prerequisites:** robot stationary near a wall; simulation NOT running

---

### `test_sensor_identification.py` — Handle resolution by hierarchy (utility)

**Status:** Utility  
**Purpose:** Resolves any detected object handle to its root scene object by walking up the parent hierarchy. Results are cached on first traversal.

Used during development to identify which object a sensor had detected (payload vs floor vs robot) from raw collision handles.

---

### `test_sensor_angles.py` — Sensor angle dump (utility)

**Status:** Utility (one-shot script)  
**Purpose:** Prints the angular orientation of each of the 16 ultrasonic sensors relative to the robot frame using `sim.getObjectMatrix`.

Used once to verify and document the Pioneer P3DX sensor layout. The resulting angles (±90°, ±50°, ±30°, ±10°) are now hardcoded as stable constants in the codebase.

---

### `test_two_robots.py` — Fixed-role approach (superseded)

**Status:** Superseded by `test_planner.py`  
**Purpose:** First two-robot cooperative attempt using fixed role assignment: P1 as longitudinal pusher (GT direction + sensor centering), P2 as yaw stabilizer (cancels payload rotation via perpendicular force ∝ yaw error).

- **P2 role:** corrects drift, not a full pusher — `PUSH_SPD_P2 = 0.2`
- **Limitation:** fixed roles assumed robots approached from the west. After exploring east, approach geometry broke
- **Replaced by:** `test_planner.py` with ContactPlanner NNLS assignment

---

## Status summary

### Passed

| Script | Date | Result |
|--------|------|--------|
| `test_sensor_360.py` | 2026-05 | PASS — position formula verified, avg error 0.025 m |
| `test_handle_mapping.py` | 2026-05 | PASS — all object handles confirmed |
| `test_sensor_calibration.py` | 2026-05 | PASS — SENSOR_FORWARD_OFFSET = 0.209 m validated |
| `test_single_push.py` | 2026-05 | PASS — 826 steps, 0.3293 m final dist, deterministic |
| `test_planner.py` | 2026-05 | PASS — 1 466 steps, NE+SE faces, 0 stall resets |
| `test_planner_benchmark.py` | 2026-05 | PASS — 5/5 scenarios; see results above |

### Partial

| Script | Result |
|--------|--------|
| `test_exploring.py` | Outer waypoints OK; inner waypoints oscillate near centre |
| `test_wall_follower.py` | PID OK; requires wall in sensor range at start |

### Active / in use

| Script | Purpose |
|--------|---------|
| `test_planner_diag.py` | Causal force/torque analysis — no pass/fail |

### Pending

| Script | Purpose |
|--------|---------|
| `test_wall_approach.py` | Wall-referenced navigation from open space |
| `test_push_alignment.py` | Approach + contact detection + alignment + push |

### Reference / utility (completed, not regression tests)

| Script | Purpose |
|--------|---------|
| `test_push_speed.py` | Push speed parametric sweep |
| `test_sensor_compare.py` | readProximitySensor vs checkProximitySensorEx comparison |
| `test_sensor_identification.py` | Handle-to-object resolution via parent hierarchy |
| `test_sensor_angles.py` | Pioneer P3DX sensor angle dump |

### Superseded

| Script | Replaced by | Notes |
|--------|-------------|-------|
| `test_two_robots.py` | `test_planner.py` | Fixed-role approach; broke after east exploration |

---

## Architecture note

`test_planner.py`, `test_planner_diag.py`, `test_planner_benchmark.py` share the same control stack:

- `src/planner.py` — ContactPlanner: 8-direction NNLS, sticky locks, epsilon hysteresis, safe waypoints
- `src/robot.py` — Robot class: sensors, motors, `drive_to`, `get_yaw`
- Per-robot NAVIGATE→PUSH→BACKOFF state machine
- Unified sensor routing: each robot reads its own d3/d4 independently every tick
- EMA stall detection (active only when both robots in PUSH simultaneously)
- Per-robot NAV timeout (400 steps) with reset that omits `robot.stop()` to avoid CoppeliaSim physics freeze
