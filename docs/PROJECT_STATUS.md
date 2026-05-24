# Project Status

## Confirmed design decisions

### Architecture
- Motor control: Python via ZMQ Remote API
- Sensor reading: Python via ZMQ Remote API
- Pioneer Lua scripts: `sysCall_init` active (configures detection collection),
  `sysCall_actuation` commented out (Python has exclusive motor control)
- Drone Lua script: disabled, direct control via `setObjectPosition`
- Physics engine: Bullet V2.78
- Control loop: Sense → Think → Act in each Python cycle (20Hz)

### Knowledge model
```
KNOWN at t=0 (given by mission):
  - rally_point position  ← sole absolute anchor

UNKNOWN, must be discovered during mission:
  - environment dimensions
  - wall positions
  - payload position
  - other robot positions (until drone communication)
  - obstacles

NEVER ASSUMED:
  - any position not yet confirmed by sensor or communication
```

### Coordinate usage policy
- Coordinates are valid ONLY after an object has been confirmed by sensor
- Navigation at metre scale (exploration waypoints): coordinates OK
- Operations at centimetre scale (contact, alignment, success): sensors only
- rally_point is the only coordinate known a priori — all others must be discovered

### Multi-agent communication architecture

**Principle:** robots are locally reactive and globally aware.
Each robot acts fast with local sensor data. When it encounters something
ambiguous, it queries the drone. The drone does not control robots —
it resolves uncertainty and redistributes global knowledge.

**Regular channel — heartbeat at 2Hz:**
```
Robot → Drone: partial map update, own position, current state
Drone → Robot: global map, other robot position, system state
```

**Urgent channel — event-driven:**
```
Robot → Drone: "detected unknown object at world position (x, y)"
Drone → Robot: KNOWN_AGENT   | identity + position + movement vector
               KNOWN_OBJECT  | payload position
               KNOWN_STATIC  | wall, already in map
               UNKNOWN       | treat as static obstacle
```

**Why the drone acts as data hub:**
- Elevated position avoids line-of-sight interference
- Has global view neither ground robot has individually
- Broadcast frequency (2Hz) simulates real radio transmission intervals
- rally_point is the shared coordinate origin for the global map

### Nomenclature
| CoppeliaSim object | Code variable  | Role |
|--------------------|----------------|------|
| `/p3dx_1`          | `p3dx_1`       | Pioneer P3DX, pusher 1 |
| `/p3dx_2`          | `p3dx_2`       | Pioneer P3DX, pusher 2 |
| `/drone`           | `drone`        | Quadcopter, data hub |
| `/payload`         | `payload_h`    | Object to transport |
| `/rally_point`     | `rally_point_h`| Extraction point, sole known anchor |

### Push role assignment
Pusher roles (which side each robot pushes from) are assigned dynamically
at runtime based on payload position, rally_point vector, and each robot's
current position. `/p3dx_1` and `/p3dx_2` are numbered, not named by role,
because their push side changes depending on where the payload is found.

### System phases
| Phase      | Description | Exit condition |
|------------|-------------|----------------|
| EXPLORING  | Wall following builds shared occupancy map | Payload detected by any agent |
| CONVERGING | Robots navigate to push positions | Both confirmed by frontal sensors |
| PUSHING    | Both robots push with sensor-confirmed contact | Payload reaches rally point |
| SUCCESS    | Mission complete | — |

### Verified sensor map (Pioneer P3DX)
| Index | Orientation (a, b, g) | Direction | Primary use |
|-------|-----------------------|-----------|-------------|
| [0]   | -90, 0, -180          | 90° left  | Left wall following |
| [3]   | -90, +80, +180        | ~10° left (near frontal) | Payload contact detection |
| [4]   | +90, +80, 0           | ~10° right (near frontal) | Payload contact detection |
| [7]   | +90, 0, 0             | 90° right | Right wall following — side |
| [8]   | +90, 0, +0.02         | 90° right rear | Right wall following — rear |

- Real range: 1.0m (Lua limits to 0.5m via `noDetectionDist`)
- Type: cone-type, aperture ±45°, minimum distance 0.05m
- Robot forward axis: +X when yaw=0°

### Position calculation formula (verified)
```
detected_world_pos = sensor_world_matrix × detected_point_local
```
Verified by analysis of test_sensor_360 results:
- All detected points fall on correct wall planes with error ≈ 0.025m
- 0.025m offset = noDetectionDist (expected behaviour)
- Formula is accurate — test metric was wrong (compared against segment
  centres instead of wall planes)

### Complete handle map (verified)
| Object        | Root handle | Child handles |
|---------------|-------------|---------------|
| /p3dx_1       | 15 | 18,19,21,22,24,26,27,28,29 |
| /p3dx_2       | 99 | 102,103,105,106,108,110,111,112,113 |
| /payload      | 98 | — |
| /Floor        | 13 | 14 |
| /drone        | 141 | 144,145,147,149,150,152,154,155,157,159,160,162,164 |
| /rally_point  | 97 | — |
| walls         | 57–95 (odd) | 58–96 (even), one visible child each |

Note: `getObjectsInTree` does not return ultrasonicSensors or motor joints.
These must be queried explicitly per robot at init.

### Scene object properties
| Object        | Mass   | Respondable | Dynamic | Detectable |
|---------------|--------|-------------|---------|------------|
| /payload      | 20 kg  | YES         | YES     | YES        |
| /Floor        | 500 kg | NO          | NO      | YES        |
| /drone        | 0.1 kg | YES (props) | NO      | NO         |
| /p3dx_1       | ~9 kg  | YES         | YES     | YES        |
| /p3dx_2       | ~9 kg  | YES         | YES     | YES        |

### Friction (Bullet V2.78)
- Floor: frictionOld = 1.0, restitution = 0.5
- Payload: frictionOld = 1.0, restitution = 0.0

## BLOCKING PREREQUISITE

### test_sensor_360.py — position calculation formal confirmation
Formula verified by analysis but formal test pass pending.
Fix needed: replace `nearest_wall()` with `distance_to_nearest_plane()`:
```python
def distance_to_nearest_plane(pos, wall_bounds=2.525):
    dx = abs(abs(pos[0]) - wall_bounds)
    dy = abs(abs(pos[1]) - wall_bounds)
    return min(dx, dy)
```
Expected result after fix: >99% pass rate, average error ≈ 0.025m.

## Pending verification (simulation_tests)
- [ ] **BLOCKING** Position calculation formal confirmation (`test_sensor_360.py` with fixed metric)
- [ ] Wall following from open position (`test_wall_approach.py`)
- [ ] Optimal PID parameters at 1.0m range (`test_wall_following_pid.py`)
- [ ] Optimal push geometry (`test_push_alignment.py`)
- [ ] Deadlock prevention (`test_deadlock_prevention.py`)
- [ ] Full communication pipeline (drone identification via urgent channel)

## Known issues
| Issue | Impact | Status |
|-------|--------|--------|
| `FRONT_SENSORS = [2,3,4]` in `robot.py` | Sensor [2] is ~60°, not truly frontal | Fix: use [3,4] only |
| Wall following requires wall in range at start | Cannot start from open space | Fix: add approach phase |
| No BFS pathfinding implemented | Planning phase not functional | Pending |
| Exploration waypoints hardcoded | Assumes known environment | Pending: reactive exploration |
| `getObjectsInTree` misses sensors and joints | Self-handle set incomplete | Fix: explicit sensor query at init |

## Project structure
```
src/                 → main system code
unit_tests/          → pytest, no CoppeliaSim required
simulation_tests/    → verification scripts, CoppeliaSim required
  logs/              → structured output from each run
  results/           → analysis and conclusions
docs/                → technical documentation
```