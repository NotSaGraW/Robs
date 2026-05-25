# Project Status

## Confirmed design decisions

### Architecture
- Motor control: Python via ZMQ Remote API
- Sensor reading: Python via ZMQ Remote API
- Pioneer Lua scripts: `sysCall_init` active (configures detection collection),
  `sysCall_actuation` commented out (Python has exclusive motor control)
- Drone Lua script: active (PID follows `/drone/target` object)
- Physics engine: Bullet V2.78
- Control loop: Sense → Think → Act in each Python cycle (20Hz)

### Knowledge model
```
KNOWN at t=0 (given by mission):
  - rally_point coordinates  ← sole absolute anchor, known by all agents
  - own agent identity and capabilities

UNKNOWN, must be discovered during mission:
  - own absolute position in world (only relative position from t=0)
  - environment dimensions and layout
  - wall positions
  - payload position
  - other agent positions (until contact)
  - obstacles

NEVER ASSUMED:
  - any position not yet confirmed by sensor or inter-agent communication
```

### Coordinate usage policy
- No hardcoded waypoints — environment is unknown at start
- Coordinates valid ONLY after confirmed by sensor contact or communication
- rally_point is the only coordinate known a priori
- All other positions are discovered and registered during mission
- Navigation at metre scale uses discovered coordinates
- Operations at centimetre scale (contact, alignment) use sensors only

### Agent identity
- Each agent has a unique recognisable signature
- Agents know their own capabilities and limitations
- Agents recognise each other when in contact range
- Identity enables cooperative map fusion and role assignment

### Multi-agent communication architecture

**Principle:** robots are locally reactive and globally aware.
Each robot acts with local sensor data. When it encounters something
ambiguous, it queries the drone. The drone does not control robots —
it resolves uncertainty and redistributes global knowledge.

**Regular channel — 2Hz heartbeat:**
```
Robot → Drone: partial map update, own position, current state
Drone → Robot: global map, other robot position, system state
```

**Urgent channel — event-driven:**
```
Robot → Drone: "detected unknown object at relative position (dist, angle)"
Drone → Robot: KNOWN_AGENT   | identity + position + movement vector
               KNOWN_OBJECT  | payload position
               KNOWN_STATIC  | wall, already in map
               UNKNOWN       | treat as static obstacle
```

**Map fusion via contact:**
- Each agent builds its own local map centred on its starting position
- Maps can only be fused once agents share a common anchor
- Drone localises rally_point first (aerial view) → establishes global reference
- Drone communicates rally_point vector to each robot on first contact
- After contact, all maps share the same coordinate system
- Without drone: robots can fuse maps on direct contact but remain unanchored
  until rally_point is detected by one of them

### Cooperative localisation sequence
```
t=0:
  All agents: local maps centred on own position, no global reference
  Drone: ascends, scans for rally_point and obstacles

Drone detects rally_point:
  Drone establishes global reference (rally_point = world anchor)

Drone contacts Robot 1:
  Transmits rally_point vector relative to Robot 1 current position
  Robot 1 map transforms to global coordinate system

Drone contacts Robot 2:
  Same process — Robot 2 map anchored to global system

All three maps fuseable — same coordinate origin (rally_point)
```

### Robot navigation — action primitives

```python
move_forward(speed)   # both wheels forward
move_backward(speed)  # both wheels backward — fine corrections,
                      # repositioning for push, post-turn adjustments
turn_left(speed)      # spin left on axis
turn_right(speed)     # spin right on axis
turn_around(speed)    # 180° — dead end recovery
stop()
```

**When to use each:**
- `move_backward`: micro-corrections, push repositioning, post-turn trim
  Never primary navigation — always use turn + forward instead
- `turn_around`: only when front, left AND right are all blocked

### Robot navigation — decision tree

```
SCAN (360° spin, build distance map at legacy range 0.5m)
  ↓
ORIENT (rotate to put nearest obstacle on right side)
  ↓
ADVANCE:
  frontal blocked?
    yes → following right wall? → turn_left()
          following left wall?  → turn_right()
          unknown?              → turn toward most space
    no  → right reference lost? → turn_right() slightly
          right reference ok?   → move_forward()
          left reference ok?    → move_forward()
  
  all sides blocked? → turn_around()
```

**Key principle:** robot advances in a direction and uses the wall as a
reference that it is on track — not as a guide rail to follow.
The wall confirms the direction is valid; it does not dictate movement.

### Wall reference — sensor pairs (verified)
```
Right side: [7] front-right lateral + [8] rear-right lateral
Left side:  [0] front-left  lateral + [15] rear-left  lateral
Frontal:    [3][4][9][10]
```

### Drone — virtual sensor model

The drone has no physical sensors. Detection is geometric (Python):
distance from drone position to known object positions.

```python
# Altitude and detection radius are linked:
PATROL_HEIGHT_LOW  = 1.5m → DETECTION_RADIUS_LOW  = 1.0m
PATROL_HEIGHT_HIGH = 2.0m → DETECTION_RADIUS_HIGH = 1.5m
```

**Altitude behaviour:**
- Start at HIGH (2.0m): maximum view for initial localisation
- Drop to LOW (1.5m): normal patrol, less interference with pioneers
- Radius capped at 1.5m to avoid interfering with pioneer sensor tests

**Drone initial sequence:**
1. Ascend to HIGH altitude (2.0m)
2. Scan environment — locate rally_point and obstacles
3. Move away from obstacles if detected within radius
4. Begin systematic patrol
5. On contact with each robot: transmit rally_point anchor vector
6. Detect payload when within detection radius

**Drone control:**
- Lua PID script active — drone follows `/drone/target` object
- Python moves `/drone/target`, Lua moves drone to follow
- Do NOT use `setObjectPosition` directly on drone handle

### Occupancy grid — `grid.py`

Current: fixed dimensions (5x5m), assumes known environment size.

Target design:
- Initialise large (10x10m) centred on agent starting position
- Origin = rally_point once anchor established
- Dimensions emerge from exploration — walls confirm boundaries
- Each agent maintains own grid; fusion occurs after anchor established

For current academic scope: fixed 5x5m grid is acceptable as
"conservative initialisation larger than the known environment".
Document as known simplification.

### System phases

| Phase | Description | Exit condition |
|-------|-------------|----------------|
| LOCALIZING | Drone ascends, locates rally_point, contacts robots to establish global reference | All agents anchored to rally_point |
| EXPLORING | Reactive navigation: SCAN→ORIENT→ADVANCE, builds shared map | Payload detected by any agent |
| CONVERGING | Robots navigate to push positions behind payload | Both confirmed by frontal sensors |
| PUSHING | Both robots push with sensor-confirmed contact | Payload reaches rally_point |
| SUCCESS | Mission complete | — |

### Nomenclature
| CoppeliaSim object | Code variable   | Role |
|--------------------|-----------------|------|
| `/p3dx_1`          | `p3dx_1`        | Pioneer P3DX, pusher 1 |
| `/p3dx_2`          | `p3dx_2`        | Pioneer P3DX, pusher 2 |
| `/drone`           | `drone`         | Quadcopter, data hub |
| `/payload`         | `payload_h`     | Object to transport |
| `/rally_point`     | `rally_point_h` | Extraction point, sole known anchor |

### Push role assignment
Pusher roles assigned dynamically at runtime based on payload position,
rally_point vector, and each robot's current position.
`/p3dx_1` and `/p3dx_2` are numbered, not named by role.

### Verified sensor map (Pioneer P3DX)
| Index | Direction | Primary use |
|-------|-----------|-------------|
| [0]   | 90° left  | Left wall reference — front |
| [3]   | ~10° left frontal | Frontal obstacle detection |
| [4]   | ~10° right frontal | Frontal obstacle detection |
| [7]   | 90° right | Right wall reference — front |
| [8]   | 90° right rear | Right wall reference — rear |
| [9-13]| Frontal cone | Frontal obstacle detection |
| [15]  | 90° left rear | Left wall reference — rear |

- Legacy range (Lua): 0.5m
- Extended range (checkProximitySensorEx): up to 1.0m
- Robot forward axis: +X when yaw=0°

### Position calculation formula (verified ✓)
```
detected_world_pos = sensor_world_matrix × detected_point_local
```
- test_sensor_360: 100% pass rate, average error 0.025m (= noDetectionDist)

### Complete handle map (verified ✓)
| Object        | Root handle |
|---------------|-------------|
| /p3dx_1       | 15 |
| /p3dx_2       | 99 |
| /payload      | 98 |
| /Floor        | 13 |
| /drone        | 141 |
| /rally_point  | 97 |

### Scene object properties
| Object        | Mass   | Respondable | Dynamic |
|---------------|--------|-------------|---------|
| /payload      | 20 kg  | YES         | YES     |
| /Floor        | 500 kg | NO          | NO      |
| /drone        | 0.1 kg | YES (props) | NO      |
| /p3dx_1       | ~9 kg  | YES         | YES     |
| /p3dx_2       | ~9 kg  | YES         | YES     |

### Friction (Bullet V2.78)
- Floor: frictionOld = 1.0, restitution = 0.5
- Payload: frictionOld = 1.0, restitution = 0.0

## Pending implementation (priority order)

1. **`robot.py`** — add action primitives (move_forward, move_backward,
   turn_left, turn_right, turn_around)
2. **`test_wall_approach.py`** — reactive navigation with decision tree
3. **`strategy.py`** — replace hardcoded waypoints with reactive EXPLORING
4. **`drone.py`** — implement dual-altitude model, fix control via target object
5. **`grid.py`** — flexible initialisation centred on agent position

## Pending verification (simulation_tests)

- [ ] **BLOCKING fixed** `test_sensor_360.py` — PASS ✓ (2026-05-25)
- [ ] `test_handle_mapping.py` — PASS ✓ (2026-05-25)
- [ ] `test_wall_approach.py` — IN PROGRESS
- [ ] `test_wall_following_pid.py` — PENDING
- [ ] `test_motor_response.py` — PENDING
- [ ] `test_push_single.py` — PENDING
- [ ] `test_push_alignment.py` — PENDING
- [ ] `test_deadlock_prevention.py` — PENDING
- [ ] `test_exploring_coordination.py` — PENDING

## Known issues
| Issue | Impact | Status |
|-------|--------|--------|
| Hardcoded waypoints in strategy.py | Assumes known environment | Pending redesign |
| Drone control via setObjectPosition on drone handle | Fights Lua PID | Fix: move /drone/target instead |
| Wall following PID loses reference in transitions | Erratic behaviour | Redesigning with decision tree |
| grid.py fixed dimensions | Assumes known environment size | Acceptable for current scope |
| FRONT_SENSORS = [2,3,4] legacy | Sensor [2] is ~60°, not frontal | Fix when updating robot.py |