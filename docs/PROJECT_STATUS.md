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

### Control cycle
Each cycle the robot:
1. Reads all 16 sensors
2. Identifies detected object by handle (box, wall, other robot)
3. Decides action based on current phase and sensor data
4. Executes action

### Verified sensor map (Pioneer P3DX)
| Index | Orientation (a, b, g) | Direction | Primary use |
|-------|-----------------------|-----------|-------------|
| [0]   | -90, 0, -180          | 90° left  | Left wall following |
| [3]   | -90, +80, +180        | ~10° left (near frontal) | Box contact detection |
| [4]   | +90, +80, 0           | ~10° right (near frontal) | Box contact detection |
| [7]   | +90, 0, 0             | 90° right | Right wall following — side |
| [8]   | +90, 0, +0.02         | 90° right rear | Right wall following — rear |
| [11]  | +90, -80, +0.1        | ~10° right rear | Rear detection |
| [13]  | -90, -60, +180        | ~30° left rear | Rear detection |

- Real range: 1.0m (Lua limits to 0.5m via `noDetectionDist`)
- Type: cone-type, aperture ±45°, minimum distance 0.05m
- `readProximitySensor` returns detected object handle — allows identifying box vs wall vs robot

### Scene object properties
| Object | Mass   | Respondable | Dynamic | Detectable |
|--------|--------|-------------|---------|------------|
| /box   | 20 kg  | YES         | YES     | YES        |
| /Floor | 500 kg | NO          | NO      | YES        |
| /qua   | 0.1 kg | YES (props) | NO      | NO         |
| /roba  | ~9 kg  | YES         | YES     | YES        |
| /robo  | ~9 kg  | YES         | YES     | YES        |

### Friction (Bullet V2.78)
- Floor: frictionOld = 1.0, restitution = 0.5
- Box: frictionOld = 1.0, restitution = 0.0

### Drone capabilities
- No proximity or vision sensors
- Box detection: geometric only (Euclidean distance via `getObjectPosition`)
- Control: direct `setObjectPosition` (Lua disabled)
- Weight: 0.1 kg — does not interfere physically with box

### System phases
| Phase | Description | Exit condition |
|-------|-------------|----------------|
| ORIENTATION | Robot spins reading all sensors | Full 360° completed |
| EXPLORING | Wall following, builds shared occupancy map | Box detected by any agent |
| DETECTING | Box found, all agents stop | Box position confirmed |
| PLANNING | BFS over map computes safe routes | Routes calculated for A and B |
| POSITIONING | A positions (B idle), then B positions (A idle) | Both confirmed by sensors [3,4] |
| PUSHING | Both robots push, sensor contact confirmed | Box reaches target |
| SUCCESS | Mission complete | — |

## Identified interaction scenarios
| Scenario | Description | Planned solution |
|----------|-------------|-----------------|
| Deadlock | Two robots face to face, both stop | Fixed priority hierarchy: A > B |
| Livelock | Both yield simultaneously | Same fixed hierarchy |
| Moving object | Box approaching robot during exploration | Identify by handle, coordinate |
| Robot in push path | B in trajectory of box pushed by A | B recognises box by handle |
| Collision during exploration | Trajectories cross | Shared map + priority hierarchy |
| Robot trapped in corner | Local loop in wall following | Pending: define escape strategy |

## Naming conventions
| Element | Convention | Example |
|---------|------------|---------|
| Files | snake_case, English | `robot.py`, `test_sensor_360.py` |
| Classes | PascalCase, English | `Robot`, `OccupancyGrid` |
| Methods / functions | snake_case, English | `read_sensors()`, `mark_free()` |
| Variables | snake_case, English | `box_position`, `wall_distance` |
| Constants | UPPER_SNAKE_CASE, English | `MAX_SPEED`, `FRONT_SENSORS` |
| Documentation | English | All .md files |
| Log output | English | `[INFO] Box detected at t=16.74s` |

## Pending verification (experiments)
- [ ] Object identification by handle from Python (`test_sensor_identification.py`)
- [ ] Real forward axis of Pioneer in world coordinates (`test_robot_orientation.py`)
- [ ] Wall following from open position (`test_wall_approach.py`)
- [ ] Optimal PID parameters at 1.0m range (`test_wall_following_pid.py`)
- [ ] Optimal push geometry with two robots (`test_push_alignment.py`)
- [ ] Deadlock prevention (`test_deadlock_prevention.py`)

## Known issues
| Issue | Impact | Status |
|-------|--------|--------|
| Sensor angles in `grid.py` use approximate values | Map accuracy ~60° sensors | Pending fix after `test_sensor_360.py` |
| `FRONT_SENSORS = [2,3,4]` in `robot.py` | Sensor [2] is ~60°, not truly frontal | Pending fix: use [3,4] only |
| Wall following requires wall in range at start | Cannot start from open space | Pending: add approach phase |
| No BFS pathfinding implemented yet | Planning phase not functional | Pending implementation |