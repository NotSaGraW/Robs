# Simulation Tests

Verification scripts that require CoppeliaSim running with the scene loaded.
These are not automated unit tests — they require the simulated hardware
and produce logs and result files for analysis.

## How to run
```powershell
# From project root, with CoppeliaSim open and scene loaded
python -m simulation_tests.test_sensor_360
```

Logs are saved to `simulation_tests/logs/` with a timestamp.
Results and conclusions are saved to `simulation_tests/results/`.

## Status

### Completed
| Script | Date | Result |
|--------|------|--------|
| `test_drone_sensor.py` | 2026-05 | No sensors confirmed |
| `test_exploring.py` | 2026-05 | Outer waypoints OK, inner waypoints oscillate |
| `test_wall_follower.py` | 2026-05 | PID works but requires wall in range at start |

### Blocking
| Script | Status | Purpose |
|--------|--------|---------|
| `test_sensor_360.py` | **PENDING — metric fix needed** | Verify position calculation formula with corrected metric (`distance_to_nearest_plane`) |

### Priority 1 — Sensor verification
| Script | Status | Purpose |
|--------|--------|---------|
| `test_handle_mapping.py` | PENDING | Verify all object handles with new nomenclature |
| `test_sensor_identification.py` | PENDING | Verify `readProximitySensor` returns detected object handle |

### Priority 2 — Movement verification
| Script | Status | Purpose |
|--------|--------|---------|
| `test_wall_approach.py` | PENDING | Robot advances until wall detected from open space |
| `test_wall_following_pid.py` | PENDING | Full wall following 60s with PID log |
| `test_motor_response.py` | PENDING | Real robot velocity vs `setJointTargetVelocity` value |

### Priority 3 — Push verification
| Script | Status | Purpose |
|--------|--------|---------|
| `test_push_single.py` | PENDING | Single robot pushes payload |
| `test_push_alignment.py` | PENDING | Two robots, different offsets — optimal push geometry |

### Priority 4 — Multi-agent
| Script | Status | Purpose |
|--------|--------|---------|
| `test_deadlock_prevention.py` | PENDING | Two robots face to face — priority hierarchy |
| `test_exploring_coordination.py` | PENDING | Coordinated exploration with shared map |