# Experiments

Experimental verification scripts that require CoppeliaSim running with the scene loaded.
These are not automated tests — they require simulated hardware and produce logs for analysis.

## How to run
```bash
# From project root, with CoppeliaSim open and scene loaded
python -m experiments.test_sensor_identification
```
Logs are saved automatically to `experiments/logs/` with a timestamp.
Results and conclusions are documented in `experiments/results/`.

## Status

### Completed
| Script | Date | Result |
|--------|------|--------|
| `test_exploring.py` | 2026-05 | Outer waypoints OK, inner waypoints oscillate. See results/ |
| `test_wall_follower.py` | 2026-05 | PID works but requires wall in range at start. See results/ |
| `test_drone_sensor.py` | 2026-05 | Drone has no sensors confirmed. See results/ |

### Priority 1 — Sensor verification
| Script | Status | Purpose |
|--------|--------|---------|
| `test_sensor_identification.py` | PENDING | Verify `readProximitySensor` returns detected object handle — distinguish box, wall, other robot |
| `test_sensor_360.py` | PENDING | Robot spins 360°, confirms real sensor map and robot forward axis |
| `test_robot_orientation.py` | PENDING | Determine Pioneer forward axis in world coordinates |

### Priority 2 — Movement verification
| Script | Status | Purpose |
|--------|--------|---------|
| `test_wall_approach.py` | PENDING | Robot advances until wall detected — confirms detection distance and first sensor to activate |
| `test_wall_following_pid.py` | PENDING | Full wall following 60s with PID log — confirms optimal parameters at 1.0m range |
| `test_motor_response.py` | PENDING | Real robot velocity vs `setJointTargetVelocity` value |

### Priority 3 — Pushing verification
| Script | Status | Purpose |
|--------|--------|---------|
| `test_push_single.py` | PENDING | Single robot pushes box — measures effective force and lateral deviation |
| `test_push_alignment.py` | PENDING | Two robots with different offsets — confirms optimal push geometry |

### Priority 4 — Multi-agent interaction
| Script | Status | Purpose |
|--------|--------|---------|
| `test_deadlock_prevention.py` | PENDING | Two robots face to face — verifies priority hierarchy prevents deadlock |
| `test_exploring_coordination.py` | PENDING | Coordinated exploration with shared map — verifies coverage and no collisions |