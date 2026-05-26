"""
test_wall_approach.py

Wall-referenced navigation using vector field with correct sensor groups.

Sensor angles verified 2026-05-25 via sim.getObjectMatrix(sensor, robot):
  Standard Pioneer P3DX layout — hardcoded as stable constants.

Key fix: using oblique sensors [5,6,7,8] for right wall and [15,0,1,2]
for left wall provides angular gradient — vector field is no longer
degenerate (wx=0, wy=±1).

Usage:
    python -m simulation_tests.test_wall_approach
"""

import time
import math
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from src.robot import Robot


# Verified 2026-05-25 via sim.getObjectMatrix(sensor, robot)
SENSOR_ANGLES_DEG = [
    +90, +50, +30, +10, -10, -30, -50, -90,
    -90, -130, -150, -170, +170, +150, +130, +90
]
SENSOR_ANGLES_RAD = [math.radians(a) for a in SENSOR_ANGLES_DEG]

# Sensor groups with angular gradient for wall estimation
RIGHT_WALL = [5, 6, 7, 8]     # -30°, -50°, -90°, -90° rear
LEFT_WALL  = [15, 0, 1, 2]    # +90° rear, +90°, +50°, +30°
FRONTAL    = [2, 3, 4, 5]     # ±30°, ±10° — near-frontal arc

# Navigation parameters
NAV_SPEED   = 1.5
TURN_SPEED  = 1.0
WALL_TARGET = 0.25
FRONT_BLOCK = 0.30
DEAD_END    = 0.12
ANGLE_GAIN  = 0.8
DIST_GAIN   = 0.5
MAX_BIAS    = 0.30

SPEED_DIST_MIN = 0.15
SPEED_DIST_MAX = 0.50
SPEED_MIN_FRAC = 0.2

TS = 0.05


def wall_vector(distances, sensors):
    """
    Estimate dominant obstacle direction from given sensor group.
    Uses exp(-3*d) weights — bounded [0,1], stable.
    Returns (vx, vy) unit vector pointing TOWARD nearest obstacles,
    or (0,0) if nothing detected.
    """
    vx, vy = 0.0, 0.0
    for i in sensors:
        d = distances[i]
        if d >= 1.0:
            continue
        w = math.exp(-3.0 * d)
        vx += math.cos(SENSOR_ANGLES_RAD[i]) * w
        vy += math.sin(SENSOR_ANGLES_RAD[i]) * w
    norm = math.sqrt(vx*vx + vy*vy)
    if norm < 1e-6:
        return 0.0, 0.0
    return vx / norm, vy / norm


def wall_distance_est(distances, sensors):
    """Weighted mean distance for given sensor group."""
    valid = [distances[i] for i in sensors if distances[i] < 1.0]
    if not valid:
        return 1.0
    return sum(valid) / len(valid)


def main():
    print("=== TEST: Wall-referenced navigation — /p3dx_1 ===\n")

    client = RemoteAPIClient()
    sim    = client.getObject('sim')
    sim.startSimulation()
    time.sleep(0.5)

    robot = Robot(sim, '/p3dx_1', name='1')

    # Determine initial wall side
    readings  = robot.read_sensors_legacy()
    distances = [d if det else 1.0 for det, d in readings]
    right_min = min(distances[i] for i in RIGHT_WALL)
    left_min  = min(distances[i] for i in LEFT_WALL)
    wall_side = RIGHT_WALL if right_min <= left_min else LEFT_WALL
    side_name = 'right' if wall_side is RIGHT_WALL else 'left'
    print(f"Initial side: {side_name}  "
          f"(right_min={right_min:.2f}  left_min={left_min:.2f})\n")

    print(f"{'action':>22} {'wx':>7} {'wy':>7} {'ang_err':>8} "
          f"{'dist':>7} {'dist_err':>9} {'spd':>6}")
    print("-" * 76)

    t_start = time.time()
    while time.time() - t_start < 60:

        readings  = robot.read_sensors_legacy()
        distances = [d if det else 1.0 for det, d in readings]

        # Frontal check
        front = min(distances[i] for i in FRONTAL)

        # Adaptive speed
        speed_frac = (front - SPEED_DIST_MIN) / (SPEED_DIST_MAX - SPEED_DIST_MIN)
        speed_frac = max(SPEED_MIN_FRAC, min(1.0, speed_frac))
        cur_speed  = NAV_SPEED * speed_frac

        # Dead end
        if front < DEAD_END:
            robot.turn_around(TURN_SPEED)
            time.sleep(0.5)
            print(f"{'turn_around':>22}")
            time.sleep(TS)
            continue

        # Frontal blocked — turn toward side with more space
        if front < FRONT_BLOCK:
            right_min = min(distances[i] for i in RIGHT_WALL)
            left_min  = min(distances[i] for i in LEFT_WALL)
            if left_min > right_min:
                action = 'turn_left'
                wall_side = LEFT_WALL
                side_name = 'left'
                robot.turn_left(TURN_SPEED)
            else:
                action = 'turn_right'
                wall_side = RIGHT_WALL
                side_name = 'right'
                robot.turn_right(TURN_SPEED)
            time.sleep(0.3)
            print(f"{action:>22} {'---':>7} {'---':>7} {'---':>8} "
                  f"{front:>7.3f} {'---':>9} {speed_frac:>6.2f}")
            time.sleep(TS)
            continue

        # Choose wall side — switch if other side significantly closer
        right_min = min(distances[i] for i in RIGHT_WALL)
        left_min  = min(distances[i] for i in LEFT_WALL)
        if right_min < left_min - 0.15:
            wall_side = RIGHT_WALL
            side_name = 'right'
        elif left_min < right_min - 0.15:
            wall_side = LEFT_WALL
            side_name = 'left'

        # Vector field from chosen wall group
        wx, wy = wall_vector(distances, wall_side)

        if wx == 0.0 and wy == 0.0:
            # No wall detected — advance straight
            action = 'forward_open'
            robot.move_forward(cur_speed)
            print(f"{action:>22} {'---':>7} {'---':>7} {'---':>8} "
                  f"{'---':>7} {'---':>9} {speed_frac:>6.2f}")
            time.sleep(TS)
            continue

        # Wall tangent perpendicular to wall normal
        tx, ty = -wy, wx

        # Angular error with correct sign
        angle_error = math.atan2(ty, tx)

        # Distance error
        dist_est = wall_distance_est(distances, wall_side)
        dist_err = dist_est - WALL_TARGET

        # Combined correction
        bias = -angle_error * ANGLE_GAIN + dist_err * DIST_GAIN
        bias = max(-MAX_BIAS, min(MAX_BIAS, bias))

        action = f'nav_{side_name}({angle_error:+.2f},{dist_err:+.2f})'
        robot.set_velocity(
            cur_speed * (1.0 - bias),
            cur_speed * (1.0 + bias))

        print(f"{action:>22} {wx:>7.3f} {wy:>7.3f} {angle_error:>8.3f} "
              f"{dist_est:>7.3f} {dist_err:>9.3f} {speed_frac:>6.2f}")
        time.sleep(TS)

    robot.stop()
    sim.stopSimulation()
    print("\nSimulation stopped.")


if __name__ == '__main__':
    main()