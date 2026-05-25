"""
test_wall_follower.py

Verifies PID wall following on /p3dx_1.

Sensor pairs (verified):
  Right side: [7] front-right lateral, [8]  rear-right lateral
  Left  side: [0] front-left  lateral, [15] rear-left  lateral

Side selection uses hysteresis to avoid oscillation:
  - Only switches side if the other side is significantly closer
  - Requires CONFIRM_CYCLES consecutive readings before switching

Usage:
    python -m simulation_tests.test_wall_follower

Dependencies:
    pip install coppeliasim-zmqremoteapi-client

Prerequisites:
    - CoppeliaSim open with scene loaded and simulation NOT running
"""

import time
import math
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from src.robot import Robot


HYSTERESIS     = 0.15   # metres — margin required to switch side
CONFIRM_CYCLES = 8      # consecutive cycles before switching side


def main():
    print("=== TEST: Wall follower — /p3dx_1 ===\n")

    client = RemoteAPIClient()
    sim    = client.getObject('sim')
    sim.startSimulation()

    robot = Robot(sim, '/p3dx_1', name='1')

    kp_rot        = 200
    kd_rot        = 10
    ki_rot        = 30
    sum_rot_error = 0
    last_rot_error= 0
    kp_trans      = 90
    v0            = 120 * math.pi / 180
    wall_distance = 0.25
    ts            = 0.05

    # Side selection state
    current_side   = None
    votes_right    = 0
    votes_left     = 0

    print(f"{'side':>6} {'front':>8} {'wall_f':>8} {'wall_b':>8}")
    print("-" * 36)

    t_start = time.time()
    while time.time() - t_start < 30:
        readings  = robot.read_sensors_legacy()
        distances = [d if det else 1.0 for det, d in readings]

        # Side detection with hysteresis
        right_dist = min(distances[7], distances[8])
        left_dist  = min(distances[0], distances[15])

        if right_dist < left_dist - HYSTERESIS:
            votes_right += 1
            votes_left   = 0
        elif left_dist < right_dist - HYSTERESIS:
            votes_left  += 1
            votes_right  = 0
        else:
            # Too close to call — keep current side
            votes_right = max(0, votes_right - 1)
            votes_left  = max(0, votes_left  - 1)

        if votes_right >= CONFIRM_CYCLES:
            current_side = 'right'
        elif votes_left >= CONFIRM_CYCLES:
            current_side = 'left'
        elif current_side is None:
            # No side confirmed yet — pick the closer one
            current_side = 'right' if right_dist <= left_dist else 'left'

        # Select sensor pair based on confirmed side
        if current_side == 'right':
            wall_front = distances[7]
            wall_back  = distances[8]
        else:
            wall_front = distances[0]
            wall_back  = distances[15]

        front_dist = min(distances[3], distances[4],
                         distances[9], distances[10])

        # PID
        rot_error      = wall_front - wall_back
        sum_rot_error += rot_error
        pid_rot        = (kp_rot * rot_error +
                          ki_rot * sum_rot_error * ts +
                          kd_rot * (rot_error - last_rot_error) / ts)
        last_rot_error = rot_error

        trans_error = wall_front - wall_distance
        pid_trans   = kp_trans * trans_error

        if current_side == 'right':
            vLeft  = v0 + pid_rot + pid_trans
            vRight = v0 - pid_rot - pid_trans
        else:
            vLeft  = v0 - pid_rot - pid_trans
            vRight = v0 + pid_rot + pid_trans

        robot.set_velocity(vLeft, vRight)

        # Turn away from frontal obstacle
        if front_dist < 0.4:
            robot.set_velocity(-v0, v0)
            time.sleep(0.2)

        print(f"{current_side:>6} {front_dist:>8.2f} "
              f"{wall_front:>8.2f} {wall_back:>8.2f}")
        time.sleep(ts)

    robot.stop()
    sim.stopSimulation()
    print("\nSimulation stopped.")


if __name__ == '__main__':
    main()