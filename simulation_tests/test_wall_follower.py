"""
test_wall_follower.py

Verifies PID wall following on /p3dx_1.
Robot follows the right wall for 30 seconds.
Logs front distance, right-front and right-back sensor readings.

Usage:
    python -m simulation_tests.test_wall_follower

Dependencies:
    pip install coppeliasim-zmqremoteapi-client

Prerequisites:
    - CoppeliaSim open with scene loaded and simulation NOT running
    - /p3dx_1 near a wall on its right side
"""

import time
import math
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from src.robot import Robot


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

    print(f"{'front':>8} {'r_front':>8} {'r_back':>8}")
    print("-" * 28)

    t_start = time.time()
    while time.time() - t_start < 30:
        readings  = robot.read_sensors_legacy()
        distances = [d if detected else 1.0
                     for detected, d in readings]

        front_dist       = distances[4]
        right_front_dist = distances[7]
        right_back_dist  = distances[8]

        rot_error      = right_front_dist - right_back_dist
        sum_rot_error += rot_error
        pid_rot        = (kp_rot * rot_error +
                          ki_rot * sum_rot_error * ts +
                          kd_rot * (rot_error - last_rot_error) / ts)
        last_rot_error = rot_error

        trans_error = right_front_dist - wall_distance
        pid_trans   = kp_trans * trans_error

        vLeft  = v0 + pid_rot + pid_trans
        vRight = v0 - pid_rot - pid_trans
        robot.set_velocity(vLeft, vRight)

        if front_dist < 0.4:
            robot.set_velocity(-v0, v0)
            time.sleep(0.2)

        print(f"{front_dist:>8.2f} {right_front_dist:>8.2f} {right_back_dist:>8.2f}")
        time.sleep(ts)

    robot.stop()
    sim.stopSimulation()
    print("\nSimulation stopped.")


if __name__ == '__main__':
    main()