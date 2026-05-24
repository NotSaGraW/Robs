"""
test_exploring.py

Isolated test of the EXPLORING phase.
Verifies that both robots sweep the area correctly
without interference from other phases.

Usage:
    python -m simulation_tests.test_exploring

Dependencies:
    pip install coppeliasim-zmqremoteapi-client

Prerequisites:
    - CoppeliaSim open with scene loaded and simulation NOT running
"""

import time
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from src.robot import Robot
from src.drone import Drone
from src.grid  import OccupancyGrid


# Exploration waypoints — same as strategy.py
EXPLORE_WP_1 = [
    [-1.8,-1.8],[0.0,-1.8],[1.8,-1.8],
    [1.8, 0.0],[1.8, 1.8],[0.0, 1.8],
    [-1.8,1.8],[-1.8,0.0],[-0.6,-0.6],
    [0.6,-0.6],[0.6, 0.6],[-0.6, 0.6],
    [0.0, 0.0],
]
EXPLORE_WP_2 = [
    [1.8, 1.8],[0.0, 1.8],[-1.8,1.8],
    [-1.8,0.0],[-1.8,-1.8],[0.0,-1.8],
    [1.8,-1.8],[1.8, 0.0],[0.6, 0.6],
    [-0.6,0.6],[-0.6,-0.6],[0.6,-0.6],
    [0.0, 0.0],
]

STEP     = 0.05
LOG_EVERY= 10
MAX_TIME = 120.0
SPEED    = 3.5


def explore_step(robot, wps, state):
    idx  = state['idx']
    goal = wps[idx] + [0.0]
    if robot.is_at(goal, 0.30):
        state['idx'] = (idx + 1) % len(wps)
        state['visited'].append(idx)
        goal = wps[state['idx']] + [0.0]
        print(f"  [{robot.name}] wp[{idx}] reached → "
              f"next wp[{state['idx']}] {goal[:2]}")

    if robot.obstacle_ahead(robot.RANGE_EXTENDED):
        vl, vr = robot.braitenberg()
        robot.set_velocity(vl, vr)
    else:
        robot.drive_to(goal, SPEED)


def main():
    print("=== TEST: EXPLORING ===\n")

    client = RemoteAPIClient()
    sim    = client.getObject('sim')

    p3dx_1 = Robot(sim, '/p3dx_1', name='1')
    p3dx_2 = Robot(sim, '/p3dx_2', name='2')
    drone  = Drone(sim, '/drone')
    grid   = OccupancyGrid()

    state_1 = {'idx': 0, 'visited': []}
    state_2 = {'idx': 0, 'visited': []}

    print(f"P3DX 1: {p3dx_1.get_position()[:2]} → wp[0] {EXPLORE_WP_1[0]}")
    print(f"P3DX 2: {p3dx_2.get_position()[:2]} → wp[0] {EXPLORE_WP_2[0]}")
    print(f"Drone : {drone.get_position()[:2]}\n")

    sim.startSimulation()
    t_start = time.time()
    cycle   = 0

    try:
        while True:
            t_cycle = time.time()

            # Payload out of range — drone finds nothing
            payload_fake = [99.0, 99.0, 0.0]

            # Update map
            p1 = p3dx_1.get_position()
            p2 = p3dx_2.get_position()
            grid.mark_robot_path(p1[0], p1[1])
            grid.mark_robot_path(p2[0], p2[1])
            grid.update_from_sensor(p1[0], p1[1], p3dx_1.get_yaw(),
                                    p3dx_1.read_sensors_legacy())
            grid.update_from_sensor(p2[0], p2[1], p3dx_2.get_yaw(),
                                    p3dx_2.read_sensors_legacy())
            drone.step(payload_fake, grid)

            explore_step(p3dx_1, EXPLORE_WP_1, state_1)
            explore_step(p3dx_2, EXPLORE_WP_2, state_2)

            if cycle % LOG_EVERY == 0:
                elapsed = time.time() - t_start
                det_1, dist_1 = p3dx_1.front_contact()
                det_2, dist_2 = p3dx_2.front_contact()
                print(f"t={elapsed:5.1f}s "
                      f"1:({p1[0]:.2f},{p1[1]:.2f}) wp[{state_1['idx']}] "
                      f"[{'C' if det_1 else '-'}:{dist_1:.2f}]  "
                      f"2:({p2[0]:.2f},{p2[1]:.2f}) wp[{state_2['idx']}] "
                      f"[{'C' if det_2 else '-'}:{dist_2:.2f}]  "
                      f"map:{grid.coverage_percent():.0f}%")

            elapsed = time.time() - t_start
            if elapsed > MAX_TIME:
                print(f"\nTest complete ({MAX_TIME}s)")
                print(f"Waypoints 1 visited: {state_1['visited']}")
                print(f"Waypoints 2 visited: {state_2['visited']}")
                print(grid.stats())
                break

            time.sleep(max(0.0, STEP - (time.time() - t_cycle)))
            cycle += 1

    except KeyboardInterrupt:
        print(f"\nInterrupted.")
        print(f"  1: {state_1['visited']}")
        print(f"  2: {state_2['visited']}")
        print(grid.stats())
        try:
            p3dx_1.stop()
            p3dx_2.stop()
        except Exception:
            pass
    finally:
        try:
            sim.stopSimulation()
        except Exception:
            pass
        print("Test finished.")


if __name__ == '__main__':
    main()