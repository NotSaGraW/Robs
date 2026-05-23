"""
test_exploring.py
Test aislado de la fase EXPLORING.
Verifica que los robots barren el tablero correctamente
sin interferencias de otras fases.
"""

import time
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from src.robot import Robot
from src.drone import Drone
from src.grid  import OccupancyGrid

# Waypoints de exploración — mismos que en strategy.py
EXPLORE_WP_A = [
    [-1.8,-1.8],[0.0,-1.8],[1.8,-1.8],
    [1.8, 0.0],[1.8, 1.8],[0.0, 1.8],
    [-1.8,1.8],[-1.8,0.0],[-0.6,-0.6],
    [0.6,-0.6],[0.6, 0.6],[-0.6, 0.6],
    [0.0, 0.0],
]
EXPLORE_WP_B = [
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
        print(f"  [{robot.name}] wp[{idx}] alcanzado → "
              f"siguiente wp[{state['idx']}] {goal[:2]}")
    if robot.obstacle_ahead(0.45):
        vl, vr = robot.braitenberg()
        robot.set_velocity(vl, vr)
    else:
        robot.drive_to(goal, SPEED)

def main():
    print("=== TEST: EXPLORING ===\n")
    client = RemoteAPIClient()
    sim    = client.getObject('sim')

    robot_a = Robot(sim, '/roba', name='A')
    robot_b = Robot(sim, '/robo', name='B')
    drone   = Drone(sim, '/qua')
    grid    = OccupancyGrid()

    state_a = {'idx': 0, 'visited': []}
    state_b = {'idx': 0, 'visited': []}

    print(f"Robot A: {robot_a.get_position()[:2]} → wp[0] {EXPLORE_WP_A[0]}")
    print(f"Robot B: {robot_b.get_position()[:2]} → wp[0] {EXPLORE_WP_B[0]}")
    print(f"Drone  : {drone.get_position()[:2]}\n")

    sim.startSimulation()
    t_start = time.time()
    cycle   = 0

    try:
        while True:
            t_cycle = time.time()
            box_fake = [99.0, 99.0, 0.0]  # fuera del tablero — drone no detecta nada

            # Actualizar mapa
            pa = robot_a.get_position()
            pb = robot_b.get_position()
            grid.mark_robot_path(pa[0], pa[1])
            grid.mark_robot_path(pb[0], pb[1])
            grid.update_from_sensor(pa[0], pa[1], robot_a.get_yaw(),
                                    robot_a.read_sensors())
            grid.update_from_sensor(pb[0], pb[1], robot_b.get_yaw(),
                                    robot_b.read_sensors())
            drone.step(box_fake, grid)

            explore_step(robot_a, EXPLORE_WP_A, state_a)
            explore_step(robot_b, EXPLORE_WP_B, state_b)

            if cycle % LOG_EVERY == 0:
                elapsed = time.time() - t_start
                ca, da  = robot_a.front_contact()
                cb, db  = robot_b.front_contact()
                print(f"t={elapsed:5.1f}s "
                      f"A:({pa[0]:.2f},{pa[1]:.2f}) wp[{state_a['idx']}] "
                      f"[{'C' if ca else '-'}:{da:.2f}]  "
                      f"B:({pb[0]:.2f},{pb[1]:.2f}) wp[{state_b['idx']}] "
                      f"[{'C' if cb else '-'}:{db:.2f}]  "
                      f"mapa:{grid.coverage_percent():.0f}%")

            elapsed = time.time() - t_start
            if elapsed > MAX_TIME:
                print(f"\nTest completado ({MAX_TIME}s)")
                print(f"Waypoints A visitados: {state_a['visited']}")
                print(f"Waypoints B visitados: {state_b['visited']}")
                print(grid.stats())
                break

            time.sleep(max(0.0, STEP - (time.time() - t_cycle)))
            cycle += 1

    except KeyboardInterrupt:
        print(f"\nInterrumpido. Waypoints visitados:")
        print(f"  A: {state_a['visited']}")
        print(f"  B: {state_b['visited']}")
        print(grid.stats())
        try:
            robot_a.stop()
            robot_b.stop()
        except Exception:
            pass
    finally:
        try:
            sim.stopSimulation()
        except Exception:
            pass
        print("Test finalizado.")

if __name__ == '__main__':
    main()