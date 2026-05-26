"""
test_sensor_identification.py

Resolves detected object handles by walking up the scene hierarchy.
Results are cached — first detection traverses parents, subsequent ones are instant.

Usage:
    python -m simulation_tests.test_sensor_identification
"""

import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from src.robot import Robot


ROOT_HANDLES = {
    15:  'p3dx_1',
    99:  'p3dx_2',
    141: 'drone',
    98:  'payload',
    13:  'floor',
    97:  'rally_point',
}

_parent_cache = {}


def resolve_handle(sim, handle):
    """
    Resolves a handle to its root agent name by walking up the hierarchy.
    Results are cached after first resolution.
    """
    if handle in ROOT_HANDLES:
        return ROOT_HANDLES[handle]
    if handle in _parent_cache:
        return _parent_cache[handle]

    current = handle
    visited = [handle]
    while current != -1:
        try:
            parent = sim.getObjectParent(current)
        except Exception:
            break
        if parent in ROOT_HANDLES:
            name = ROOT_HANDLES[parent]
            for h in visited:
                _parent_cache[h] = name
            return name
        if parent == -1:
            break
        current = parent
        visited.append(current)

    name = f'unknown(h={handle})'
    _parent_cache[handle] = name
    return name


def main():
    print("=== TEST: Sensor identification with hierarchy resolution ===\n")

    client = RemoteAPIClient()
    sim    = client.getObject('sim')
    sim.startSimulation()
    time.sleep(0.5)

    robot      = Robot(sim, '/p3dx_1', name='1')
    target_h   = sim.getObject('/p3dx_2')
    target_pos = sim.getObjectPosition(target_h, -1)

    print(f"p3dx_1: {[round(x,3) for x in robot.get_position()[:2]]}")
    print(f"p3dx_2: {[round(x,3) for x in target_pos[:2]]}\n")
    print(f"{'t':>5} {'S':>3} {'dist':>7} {'object':>20}  {'h_raw':>10}")
    print("-" * 50)

    t_start = time.time()
    while time.time() - t_start < 20.0:

        readings = robot.read_sensors_legacy()

        # Speed proportional to nearest frontal obstacle
        frontal  = [1, 2, 3, 4, 5, 6]
        dists    = [readings[i][1] for i in frontal
                    if readings[i][0] and readings[i][1] < 1.0]
        min_front = min(dists) if dists else 1.0

        if min_front < 0.15:
            robot.stop()
        elif min_front < 0.40:
            vl, vr = robot.braitenberg()
            robot.set_velocity(vl, vr)
        else:
            speed = 2.0 * min(1.0, (min_front - 0.15) / 0.35)
            robot.drive_to(target_pos, speed)

        # Identify all detections
        t = time.time() - t_start
        any_detected = False

        for i, h in enumerate(robot.sensors):
            try:
                res = sim.readProximitySensor(h)
                if res[0] > 0:
                    any_detected = True
                    dist       = res[1]
                    obj_handle = int(res[3])
                    obj_name   = resolve_handle(sim, obj_handle)
                    print(f"{t:5.1f} [{i:>2}] {dist:7.3f}m "
                          f"{obj_name:>20}  h={obj_handle}")
            except Exception:
                pass

        if not any_detected:
            pos = robot.get_position()
            print(f"{t:5.1f} --- nothing  pos=({pos[0]:.2f},{pos[1]:.2f})")

        time.sleep(0.05)

    robot.stop()
    sim.stopSimulation()
    print(f"\nCache built: {len(_parent_cache)} handles resolved")
    print("\nSimulation stopped.")


if __name__ == '__main__':
    main()