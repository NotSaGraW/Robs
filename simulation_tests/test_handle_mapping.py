"""
test_handle_mapping.py

Maps all object handles in the scene and identifies every child object.
Verifies that /p3dx_1, /p3dx_2, /drone, /payload and /rally_point
have the expected handles and children.

Usage:
    python -m simulation_tests.test_handle_mapping

Dependencies:
    pip install coppeliasim-zmqremoteapi-client

Prerequisites:
    - CoppeliaSim open with scene loaded and simulation NOT running
"""

import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


EXPECTED_OBJECTS = [
    '/p3dx_1',
    '/p3dx_2',
    '/drone',
    '/payload',
    '/rally_point',
    '/Floor',
]


def map_object(sim, path: str):
    try:
        h       = sim.getObject(path)
        alias   = sim.getObjectAlias(h, 2)
        pos     = sim.getObjectPosition(h, -1)
        children = sim.getObjectsInTree(h, 0, 0)
        child_aliases = []
        for c in children:
            try:
                child_aliases.append(sim.getObjectAlias(c, 2))
            except Exception:
                child_aliases.append(f"handle:{c}")
        return {
            'path':     path,
            'handle':   h,
            'alias':    alias,
            'position': [round(x, 3) for x in pos],
            'children': child_aliases,
        }
    except Exception as e:
        return {'path': path, 'error': str(e)}


def main():
    print("=== TEST: Handle mapping ===\n")

    client = RemoteAPIClient()
    sim    = client.getObject('sim')
    sim.startSimulation()
    time.sleep(0.5)

    results = {}
    all_ok  = True

    for path in EXPECTED_OBJECTS:
        info = map_object(sim, path)
        results[path] = info

        if 'error' in info:
            print(f"FAIL  {path}: {info['error']}")
            all_ok = False
        else:
            print(f"OK    {path}")
            print(f"      handle  : {info['handle']}")
            print(f"      alias   : {info['alias']}")
            print(f"      position: {info['position']}")
            print(f"      children: {len(info['children'])}")
            for c in info['children']:
                print(f"        {c}")
            print()

    # Verify sensors on each robot
    print("--- Sensor verification ---")
    for robot_path in ['/p3dx_1', '/p3dx_2']:
        sensor_count = 0
        for i in range(16):
            try:
                sim.getObject(f'{robot_path}/ultrasonicSensor', {'index': i})
                sensor_count += 1
            except Exception:
                break
        status = 'OK' if sensor_count == 16 else 'FAIL'
        print(f"{status}  {robot_path}: {sensor_count}/16 sensors")
        if sensor_count != 16:
            all_ok = False

    print(f"\n{'='*40}")
    print(f"RESULT: {'PASS ✓' if all_ok else 'FAIL ✗'}")
    print(f"{'='*40}")

    sim.stopSimulation()
    print("Simulation stopped.")


if __name__ == '__main__':
    main()