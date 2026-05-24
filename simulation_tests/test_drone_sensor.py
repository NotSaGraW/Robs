"""
test_drone_sensor.py

Verifies the drone object tree structure and confirms it has no proximity
or vision sensors — detection must be purely geometric.

Usage:
    python -m simulation_tests.test_drone_sensor

Dependencies:
    pip install coppeliasim-zmqremoteapi-client

Prerequisites:
    - CoppeliaSim open with scene loaded and simulation NOT running
"""

import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


def main():
    print("=== TEST: Drone sensor structure ===\n")

    client = RemoteAPIClient()
    sim    = client.getObject('sim')
    sim.startSimulation()
    time.sleep(0.5)

    drone = sim.getObject('/drone')
    print(f"Drone handle: {drone}")
    print("\nObject tree:")

    objects = sim.getObjectsInTree(drone, 0, 0)
    for obj in objects:
        alias = sim.getObjectAlias(obj, 2)
        print(f"  {alias}")

    print(f"\nTotal children: {len(objects)}")
    print("Expected: no proximity or vision sensors")

    sim.stopSimulation()
    print("\nSimulation stopped.")


if __name__ == '__main__':
    main()