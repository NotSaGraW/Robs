"""
test_sensor_compare.py

Diagnostic — compares readProximitySensor (legacy) vs checkProximitySensorEx
for all 16 sensors with robot stationary.

Run with robot near a wall to see which sensors detect it and at what distance.

Usage:
    python -m simulation_tests.test_sensor_compare

Prerequisites:
    - CoppeliaSim open with scene loaded and simulation NOT running
    - /p3dx_1 stationary near a wall
"""

import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from src.robot import Robot


def main():
    print("=== DIAGNOSTIC: Sensor comparison — legacy vs checkProximitySensorEx ===\n")

    client = RemoteAPIClient()
    sim    = client.getObject('sim')
    sim.startSimulation()
    time.sleep(0.5)

    robot = Robot(sim, '/p3dx_1', name='1')

    print(f"Robot position: {robot.get_position()[:2]}")
    print(f"Robot yaw:      {round(robot.get_yaw(), 3)} rad\n")

    legacy   = robot.read_sensors_legacy()
    new_05   = robot.read_sensors(range(16), robot.RANGE_NORMAL)
    new_10   = robot.read_sensors(range(16), robot.RANGE_EXTENDED)

    print(f"{'S':>3} {'legacy_det':>10} {'legacy_d':>10} "
          f"{'new_0.5_det':>12} {'new_0.5_d':>10} "
          f"{'new_1.0_det':>12} {'new_1.0_d':>10}")
    print("-" * 75)

    for i in range(16):
        l_det, l_d   = legacy[i]
        n05_det, n05_d, _ = new_05[i]
        n10_det, n10_d, _ = new_10[i]

        l_d_str   = f"{l_d:.3f}"   if l_det   else "---"
        n05_d_str = f"{n05_d:.3f}" if n05_det else "---"
        n10_d_str = f"{n10_d:.3f}" if n10_det else "---"

        # Mark sensors that disagree between methods
        flag = " ←" if (l_det != n05_det) else ""

        print(f"[{i:>2}] {str(l_det):>10} {l_d_str:>10} "
              f"{str(n05_det):>12} {n05_d_str:>10} "
              f"{str(n10_det):>12} {n10_d_str:>10}{flag}")

    sim.stopSimulation()
    print("\nSimulation stopped.")


if __name__ == '__main__':
    main()