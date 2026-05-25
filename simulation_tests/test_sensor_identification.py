"""
test_sensor_identification.py

Maps the full sensor layout of /p3dx_1 by spinning the robot slowly
and recording which sensors activate as it faces known walls.

For each sensor records:
  - Activation angle (robot yaw when sensor first detects)
  - Distance at activation
  - Estimated sensor direction relative to robot forward axis

Usage:
    python -m simulation_tests.test_sensor_identification

Dependencies:
    pip install coppeliasim-zmqremoteapi-client

Prerequisites:
    - CoppeliaSim open with scene loaded and simulation NOT running
    - /p3dx_1 placed in open space away from walls (center of arena)
      so sensor activations are unambiguous
"""

import time
import math
import csv
import os
from datetime import datetime
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from src.robot import Robot


LOG_DIR    = 'simulation_tests/logs'
RESULT_DIR = 'simulation_tests/results'


def main():
    print("=== TEST: Sensor identification — full layout mapping ===\n")

    client = RemoteAPIClient()
    sim    = client.getObject('sim')

    os.makedirs(LOG_DIR,    exist_ok=True)
    os.makedirs(RESULT_DIR, exist_ok=True)
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')

    # Place robot at center for unambiguous readings
    robot  = Robot(sim, '/p3dx_1', name='1')
    sim.startSimulation()
    time.sleep(0.5)

    print(f"Robot position: {[round(x,3) for x in robot.get_position()[:2]]}")
    print(f"Robot yaw:      {round(math.degrees(robot.get_yaw()), 1)}°\n")
    print("Spinning robot 360° slowly, recording sensor activations...\n")

    # Track first activation per sensor
    activations  = {}   # sensor_idx → {yaw, dist}
    all_readings = []

    # Spin slowly for one full rotation
    spin_speed  = 0.3   # rad/s — slow enough to catch all activations
    spin_time   = 25.0  # seconds — enough for ~360° at 0.3 rad/s
    step        = 0.05

    left_motor  = sim.getObject('/p3dx_1/leftMotor')
    right_motor = sim.getObject('/p3dx_1/rightMotor')
    sim.setJointTargetVelocity(left_motor,   spin_speed)
    sim.setJointTargetVelocity(right_motor, -spin_speed)

    print(f"{'S':>3} {'Yaw':>8} {'Dist':>8} {'Dir_from_fwd':>15}")
    print("-" * 40)

    t_start = time.time()
    while time.time() - t_start < spin_time:
        yaw_rad = robot.get_yaw()
        yaw_deg = math.degrees(yaw_rad)
        readings = robot.read_sensors_legacy()

        for i, (detected, dist) in enumerate(readings):
            if detected and dist < 0.9:
                # Record first activation only
                if i not in activations:
                    activations[i] = {
                        'yaw_deg': round(yaw_deg, 1),
                        'dist':    round(dist, 3)
                    }
                    print(f"[{i:>2}] {yaw_deg:>7.1f}° {dist:>7.3f}m  first activation")

                all_readings.append({
                    'time':    round(time.time() - t_start, 2),
                    'yaw_deg': round(yaw_deg, 1),
                    'sensor':  i,
                    'dist':    round(dist, 3)
                })

        time.sleep(step)

    sim.setJointTargetVelocity(left_motor,  0)
    sim.setJointTargetVelocity(right_motor, 0)

    # Compute relative angle for each sensor
    # The sensor that activates at a given yaw points in the direction
    # the robot was facing when it detected — relative to initial forward
    print(f"\n{'='*55}")
    print(f"SENSOR MAP (relative to robot forward axis = 0°)")
    print(f"{'='*55}")
    print(f"{'S':>3} {'First_yaw':>10} {'Dist':>8} {'Notes':>20}")
    print("-" * 55)

    for i in sorted(activations.keys()):
        a = activations[i]
        # Sensors 0-7: frontal half, 8-15: rear half (approximate)
        half = "frontal" if i < 8 else "rear"
        print(f"[{i:>2}] {a['yaw_deg']:>9.1f}° {a['dist']:>7.3f}m  {half}")

    # Save CSV
    log_path = os.path.join(LOG_DIR, f'sensor_identification_{ts}.csv')
    with open(log_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['time','yaw_deg','sensor','dist'])
        writer.writeheader()
        writer.writerows(all_readings)

    # Save result
    result_path = os.path.join(RESULT_DIR, 'test_sensor_identification_result.md')
    with open(result_path, 'w') as f:
        f.write(f"# Result: test_sensor_identification.py\n\n")
        f.write(f"**Date:** {datetime.now().strftime('%Y-%m-%d')}\n\n")
        f.write(f"## Sensor activation map\n\n")
        f.write(f"| Sensor | First activation yaw | Distance | Half |\n")
        f.write(f"|--------|---------------------|----------|------|\n")
        for i in sorted(activations.keys()):
            a = activations[i]
            half = "frontal" if i < 8 else "rear"
            f.write(f"| [{i}] | {a['yaw_deg']}° | {a['dist']}m | {half} |\n")
        f.write(f"\n## Log\n`{log_path}`\n")

    print(f"\nResult: {result_path}")
    print(f"Log:    {log_path}")

    sim.stopSimulation()
    print("Simulation stopped.")


if __name__ == '__main__':
    main()