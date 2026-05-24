"""
test_sensor_360.py

BLOCKING PREREQUISITE — must pass before implementing the communication
architecture.

Verifies that detected world positions calculated from sensor readings
match actual wall positions (ground truth from CoppeliaSim).

Formula under test:
    detected_world_pos = sensor_world_matrix × detected_point_local

Usage:
    python -m simulation_tests.test_sensor_360

Dependencies:
    pip install coppeliasim-zmqremoteapi-client

Prerequisites:
    - CoppeliaSim open with scene loaded and simulation NOT running
    - /p3dx_1 Lua: sysCall_init active, sysCall_actuation commented out
"""

import os
import csv
import math
import time
from datetime import datetime
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


LOG_DIR         = 'simulation_tests/logs'
RESULT_DIR      = 'simulation_tests/results'
ROTATE_SPEED    = 0.4
SCAN_DURATION   = 15.0
STEP            = 0.05
ERROR_THRESHOLD = 0.05


def get_timestamp():
    return datetime.now().strftime('%Y%m%d_%H%M%S')


def transform_point(matrix, point):
    m = matrix
    x = m[0]*point[0] + m[1]*point[1] + m[2]*point[2]  + m[3]
    y = m[4]*point[0] + m[5]*point[1] + m[6]*point[2]  + m[7]
    z = m[8]*point[0] + m[9]*point[1] + m[10]*point[2] + m[11]
    return (x, y, z)


def distance_to_nearest_plane(pos, wall_bounds=2.525):
    """Distance from pos to nearest wall plane (not segment centre)."""
    dx = abs(abs(pos[0]) - wall_bounds)
    dy = abs(abs(pos[1]) - wall_bounds)
    return min(dx, dy)


def main():
    print("=== TEST: Sensor 360° — Position Calculation Verification ===\n")

    client = RemoteAPIClient()
    sim    = client.getObject('sim')

    os.makedirs(LOG_DIR,    exist_ok=True)
    os.makedirs(RESULT_DIR, exist_ok=True)
    ts = get_timestamp()

    p3dx_1_handle = sim.getObject('/p3dx_1')
    left_motor    = sim.getObject('/p3dx_1/leftMotor')
    right_motor   = sim.getObject('/p3dx_1/rightMotor')

    sensor_handles = []
    for i in range(16):
        try:
            h = sim.getObject('/p3dx_1/ultrasonicSensor', {'index': i})
            sensor_handles.append((i, h))
        except Exception:
            pass
    print(f"Sensors loaded: {len(sensor_handles)}/16")

    log_path = os.path.join(LOG_DIR, f'sensor_360_{ts}.csv')
    fieldnames = [
        'sim_time', 'robot_yaw', 'sensor_index',
        'detected_local_x', 'detected_local_y',
        'detected_world_x', 'detected_world_y',
        'distance_to_wall_plane', 'pass'
    ]

    errors_all = []

    print(f"Log: {log_path}\n")
    sim.startSimulation()
    time.sleep(0.5)

    with open(log_path, 'w', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()

        sim.setJointTargetVelocity(left_motor,   ROTATE_SPEED)
        sim.setJointTargetVelocity(right_motor,  -ROTATE_SPEED)

        t_start  = time.time()
        cycle    = 0
        n_detect = 0
        n_pass   = 0

        print(f"{'Cycle':>6} {'Yaw':>8} {'S':>3} {'WorldX':>8} "
              f"{'WorldY':>8} {'WallDist':>10} {'OK':>4}")
        print("-" * 55)

        while time.time() - t_start < SCAN_DURATION:
            sim_time = sim.getSimulationTime()
            ori      = sim.getObjectOrientation(p3dx_1_handle, -1)
            yaw_deg  = math.degrees(ori[2])

            for idx, sh in sensor_handles:
                result = sim.readProximitySensor(sh)
                if result[0] <= 0:
                    continue

                detected_local = result[2]
                matrix         = sim.getObjectMatrix(sh, -1)
                world_pt       = transform_point(matrix, detected_local)
                err            = distance_to_nearest_plane(
                                    (world_pt[0], world_pt[1]))
                passed         = err < ERROR_THRESHOLD

                n_detect += 1
                if passed:
                    n_pass += 1
                errors_all.append(err)

                writer.writerow({
                    'sim_time':              round(sim_time, 3),
                    'robot_yaw':             round(yaw_deg, 1),
                    'sensor_index':          idx,
                    'detected_local_x':      round(detected_local[0], 4),
                    'detected_local_y':      round(detected_local[1], 4),
                    'detected_world_x':      round(world_pt[0], 4),
                    'detected_world_y':      round(world_pt[1], 4),
                    'distance_to_wall_plane': round(err, 4),
                    'pass':                  1 if passed else 0,
                })

                if cycle % 40 == 0 or not passed:
                    ok_str = '✓' if passed else '✗ FAIL'
                    print(f"{cycle:>6} {yaw_deg:>7.1f}° [{idx:>2}] "
                          f"{world_pt[0]:>8.3f} {world_pt[1]:>8.3f} "
                          f"{err:>10.4f}m {ok_str}")

            time.sleep(STEP)
            cycle += 1

    sim.setJointTargetVelocity(left_motor,  0)
    sim.setJointTargetVelocity(right_motor, 0)

    pass_rate = n_pass / n_detect * 100 if n_detect else 0
    avg_error = sum(errors_all) / len(errors_all) if errors_all else 0
    max_error = max(errors_all) if errors_all else 0
    result_ok = pass_rate >= 95.0 and avg_error < ERROR_THRESHOLD

    print(f"\n{'='*50}")
    print(f"RESULT: {'PASS ✓' if result_ok else 'FAIL ✗'}")
    print(f"{'='*50}")
    print(f"Total detections : {n_detect}")
    print(f"Pass rate        : {n_pass} ({pass_rate:.1f}%)")
    print(f"Average error    : {avg_error:.4f}m")
    print(f"Max error        : {max_error:.4f}m")

    result_path = os.path.join(RESULT_DIR, 'test_sensor_360_result.md')
    with open(result_path, 'w') as f:
        f.write(f"# Result: test_sensor_360.py\n\n")
        f.write(f"**Date:** {datetime.now().strftime('%Y-%m-%d')}\n")
        f.write(f"**Result: {'PASS' if result_ok else 'FAIL'}**\n\n")
        f.write(f"## Summary\n")
        f.write(f"- Total detections: {n_detect}\n")
        f.write(f"- Pass rate (err < {ERROR_THRESHOLD}m): {pass_rate:.1f}%\n")
        f.write(f"- Average error: {avg_error:.4f}m\n")
        f.write(f"- Max error: {max_error:.4f}m\n\n")
        f.write(f"## Formula verified\n")
        f.write(f"`detected_world_pos = sensor_world_matrix × detected_point_local`\n\n")
        f.write(f"## Metric\n")
        f.write(f"Distance to nearest wall **plane** (not segment centre).\n")
        if result_ok:
            f.write(f"\nFormula accurate. Communication architecture implementation unblocked.\n")
        else:
            f.write(f"\nInvestigate: matrix transform, sensor orientation data.\n")

    print(f"\nResult : {result_path}")
    print(f"Log    : {log_path}")

    sim.stopSimulation()
    print("Simulation stopped.")


if __name__ == '__main__':
    main()