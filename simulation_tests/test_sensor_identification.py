"""
test_sensor_identification.py

Verifies that readProximitySensor returns the detected object handle,
allowing the system to distinguish between box, walls, and other robots.

Prerequisites:
    - CoppeliaSim open with scene loaded
    - /box is Detectable (confirmed)
    - /roba and /robo Lua scripts: sysCall_init active, sysCall_actuation commented

What this test verifies:
    1. readProximitySensor returns a valid object handle on detection
    2. The handle can be matched to /box, /Floor, walls, or other robot
    3. Sensors [3,4] detect the box when robot faces it
    4. Forward axis of Pioneer in world coordinates

Procedure:
    - Phase 1: robot A stationary, reads all 16 sensors, logs handles
    - Phase 2: robot A rotates slowly 360°, logs all detections with handles
    - Phase 3: robot A advances toward box, logs contact detection by handle

Log output: experiments/logs/sensor_identification_<timestamp>.csv
"""

import time
import math
import csv
import os
from datetime import datetime
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


# --- Configuration ---
LOG_DIR      = 'experiments/logs'
ROTATE_SPEED = 0.5    # rad/s — slow rotation for clean sensor sweep
ADVANCE_SPEED= 1.0    # rad/s — slow advance toward box
SCAN_DURATION= 12.0   # seconds for full 360° scan at ROTATE_SPEED
STEP         = 0.05   # seconds per cycle


def get_timestamp():
    return datetime.now().strftime('%Y%m%d_%H%M%S')


def label_handle(handle, known_handles):
    """Returns a human-readable label for a detected object handle."""
    for name, h in known_handles.items():
        if h == handle:
            return name
    return f'unknown({handle})'


def main():
    print("=== TEST: Sensor Identification ===\n")

    # --- Connect ---
    client = RemoteAPIClient()
    sim    = client.getObject('sim')

    # --- Get object handles ---
    roba_handle  = sim.getObject('/roba')
    robo_handle  = sim.getObject('/robo')
    box_handle   = sim.getObject('/box')
    floor_handle = sim.getObject('/Floor')

    # Get wall handles (there are multiple — get as many as exist)
    wall_handles = {}
    for i in range(20):
        try:
            h = sim.getObject(f'/20cmHighWall100cm[{i}]')
            wall_handles[f'wall[{i}]'] = h
        except Exception:
            break

    # Build known handles dictionary for labelling
    known_handles = {
        '/roba':  roba_handle,
        '/robo':  robo_handle,
        '/box':   box_handle,
        '/Floor': floor_handle,
    }
    known_handles.update(wall_handles)

    print(f"Known objects:")
    for name, h in known_handles.items():
        print(f"  {name}: handle={h}")

    # --- Get sensor handles for robot A ---
    sensor_handles = []
    for i in range(16):
        try:
            h = sim.getObject('/roba/ultrasonicSensor', {'index': i})
            sensor_handles.append(h)
        except Exception:
            sensor_handles.append(None)
    print(f"\nSensors loaded: {sum(1 for h in sensor_handles if h is not None)}/16")

    # Get motor handles
    left_motor  = sim.getObject('/roba/leftMotor')
    right_motor = sim.getObject('/roba/rightMotor')

    # --- Prepare log file ---
    os.makedirs(LOG_DIR, exist_ok=True)
    log_path = os.path.join(LOG_DIR, f'sensor_identification_{get_timestamp()}.csv')

    fieldnames = [
        'timestamp', 'phase', 'sim_time',
        'robot_x', 'robot_y', 'robot_yaw',
        'sensor_index', 'detected', 'distance',
        'detected_handle', 'detected_label'
    ]

    print(f"\nLog file: {log_path}")
    print("\nStarting simulation...\n")

    sim.startSimulation()
    t_start = time.time()

    with open(log_path, 'w', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()

        def log_sensors(phase):
            """Read all 16 sensors and write one row per detection."""
            pos = sim.getObjectPosition(roba_handle, -1)
            ori = sim.getObjectOrientation(roba_handle, -1)
            yaw = math.degrees(ori[2])
            sim_time = sim.getSimulationTime()

            detections = []
            for i, sh in enumerate(sensor_handles):
                if sh is None:
                    continue
                result   = sim.readProximitySensor(sh)
                state    = result[0]
                dist     = result[1]
                det_handle = result[3] if state > 0 else -1
                det_label  = label_handle(det_handle, known_handles) if state > 0 else 'none'

                row = {
                    'timestamp':      time.time() - t_start,
                    'phase':          phase,
                    'sim_time':       sim_time,
                    'robot_x':        round(pos[0], 4),
                    'robot_y':        round(pos[1], 4),
                    'robot_yaw':      round(yaw, 2),
                    'sensor_index':   i,
                    'detected':       1 if state > 0 else 0,
                    'distance':       round(dist, 4) if state > 0 else '',
                    'detected_handle':det_handle,
                    'detected_label': det_label,
                }
                writer.writerow(row)

                if state > 0:
                    detections.append(
                        f"  sensor[{i:2d}] → {det_label} at {dist:.3f}m"
                    )

            return detections

        # ----------------------------------------------------------------
        # PHASE 1: Stationary scan
        # Read all sensors once with robot stationary.
        # Tells us what is visible from the initial position.
        # ----------------------------------------------------------------
        print("--- Phase 1: Stationary scan ---")
        sim.setJointTargetVelocity(left_motor, 0)
        sim.setJointTargetVelocity(right_motor, 0)
        time.sleep(0.5)

        detections = log_sensors('stationary')
        pos = sim.getObjectPosition(roba_handle, -1)
        ori = sim.getObjectOrientation(roba_handle, -1)
        print(f"Robot A position: ({pos[0]:.3f}, {pos[1]:.3f})")
        print(f"Robot A yaw: {math.degrees(ori[2]):.1f}°")
        if detections:
            print("Detections:")
            for d in detections:
                print(d)
        else:
            print("No detections from initial position")

        # ----------------------------------------------------------------
        # PHASE 2: 360° rotation scan
        # Robot rotates slowly, logging all sensor detections.
        # Tells us the real forward axis and full sensor map.
        # ----------------------------------------------------------------
        print(f"\n--- Phase 2: 360° rotation scan ({SCAN_DURATION}s) ---")
        sim.setJointTargetVelocity(left_motor,  ROTATE_SPEED)
        sim.setJointTargetVelocity(right_motor, -ROTATE_SPEED)

        t_phase = time.time()
        cycle   = 0
        while time.time() - t_phase < SCAN_DURATION:
            detections = log_sensors('rotation_360')
            if cycle % 20 == 0:
                pos = sim.getObjectPosition(roba_handle, -1)
                ori = sim.getObjectOrientation(roba_handle, -1)
                elapsed = time.time() - t_phase
                print(f"  t={elapsed:.1f}s yaw={math.degrees(ori[2]):.1f}°", end='')
                if detections:
                    print(f" → {len(detections)} detection(s):")
                    for d in detections:
                        print(f"    {d}")
                else:
                    print(" → no detections")
            time.sleep(STEP)
            cycle += 1

        sim.setJointTargetVelocity(left_motor, 0)
        sim.setJointTargetVelocity(right_motor, 0)
        time.sleep(0.3)

        # ----------------------------------------------------------------
        # PHASE 3: Advance toward box
        # Robot moves toward box origin (0,0) from its position.
        # Logs which sensor detects box first and at what distance.
        # ----------------------------------------------------------------
        print("\n--- Phase 3: Advance toward box ---")

        # First orient robot toward box
        pos = sim.getObjectPosition(roba_handle, -1)
        box_pos = sim.getObjectPosition(box_handle, -1)
        angle_to_box = math.atan2(
            box_pos[1] - pos[1],
            box_pos[0] - pos[0]
        )
        print(f"Box position: ({box_pos[0]:.3f}, {box_pos[1]:.3f})")
        print(f"Angle to box: {math.degrees(angle_to_box):.1f}°")
        print("Advancing toward box (15s max or until contact)...")

        sim.setJointTargetVelocity(left_motor,  ADVANCE_SPEED)
        sim.setJointTargetVelocity(right_motor, ADVANCE_SPEED)

        box_detected_by = None
        t_phase = time.time()
        while time.time() - t_phase < 15.0:
            detections = log_sensors('advance_to_box')

            # Check specifically for box detection
            for i, sh in enumerate(sensor_handles):
                if sh is None:
                    continue
                result = sim.readProximitySensor(sh)
                if result[0] > 0 and result[3] == box_handle:
                    if box_detected_by is None:
                        box_detected_by = i
                        dist = result[1]
                        pos  = sim.getObjectPosition(roba_handle, -1)
                        ori  = sim.getObjectOrientation(roba_handle, -1)
                        print(f"\n  *** BOX DETECTED by sensor[{i}] ***")
                        print(f"      Distance: {dist:.3f}m")
                        print(f"      Robot yaw: {math.degrees(ori[2]):.1f}°")
                        print(f"      Robot pos: ({pos[0]:.3f}, {pos[1]:.3f})")

            time.sleep(STEP)

        sim.setJointTargetVelocity(left_motor, 0)
        sim.setJointTargetVelocity(right_motor, 0)

        if box_detected_by is None:
            print("\n  Box not detected during advance phase")
            print("  Check that /box has Detectable enabled")

    # --- Summary ---
    print(f"\n=== Test complete ===")
    print(f"Log saved to: {log_path}")
    print(f"\nKey findings to check in the log:")
    print(f"  1. Which sensor index first detects /box")
    print(f"  2. Robot yaw when box is detected → forward axis")
    print(f"  3. Whether /Floor and walls return different handles")
    print(f"  4. Whether /robo handle appears in any sensor reading")

    sim.stopSimulation()
    print("\nSimulation stopped.")


if __name__ == '__main__':
    main()