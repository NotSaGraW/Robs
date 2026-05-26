"""
test_push_alignment.py v4

Tests full push sequence:
  1. Approach payload
  2. Detect physical contact (checkCollision)
  3. Assess alignment (log_ratio, centroid)
  4. If aligned → push toward rally_point for 5 seconds
  5. If misaligned → back off + reorient

Usage:
    python -m simulation_tests.test_push_alignment
"""

import time
import math
import statistics
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from src.robot import Robot


ROOT_HANDLES = {
    15: 'p3dx_1', 99: 'p3dx_2', 141: 'drone',
    98: 'payload', 13: 'floor', 97: 'rally_point',
}

FRONTAL_ANGLES   = [90, 50, 30, 10, -10, -30, -50, -90]
STABILITY_WINDOW = 10
STABILITY_THR    = 0.5
FACE_LOG_MAX     = 0.05
FACE_ACTIVE_MIN  = 5
PUSH_DURATION    = 8.0   # seconds to push once aligned


def identify(sim, handle):
    if handle in ROOT_HANDLES:
        return ROOT_HANDLES[handle]
    current = handle
    while current != -1:
        try:
            parent = sim.getObjectParent(current)
        except Exception:
            break
        if parent in ROOT_HANDLES:
            return ROOT_HANDLES[parent]
        if parent == -1:
            break
        current = parent
    return 'wall'


def contact_metrics(sim, robot):
    readings = robot.read_sensors_legacy()
    total_w = weighted_angle = 0.0
    active = 0
    d_by_sensor = {}

    for i in range(8):
        det, d = readings[i]
        if not det or d >= 1.0:
            continue
        try:
            res = sim.readProximitySensor(robot.sensors[i])
            if res[0] > 0 and identify(sim, int(res[3])) == 'payload':
                w = 1.0 / max(d, 0.10)
                weighted_angle += FRONTAL_ANGLES[i] * w
                total_w += w
                active += 1
                d_by_sensor[i] = d
        except Exception:
            pass

    centroid = weighted_angle / total_w if total_w > 1e-6 else None

    d3 = d_by_sensor.get(3)
    d4 = d_by_sensor.get(4)
    if d3 and d4 and d3 > 0 and d4 > 0:
        log_ratio  = math.log(d3 / d4)
        norm_delta = (d3 - d4) / max(d3, d4)
    else:
        log_ratio = norm_delta = None

    return {
        'centroid':   centroid,
        'log_ratio':  log_ratio,
        'norm_delta': norm_delta,
        'active':     active,
        'sensors':    d_by_sensor,
    }


def main():
    print("=== TEST: Push alignment v4 — approach + align + push ===\n")

    client      = RemoteAPIClient()
    sim         = client.getObject('sim')
    sim.startSimulation()
    time.sleep(0.5)

    robot       = Robot(sim, '/p3dx_1', name='1')
    payload_h   = sim.getObject('/payload')
    rally_h     = sim.getObject('/rally_point')
    payload_pos = sim.getObjectPosition(payload_h, -1)
    rally_pos   = sim.getObjectPosition(rally_h, -1)

    print(f"Robot:        {[round(x,3) for x in robot.get_position()[:2]]}")
    print(f"Payload:      {[round(x,3) for x in payload_pos[:2]]}")
    print(f"Rally point:  {[round(x,3) for x in rally_pos[:2]]}\n")

    print(f"{'t':>5} {'contact':>8} {'aligned':>8} {'action':>14} "
          f"{'log_r':>7} {'active':>7}  payload_pos")
    print("-" * 85)

    centroid_history = []
    push_start       = None
    # States: APPROACH, BACKOFF, REORIENT, PUSH, DONE
    state = 'APPROACH'

    t_start = time.time()
    while time.time() - t_start < 40.0:

        # Physical contact
        try:
            result, _ = sim.checkCollision(robot.handle, payload_h)
            in_contact = result == 1
        except Exception:
            in_contact = False

        # Metrics
        m = contact_metrics(sim, robot)
        if m['centroid'] is not None:
            centroid_history.append(m['centroid'])
            if len(centroid_history) > STABILITY_WINDOW:
                centroid_history.pop(0)

        stable = False
        if len(centroid_history) >= STABILITY_WINDOW and m['active'] >= 3:
            std = statistics.stdev(centroid_history)
            stable = std < STABILITY_THR

        aligned = (
            stable
            and m['log_ratio'] is not None
            and abs(m['log_ratio']) < FACE_LOG_MAX
            and m['active'] >= FACE_ACTIVE_MIN
        )

        # Current payload position
        cur_payload = sim.getObjectPosition(payload_h, -1)

        # --- State machine ---
        if state == 'APPROACH':
            if in_contact and aligned:
                state = 'PUSH'
                push_start = time.time()
                action = 'PUSH_START'
            elif in_contact and not aligned:
                state = 'BACKOFF'
                centroid_history.clear()
                action = 'BACKOFF'
                robot.move_backward(0.6)
            else:
                readings = robot.read_sensors_legacy()
                d = [r[1] if r[0] else 99.0 for r in readings]
                front_min = min(d[2], d[3], d[4], d[5])
                speed = max(0.3, min(1.5, front_min * 1.5))
                robot.drive_to(payload_pos, speed)
                action = f'approach({speed:.2f})'

        elif state == 'BACKOFF':
            # Reverse until no contact
            if not in_contact:
                state = 'REORIENT'
                action = 'REORIENT'
            else:
                robot.move_backward(0.6)
                action = 'backing'

        elif state == 'REORIENT':
            # Adjust orientation based on last log_ratio
            lr = m['log_ratio'] if m['log_ratio'] is not None else 0
            if abs(lr) < FACE_LOG_MAX:
                state = 'APPROACH'
                action = 'realigned'
            elif lr > 0:
                robot.turn_left(0.5)
                action = f'turn_left(lr={lr:+.2f})'
            else:
                robot.turn_right(0.5)
                action = f'turn_right(lr={lr:+.2f})'

        elif state == 'PUSH':
            elapsed_push = time.time() - push_start
            if elapsed_push > PUSH_DURATION:
                state = 'DONE'
                robot.stop()
                action = 'DONE'
            else:
                robot.drive_to(rally_pos, 1.2)
                action = f'pushing({elapsed_push:.1f}s)'

        elif state == 'DONE':
            robot.stop()
            action = 'DONE'

        # Log
        t = time.time() - t_start
        logr_s = f"{m['log_ratio']:+6.3f}" if m['log_ratio'] is not None else "   ---"
        pay_s  = f"({cur_payload[0]:.3f},{cur_payload[1]:.3f})"

        print(f"{t:5.1f} {'YES' if in_contact else 'no':>8} "
              f"{'YES' if aligned else 'no':>8} {action:>14} "
              f"{logr_s:>7} {m['active']:>7}  {pay_s}")

        if state == 'DONE' and time.time() - t_start > 35.0:
            break

        time.sleep(0.05)

    robot.stop()
    sim.stopSimulation()
    print("\nSimulation stopped.")


if __name__ == '__main__':
    main()