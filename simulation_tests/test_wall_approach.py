"""
test_wall_approach.py

Full wall acquisition sequence:
  1. SCAN    — spin 360° in place, build distance map at legacy range
  2. ORIENT  — rotate to put nearest obstacle on right side
  3. APPROACH — reactive: advance keeping obstacle on right,
                turn left if frontal blocked, turn right if lost
                until [7]+[8] both detect at <= FOLLOW_DIST
  4. FOLLOW  — PID wall following

Usage:
    python -m simulation_tests.test_wall_approach

Dependencies:
    pip install coppeliasim-zmqremoteapi-client

Prerequisites:
    - CoppeliaSim open with scene loaded and simulation NOT running
"""

import time
import math
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from src.robot import Robot


SPIN_SPEED      = 0.4
SCAN_DURATION   = 17.0
ORIENT_SPEED    = 0.4
ORIENT_TOL      = 0.08
APPROACH_SPEED  = 60     # deg/s — slow enough for sensor readings
FOLLOW_DIST     = 0.35   # metres — switch to PID when wall this close on lateral
FRONT_STOP      = 0.35   # metres — frontal obstacle, turn left
WALL_FOLLOW_SPD = 120
WALL_DISTANCE   = 0.25
TS              = 0.05
KP_ROT          = 200
KD_ROT          = 10
KI_ROT          = 30
KP_TRANS        = 90
HYSTERESIS      = 0.15
CONFIRM_CYCLES  = 8


def main():
    print("=== TEST: Wall approach + follow — /p3dx_1 ===\n")

    client = RemoteAPIClient()
    sim    = client.getObject('sim')
    sim.startSimulation()
    time.sleep(0.5)

    robot = Robot(sim, '/p3dx_1', name='1')
    left_motor  = sim.getObject('/p3dx_1/leftMotor')
    right_motor = sim.getObject('/p3dx_1/rightMotor')

    # ----------------------------------------------------------------
    # PHASE 1 — SCAN
    # ----------------------------------------------------------------
    print("Phase 1: SCAN — spinning 360°...")

    distance_map = [float('inf')] * 16
    yaw_map      = [None] * 16

    sim.setJointTargetVelocity(left_motor,   SPIN_SPEED)
    sim.setJointTargetVelocity(right_motor, -SPIN_SPEED)

    t_scan = time.time()
    while time.time() - t_scan < SCAN_DURATION:
        yaw      = robot.get_yaw()
        readings = robot.read_sensors_legacy()
        for i, (det, dist) in enumerate(readings):
            if det and dist < distance_map[i]:
                distance_map[i] = dist
                yaw_map[i]      = yaw
        time.sleep(TS)

    sim.setJointTargetVelocity(left_motor,  0)
    sim.setJointTargetVelocity(right_motor, 0)
    time.sleep(0.3)

    print("\nScan result:")
    for i in range(16):
        if distance_map[i] < float('inf'):
            print(f"  [{i:>2}] {distance_map[i]:.3f}m  "
                  f"yaw={math.degrees(yaw_map[i]):.1f}°")

    # ----------------------------------------------------------------
    # PHASE 2 — ORIENT: rotate so nearest obstacle is on right [7]
    # ----------------------------------------------------------------
    print("\nPhase 2: ORIENT...")

    # Find yaw where right-side sensors detected nearest obstacle
    best_dist = float('inf')
    best_yaw  = None
    for i in [5, 6, 7, 8]:
        if distance_map[i] < best_dist:
            best_dist = distance_map[i]
            best_yaw  = yaw_map[i]

    if best_yaw is None:
        for i in range(16):
            if distance_map[i] < best_dist:
                best_dist = distance_map[i]
                best_yaw  = yaw_map[i]

    if best_yaw is not None:
        print(f"  Target yaw: {math.degrees(best_yaw):.1f}°  "
              f"(obstacle at {best_dist:.3f}m)")
        t_orient = time.time()
        while time.time() - t_orient < 15.0:
            current_yaw = robot.get_yaw()
            error = math.atan2(
                math.sin(best_yaw - current_yaw),
                math.cos(best_yaw - current_yaw)
            )
            if abs(error) < ORIENT_TOL:
                break
            if error > 0:
                sim.setJointTargetVelocity(left_motor,  -ORIENT_SPEED)
                sim.setJointTargetVelocity(right_motor,  ORIENT_SPEED)
            else:
                sim.setJointTargetVelocity(left_motor,   ORIENT_SPEED)
                sim.setJointTargetVelocity(right_motor, -ORIENT_SPEED)
            time.sleep(TS)

        sim.setJointTargetVelocity(left_motor,  0)
        sim.setJointTargetVelocity(right_motor, 0)
        time.sleep(0.3)
        print(f"  Oriented. Yaw: {math.degrees(robot.get_yaw()):.1f}°")

    # ----------------------------------------------------------------
    # PHASE 3 — APPROACH: reactive wall acquisition
    # ----------------------------------------------------------------
    print("\nPhase 3: APPROACH...")

    phase          = 'approach'
    current_side   = None
    sum_rot_error  = 0
    last_rot_error = 0
    votes_right    = 0
    votes_left     = 0

    print(f"\n{'phase':>8} {'side':>6} {'front':>7} "
          f"{'right':>7} {'left':>7}")
    print("-" * 44)

    t_start = time.time()
    while time.time() - t_start < 60:

        legacy    = robot.read_sensors_legacy()
        distances = [d if det else 1.0 for det, d in legacy]

        # Key distances
        front    = min(distances[3], distances[4],
                       distances[9], distances[10])
        right_d  = min(distances[5], distances[6],
                       distances[7], distances[8])
        left_d   = min(distances[0], distances[1],
                       distances[2], distances[15])

        if phase == 'approach':
            if front < FRONT_STOP:
                # Frontal obstacle — turn left
                robot.set_velocity(-APPROACH_SPEED * math.pi/180, APPROACH_SPEED * math.pi/180)
            elif right_d < 0.6:
                # Obstacle on right — advance straight, wall is where we want it
                robot.set_velocity(APPROACH_SPEED * math.pi/180, APPROACH_SPEED * math.pi/180)
            else:
                # No obstacle on right — curve right to find wall
                robot.set_velocity(APPROACH_SPEED * 1.3 * math.pi/180, APPROACH_SPEED * 0.7 * math.pi/180)

            # Check acquisition — lateral sensors close enough
            dist_7  = distances[7]
            dist_8  = distances[8]
            dist_0  = distances[0]
            dist_15 = distances[15]

            if dist_7 <= FOLLOW_DIST and dist_8 <= FOLLOW_DIST:
                current_side   = 'right'
                phase          = 'follow'
                sum_rot_error  = 0
                last_rot_error = 0
                print(f"\n>>> RIGHT acquired d7={dist_7:.2f} "
                      f"d8={dist_8:.2f} → FOLLOW\n")
            elif dist_0 <= FOLLOW_DIST and dist_15 <= FOLLOW_DIST:
                current_side   = 'left'
                phase          = 'follow'
                sum_rot_error  = 0
                last_rot_error = 0
                print(f"\n>>> LEFT acquired d0={dist_0:.2f} "
                      f"d15={dist_15:.2f} → FOLLOW\n")

            print(f"{'approach':>8} {'---':>6} {front:>7.2f} "
                  f"{right_d:>7.2f} {left_d:>7.2f}")

        else:  # follow
            right_dist = min(distances[7], distances[8])
            left_dist  = min(distances[0], distances[15])

            if right_dist < left_dist - HYSTERESIS:
                votes_right += 1; votes_left = 0
            elif left_dist < right_dist - HYSTERESIS:
                votes_left += 1; votes_right = 0
            else:
                votes_right = max(0, votes_right - 1)
                votes_left  = max(0, votes_left  - 1)

            if votes_right >= CONFIRM_CYCLES:
                current_side = 'right'
            elif votes_left >= CONFIRM_CYCLES:
                current_side = 'left'

            if current_side == 'right':
                wall_front = distances[7]
                wall_back  = distances[8]
            else:
                wall_front = distances[0]
                wall_back  = distances[15]

            rot_error      = wall_front - wall_back
            sum_rot_error += rot_error
            pid_rot = (KP_ROT * rot_error +
                       KI_ROT * sum_rot_error * TS +
                       KD_ROT * (rot_error - last_rot_error) / TS)
            last_rot_error = rot_error

            trans_error = wall_front - WALL_DISTANCE
            pid_trans   = KP_TRANS * trans_error

            if current_side == 'right':
                vL = WALL_FOLLOW_SPD + pid_rot + pid_trans
                vR = WALL_FOLLOW_SPD - pid_rot - pid_trans
            else:
                vL = WALL_FOLLOW_SPD - pid_rot - pid_trans
                vR = WALL_FOLLOW_SPD + pid_rot + pid_trans

            robot.set_velocity(vL * math.pi/180, vR * math.pi/180)

            if front < 0.35:
                robot.set_velocity(-WALL_FOLLOW_SPD * math.pi/180, WALL_FOLLOW_SPD * math.pi/180)
                time.sleep(0.2)

            print(f"{'follow':>8} {current_side:>6} {front:>7.2f} "
                  f"{wall_front:>7.2f} {wall_back:>7.2f}")

        time.sleep(TS)

    robot.stop()
    sim.stopSimulation()
    print("\nSimulation stopped.")


if __name__ == '__main__':
    main()