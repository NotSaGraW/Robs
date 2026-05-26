"""
test_two_robots.py — v2

P1: longitudinal pusher (GT direction + sensor centering)
P2: yaw stabilizer — cancels payload rotation, not a pusher

P2 control signal: yaw_error between payload movement direction
and ideal direction (payload→rally). F_p2 ∝ -yaw_error in perp axis.

P2 enters PUSH only with confirmed side contact AND alignment.
P2 target frozen (like v7) to avoid chasing moving reference.
PUSH_SPD_P2 = 0.2 (small — canceling rotation, not pushing)

Run:
    python -m simulation_tests.test_two_robots
"""

import math
import collections
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from src.robot import Robot
from src import scene


# ── Geometry ──────────────────────────────────────────────────────
PAYLOAD_HALF          = 0.25
SENSOR_FORWARD_OFFSET = 0.209
PUSH_DIST             = PAYLOAD_HALF + SENSOR_FORWARD_OFFSET + 0.03
LATERAL_DIST          = PAYLOAD_HALF + SENSOR_FORWARD_OFFSET + 0.03

# ── P1 (longitudinal) ─────────────────────────────────────────────
PUSH_SPD_P1   = 2.0
K_CENTER      = 3.0

# ── P2 (yaw stabilizer) ───────────────────────────────────────────
PUSH_SPD_P2   = 0.2     # small — canceling rotation, not pushing
K_YAW         = 2.0     # yaw error gain
YAW_WINDOW    = 8       # steps for velocity smoothing
YAW_THR       = 2.0     # degrees — P2 only acts above this drift threshold
P2_CONTACT    = 0.40    # m — side sensor threshold for contact

# ── Navigation ────────────────────────────────────────────────────
APPROACH_SPD  = 2.0
POSITION_THR  = 0.15

# ── Sensor thresholds ─────────────────────────────────────────────
DETECT_DIST   = 0.80

# ── P1 progress monitor ───────────────────────────────────────────
EMA_ALPHA     = 0.15
MIN_PROGRESS  = 0.0002
STALL_STEPS   = 80

MAX_STEPS     = 4000
LOG_EVERY     = 20


def read_sensor(sim, handle):
    try:
        res = sim.readProximitySensor(handle)
        if res[0] > 0:
            return float(res[1])
    except Exception:
        pass
    return float('inf')


def normalize(dx, dy):
    n = math.sqrt(dx*dx + dy*dy) + 1e-9
    return dx/n, dy/n


def dist2d(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)


def angle_diff(a, b):
    """Signed angle difference a-b in [-pi, pi]."""
    return math.atan2(math.sin(a-b), math.cos(a-b))


def push_target_p1(px, py, rally_pos):
    ux, uy = normalize(rally_pos[0]-px, rally_pos[1]-py)
    return [px - ux*PUSH_DIST, py - uy*PUSH_DIST, 0.0]


def push_target_p2(px, py, rally_pos):
    """P2 approaches from +perp side of payload."""
    ux, uy = normalize(rally_pos[0]-px, rally_pos[1]-py)
    perpx, perpy = -uy, ux
    return [px + perpx*LATERAL_DIST, py + perpy*LATERAL_DIST, 0.0]


def apply_force(robot, fx, fy, speed):
    hdg = math.atan2(fy, fx)
    yaw = robot.get_yaw()
    err = math.atan2(math.sin(hdg-yaw), math.cos(hdg-yaw))
    w   = max(-1.5, min(1.5, 3.0*err))
    robot.set_velocity(speed-w, speed+w)


def step_p1(robot, px, py, rally_pos, d3, d4, phase):
    rpos   = robot.get_position()
    ux, uy = normalize(rally_pos[0]-px, rally_pos[1]-py)
    px_p, py_p = -uy, ux

    if phase == 'SCAN':
        tgt   = push_target_p1(px, py, rally_pos)
        d_tgt = dist2d(rpos, tgt)
        if d_tgt < POSITION_THR:
            return 'PUSH'
        robot.drive_to(tgt, APPROACH_SPD)
        return 'SCAN'

    elif phase == 'PUSH':
        if d3 < DETECT_DIST and d4 < DETECT_DIST:
            center = K_CENTER * (d4 - d3)
            fx = ux + px_p * center
            fy = uy + py_p * center
        else:
            fx, fy = ux, uy
        n = math.sqrt(fx*fx + fy*fy) + 1e-9
        apply_force(robot, fx/n, fy/n, PUSH_SPD_P1)
        return 'PUSH'

    return phase


def step_p2(robot, px, py, rally_pos, d7, phase,
            frozen_tgt, yaw_error):
    """
    P2 yaw stabilizer.
    Enters PUSH only with side contact.
    Applies force proportional to yaw_error in perp direction.
    Uses frozen target to avoid chasing moving payload.
    """
    rpos = robot.get_position()
    ux, uy = normalize(rally_pos[0]-px, rally_pos[1]-py)
    perpx, perpy = -uy, ux   # +perp
    in_contact = (d7 < P2_CONTACT)

    if phase == 'SCAN':
        if in_contact:
            return 'PUSH', None   # freeze target on entry
        # Navigate to lateral position (freeze target once set)
        tgt = frozen_tgt if frozen_tgt is not None \
              else push_target_p2(px, py, rally_pos)
        d_tgt = dist2d(rpos, tgt)
        if d_tgt < POSITION_THR:
            robot.stop()
        else:
            robot.drive_to(tgt, APPROACH_SPD)
        return 'SCAN', tgt

    elif phase == 'PUSH':
        if not in_contact:
            return 'SCAN', None   # lost contact → rescan

        # Only act if drift exceeds threshold — passive otherwise
        if abs(math.degrees(yaw_error)) < YAW_THR:
            robot.stop()
            return 'PUSH', None

        # F_p2 ∝ -yaw_error in perp direction
        # cross > 0 → drifting left (CCW) → push in -perp
        # cross < 0 → drifting right (CW) → push in +perp
        force_scale = -K_YAW * yaw_error
        force_scale = max(-1.0, min(1.0, force_scale))

        # Direction: force_scale controls sign AND magnitude
        # force_scale > 0 → push in -perp
        # force_scale < 0 → push in +perp
        fx = -perpx * force_scale
        fy = -perpy * force_scale
        n  = math.sqrt(fx*fx + fy*fy) + 1e-9
        if n > 0.05:   # ignore negligible corrections
            apply_force(robot, fx/n, fy/n, PUSH_SPD_P2)
        else:
            robot.stop()
        return 'PUSH', None

    return phase, frozen_tgt


def main():
    print("=== TWO ROBOT PUSH v2 — P1 longitudinal + P2 yaw stabilizer ===\n")

    client    = RemoteAPIClient()
    sim       = client.getObject('sim')
    p1        = Robot(sim, '/p3dx_1', name='1')
    p2        = Robot(sim, '/p3dx_2', name='2')
    payload_h = sim.getObject('/payload')
    rally_h   = sim.getObject('/rally_point')

    rally_pos  = scene.get_position(sim, rally_h)
    payload_gt = scene.get_position(sim, payload_h)

    print(f"Payload GT  : {payload_gt[:2]}")
    print(f"Rally point : {rally_pos[:2]}")
    print(f"P1 start    : {p1.get_position()[:2]}")
    print(f"P2 start    : {p2.get_position()[:2]}")
    print(f"PUSH_SPD_P1={PUSH_SPD_P1}  PUSH_SPD_P2={PUSH_SPD_P2}  "
          f"K_YAW={K_YAW}\n")

    sim.setStepping(True)
    sim.startSimulation()

    # Payload velocity history (smoothed for drift estimation)
    payload_history = collections.deque(maxlen=YAW_WINDOW)

    phase_p1    = 'SCAN'
    phase_p2    = 'SCAN'
    frozen_tgt  = None
    step        = 0

    ema_progress  = 0.0
    stall_counter = 0
    prev_payload  = None


    try:
        while step < MAX_STEPS:
            sim.step()

            payload_real = scene.get_position(sim, payload_h)
            rally_pos    = scene.get_position(sim, rally_h)
            d_rally      = scene.dist2d(payload_real, rally_pos)
            px, py       = payload_real[0], payload_real[1]

            # Read sensors
            d3 = read_sensor(sim, p1.sensors[3]) if len(p1.sensors) > 3 else float('inf')
            d4 = read_sensor(sim, p1.sensors[4]) if len(p1.sensors) > 4 else float('inf')
            d7 = read_sensor(sim, p2.sensors[7]) if len(p2.sensors) > 7 else float('inf')

            # ── Lateral drift error ───────────────────────────────
            # cross product: desired_dir × actual_velocity
            # cross > 0 → drifting left of rally direction
            # cross < 0 → drifting right of rally direction
            payload_history.append((px, py))
            yaw_error = 0.0
            if len(payload_history) >= YAW_WINDOW:
                old_p = payload_history[0]
                dpx_ = px - old_p[0]
                dpy_ = py - old_p[1]
                spd  = math.sqrt(dpx_*dpx_ + dpy_*dpy_)
                if spd > 0.001:
                    # Actual velocity direction (normalized)
                    vx, vy = dpx_/spd, dpy_/spd
                    # Desired direction (payload → rally)
                    ux_, uy_ = normalize(rally_pos[0]-px, rally_pos[1]-py)
                    # 2D cross product: measures lateral drift
                    yaw_error = ux_ * vy - uy_ * vx

            # ── Success ──────────────────────────────────────────
            if scene.payload_reached_rally_point(payload_real, rally_pos):
                print(f"\n{'='*52}")
                print(f"  PAYLOAD REACHED RALLY POINT")
                print(f"  Steps    : {step}")
                print(f"  Final dist: {d_rally:.4f}m")
                print(f"{'='*52}\n")
                p1.stop(); p2.stop()
                break

            # ── P1 progress monitor ───────────────────────────────
            if phase_p1 == 'PUSH':
                if prev_payload is not None:
                    dpx_ = px - prev_payload[0]
                    dpy_ = py - prev_payload[1]
                    ux_, uy_ = normalize(rally_pos[0]-px, rally_pos[1]-py)
                    progress  = dpx_*ux_ + dpy_*uy_
                    ema_progress  = EMA_ALPHA*progress + \
                                    (1-EMA_ALPHA)*ema_progress
                    stall_counter = stall_counter+1 \
                                    if ema_progress < MIN_PROGRESS else 0
                prev_payload = (px, py)

                if stall_counter >= STALL_STEPS:
                    print(f"\n  [STALL] step={step}  "
                          f"ema={ema_progress:.5f}  d_rally={d_rally:.3f}m")
                    phase_p1     = 'SCAN'
                    phase_p2     = 'SCAN'
                    frozen_tgt   = None
                    ema_progress  = 0.0
                    stall_counter = 0
                    prev_payload  = None

            # ── Step each robot ───────────────────────────────────
            prev_p1 = phase_p1
            prev_p2 = phase_p2

            phase_p1 = step_p1(p1, px, py, rally_pos, d3, d4, phase_p1)
            phase_p2, frozen_tgt = step_p2(p2, px, py, rally_pos, d7,
                                            phase_p2, frozen_tgt, yaw_error)

            if prev_p1 != phase_p1:
                print(f"  P1 → {phase_p1}  step={step}  "
                      f"d3={d3:.3f} d4={d4:.3f}")
            if prev_p2 != phase_p2:
                print(f"  P2 → {phase_p2}  step={step}  "
                      f"d7={d7:.3f}  drift={math.degrees(math.asin(max(-1,min(1,yaw_error)))):+.1f}°")

            # ── Log ──────────────────────────────────────────────
            if step % LOG_EVERY == 0:
                r1 = p1.get_position()
                r2 = p2.get_position()
                print(f"step={step:4d}  d_rally:{d_rally:.3f}m  "
                      f"P1:[{phase_p1:4s}]({r1[0]:.2f},{r1[1]:.2f}) "
                      f"d3:{d3:.3f} d4:{d4:.3f}  "
                      f"P2:[{phase_p2:4s}]({r2[0]:.2f},{r2[1]:.2f}) "
                      f"d7:{d7:.3f}  "
                      f"drift:{math.degrees(math.asin(max(-1,min(1,yaw_error)))):+.1f}°  "
                      f"ema:{ema_progress:.5f}")

            step += 1

        if step >= MAX_STEPS:
            print(f"\nTIMEOUT ({MAX_STEPS} steps).")
            p1.stop(); p2.stop()

    except KeyboardInterrupt:
        print("\nInterrupted.")
        p1.stop(); p2.stop()
    finally:
        sim.stopSimulation()
        print("Simulation stopped.")


if __name__ == '__main__':
    main()