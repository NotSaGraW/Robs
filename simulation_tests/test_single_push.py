"""
test_single_push.py — v7

Single-robot push using GT for navigation and position,
sensors [3],[4] only for lateral centering correction during push.

Phases:
  SCAN    → drive_to push_target(payload_GT, rally)
            transition when d_tgt < POSITION_THR
  PUSH    → continuous vector: GT direction + sensor centering
            reposition on stall or contact loss

getObjectPosition used for: payload position, rally position, success check.
Sensors used for: lateral centering correction (K_CENTER * (d4-d3)).

Run:
    python -m simulation_tests.test_single_push
"""

import math
import collections
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from src.robot import Robot
from src import scene


# ── Geometry ──────────────────────────────────────────────────────
PAYLOAD_HALF          = 0.25
SENSOR_FORWARD_OFFSET = 0.209
PUSH_DIST             = PAYLOAD_HALF + SENSOR_FORWARD_OFFSET + 0.03  # 0.489m

# ── Control ───────────────────────────────────────────────────────
PUSH_SPD      = 2.0
APPROACH_SPD  = 2.0
POSITION_THR  = 0.15    # m — arrival threshold
K_CENTER      = 3.0     # lateral centering gain

# ── Sensor thresholds ─────────────────────────────────────────────
DETECT_DIST   = 0.80    # m — sensor active below this

# ── Progress monitor ──────────────────────────────────────────────
EMA_ALPHA     = 0.15
MIN_PROGRESS  = 0.0002  # m/step toward rally
STALL_STEPS   = 60

# ── Repositioning ─────────────────────────────────────────────────
RETREAT_DIST  = 0.60
SAFE_DIST     = 1.20

MAX_STEPS     = 6000
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


def push_target(px, py, rally_pos):
    """Position robot center should be to push flush from behind."""
    ux, uy = normalize(rally_pos[0]-px, rally_pos[1]-py)
    return [px - ux*PUSH_DIST, py - uy*PUSH_DIST, 0.0]


def compute_push_vector(px, py, rally_pos, d3, d4):
    """
    GT-based direction + sensor lateral correction.
      F = normalize( forward + K_CENTER*(d4-d3)*perp )
    """
    ux, uy = normalize(rally_pos[0]-px, rally_pos[1]-py)
    px_perp, py_perp = -uy, ux

    if d3 < DETECT_DIST and d4 < DETECT_DIST:
        center = K_CENTER * (d4 - d3)
        fx = ux + px_perp * center
        fy = uy + py_perp * center
    else:
        fx, fy = ux, uy

    n = math.sqrt(fx*fx + fy*fy) + 1e-9
    return fx/n, fy/n


def apply_force(robot, fx, fy, speed):
    hdg = math.atan2(fy, fx)
    yaw = robot.get_yaw()
    err = math.atan2(math.sin(hdg-yaw), math.cos(hdg-yaw))
    w   = max(-1.5, min(1.5, 3.0*err))
    robot.set_velocity(speed-w, speed+w)


def safe_waypoint(px, py, rally_pos):
    ux, uy = normalize(rally_pos[0]-px, rally_pos[1]-py)
    return [px - ux*SAFE_DIST, py - uy*SAFE_DIST, 0.0]


def retreat_point(px, py, rpos):
    ux, uy = normalize(rpos[0]-px, rpos[1]-py)
    return [rpos[0]+ux*RETREAT_DIST, rpos[1]+uy*RETREAT_DIST, 0.0]


def main():
    print("=== SINGLE PUSH v7 — GT navigation + sensor centering ===\n")

    client    = RemoteAPIClient()
    sim       = client.getObject('sim')
    robot     = Robot(sim, '/p3dx_1', name='1')
    payload_h = sim.getObject('/payload')
    rally_h   = sim.getObject('/rally_point')

    rally_pos  = scene.get_position(sim, rally_h)
    payload_gt = scene.get_position(sim, payload_h)

    print(f"Payload GT  : {payload_gt[:2]}")
    print(f"Rally point : {rally_pos[:2]}")
    print(f"Robot start : {robot.get_position()[:2]}")
    print(f"PUSH_SPD={PUSH_SPD}  K_CENTER={K_CENTER}  "
          f"STALL_STEPS={STALL_STEPS}\n")

    sim.setStepping(True)
    sim.startSimulation()

    phase         = 'SCAN'
    attempt       = 1
    total_repos   = 0
    step          = 0

    ema_progress  = 0.0
    stall_counter = 0
    prev_payload  = None

    repos_stage   = 0
    repos_retreat = None
    repos_wp      = None

    try:
        while step < MAX_STEPS:
            sim.step()

            rpos         = robot.get_position()
            yaw          = robot.get_yaw()
            payload_real = scene.get_position(sim, payload_h)
            rally_pos    = scene.get_position(sim, rally_h)
            d_rally      = scene.dist2d(payload_real, rally_pos)

            px, py = payload_real[0], payload_real[1]

            d3 = read_sensor(sim, robot.sensors[3]) \
                 if len(robot.sensors) > 3 else float('inf')
            d4 = read_sensor(sim, robot.sensors[4]) \
                 if len(robot.sensors) > 4 else float('inf')

            # ── Success ──────────────────────────────────────────
            if scene.payload_reached_rally_point(payload_real, rally_pos):
                print(f"\n{'='*52}")
                print(f"  PAYLOAD REACHED RALLY POINT")
                print(f"  Steps       : {step}")
                print(f"  Attempts    : {attempt}")
                print(f"  Repositions : {total_repos}")
                print(f"  Final dist  : {d_rally:.4f}m")
                print(f"{'='*52}\n")
                robot.stop()
                break

            # ── Progress monitor ──────────────────────────────────
            if phase == 'PUSH':
                if prev_payload is not None:
                    dpx = px - prev_payload[0]
                    dpy = py - prev_payload[1]
                    ux, uy   = normalize(rally_pos[0]-px, rally_pos[1]-py)
                    progress = dpx*ux + dpy*uy
                    ema_progress  = EMA_ALPHA*progress + \
                                    (1-EMA_ALPHA)*ema_progress
                    stall_counter = stall_counter+1 \
                                    if ema_progress < MIN_PROGRESS else 0

                prev_payload  = (px, py)
                contact_lost  = (d3 == float('inf') and d4 == float('inf'))

                if stall_counter >= STALL_STEPS or contact_lost:
                    robot.stop()
                    reason = "contact lost" if contact_lost \
                             else f"stall ema={ema_progress:.5f}"
                    print(f"\n  [attempt {attempt}] REPOSITION  "
                          f"step={step}  {reason}  d_rally={d_rally:.3f}m")
                    phase         = 'REPOSITION'
                    repos_stage   = 0
                    repos_retreat = retreat_point(px, py, rpos)
                    repos_wp      = safe_waypoint(px, py, rally_pos)
                    ema_progress  = 0.0
                    stall_counter = 0
                    prev_payload  = None
                    total_repos  += 1

            # ── SCAN — GT navigation to push position ─────────────
            if phase == 'SCAN':
                tgt   = push_target(px, py, rally_pos)
                d_tgt = dist2d(rpos, tgt)
                if d_tgt < POSITION_THR:
                    phase        = 'PUSH'
                    ema_progress = 0.0
                    stall_counter = 0
                    prev_payload  = None
                    print(f"  [attempt {attempt}] PUSH  step={step}  "
                          f"d3={d3:.3f} d4={d4:.3f}\n")
                else:
                    robot.drive_to(tgt, APPROACH_SPD)

            # ── PUSH — GT direction + sensor centering ────────────
            elif phase == 'PUSH':
                fx, fy = compute_push_vector(px, py, rally_pos, d3, d4)
                apply_force(robot, fx, fy, PUSH_SPD)

            # ── REPOSITION ────────────────────────────────────────
            elif phase == 'REPOSITION':
                if repos_stage == 0:
                    if dist2d(rpos, repos_retreat) < POSITION_THR:
                        repos_stage = 1
                        repos_wp    = safe_waypoint(px, py, rally_pos)
                        print(f"  [attempt {attempt}] REPOS→waypoint  "
                              f"step={step}")
                    else:
                        robot.drive_to(repos_retreat, APPROACH_SPD)
                else:
                    repos_wp = safe_waypoint(px, py, rally_pos)
                    if dist2d(rpos, repos_wp) < POSITION_THR * 2:
                        phase        = 'SCAN'
                        attempt     += 1
                        print(f"  [attempt {attempt}] SCAN  step={step}\n")
                    else:
                        robot.drive_to(repos_wp, APPROACH_SPD)

            # ── Log ──────────────────────────────────────────────
            if step % LOG_EVERY == 0:
                fx_l, fy_l = compute_push_vector(px, py, rally_pos, d3, d4) \
                             if phase == 'PUSH' else (0, 0)
                extra = (f"F=({fx_l:+.2f},{fy_l:+.2f})  "
                         f"ema:{ema_progress:.5f} stall:{stall_counter}"
                         if phase == 'PUSH' else "")
                print(f"step={step:4d}  [{phase:10s}]  "
                      f"d_rally:{d_rally:.3f}m  "
                      f"d3:{d3:.3f} d4:{d4:.3f}  "
                      f"payload:({px:.2f},{py:.2f})  "
                      f"robot:({rpos[0]:.2f},{rpos[1]:.2f})  {extra}")

            step += 1

        if step >= MAX_STEPS:
            print(f"\nTIMEOUT ({MAX_STEPS} steps).")
            robot.stop()

    except KeyboardInterrupt:
        print("\nInterrupted.")
        robot.stop()
    finally:
        sim.stopSimulation()
        print("Simulation stopped.")


if __name__ == '__main__':
    main()