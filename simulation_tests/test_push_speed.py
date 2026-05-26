"""
test_push_speed.py

Speed sweep test: runs the single-robot push at multiple PUSH_SPD values
and reports time, steps, final hdg_err and lateral drift for each.

Run:
    python -m simulation_tests.test_push_speed
"""

import time
import math

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from src.robot import Robot
from src import scene


PAYLOAD_HALF   = 0.25
ROBOT_HALF_LEN = 0.26
PUSH_DIST      = PAYLOAD_HALF + ROBOT_HALF_LEN + 0.03  # 0.54m

POSITION_THR   = 0.18
HEADING_THR    = 0.08
APPROACH_SPD   = 2.0

SPEEDS_TO_TEST = [1.5, 2.0, 2.5, 3.0, 3.5]

MAX_STEPS      = 5000
LOG_EVERY      = 40


def normalize(dx, dy):
    n = math.sqrt(dx*dx + dy*dy) + 1e-9
    return dx/n, dy/n

def push_target(payload_pos, rally_pos):
    ux, uy = normalize(rally_pos[0]-payload_pos[0], rally_pos[1]-payload_pos[1])
    return [payload_pos[0]-ux*PUSH_DIST, payload_pos[1]-uy*PUSH_DIST, 0.0]

def push_heading(payload_pos, rally_pos):
    ux, uy = normalize(rally_pos[0]-payload_pos[0], rally_pos[1]-payload_pos[1])
    return math.atan2(uy, ux)

def apply_push(robot, hdg, speed):
    yaw = robot.get_yaw()
    err = math.atan2(math.sin(hdg-yaw), math.cos(hdg-yaw))
    w   = max(-1.5, min(1.5, 3.0*err))
    robot.set_velocity(speed-w, speed+w)

def dist2d(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)


def run_once(sim, push_spd: float) -> dict:
    """Run a single push trial at given speed. Returns result dict."""
    robot     = Robot(sim, '/p3dx_1', name='1')
    payload_h = sim.getObject('/payload')
    rally_h   = sim.getObject('/rally_point')

    sim.setStepping(True)
    sim.startSimulation()

    phase      = 'APPROACH'
    t0         = time.time()
    t_push     = None
    step       = 0
    max_hdgerr = 0.0
    start_pos  = scene.get_position(sim, payload_h)

    result = {'speed': push_spd, 'outcome': 'TIMEOUT',
              'steps': MAX_STEPS, 'time': 0,
              'push_time': 0, 'final_dist': 0,
              'max_hdg_err': 0, 'lateral_drift': 0}

    while step < MAX_STEPS:
        sim.step()

        payload_pos = scene.get_position(sim, payload_h)
        rally_pos   = scene.get_position(sim, rally_h)
        d_rally     = scene.dist2d(payload_pos, rally_pos)
        tgt         = push_target(payload_pos, rally_pos)
        hdg         = push_heading(payload_pos, rally_pos)
        rpos        = robot.get_position()
        d_tgt       = dist2d(rpos, tgt)
        yaw         = robot.get_yaw()
        h_err       = abs(math.atan2(math.sin(hdg-yaw), math.cos(hdg-yaw)))

        if phase == 'PUSH':
            max_hdgerr = max(max_hdgerr, math.degrees(h_err))

        if scene.payload_reached_rally_point(payload_pos, rally_pos):
            elapsed = time.time() - t0
            dev = scene.lateral_deviation(payload_pos, rally_pos, start_pos)
            result.update({
                'outcome': 'SUCCESS',
                'steps': step,
                'time': elapsed,
                'push_time': elapsed - (t_push or elapsed),
                'final_dist': d_rally,
                'max_hdg_err': max_hdgerr,
                'lateral_drift': abs(dev),
            })
            robot.stop()
            break

        if phase == 'APPROACH':
            if d_tgt < POSITION_THR:
                phase = 'ALIGN'
            elif d_tgt < 0.25:
                apply_push(robot, hdg, speed=0.6)
            else:
                robot.drive_to(tgt, APPROACH_SPD)

        elif phase == 'ALIGN':
            if h_err < HEADING_THR:
                phase  = 'PUSH'
                t_push = time.time() - t0
            else:
                err = math.atan2(math.sin(hdg-yaw), math.cos(hdg-yaw))
                w   = max(-1.5, min(1.5, 4.0*err))
                robot.set_velocity(-w, w)

        elif phase == 'PUSH':
            apply_push(robot, hdg, speed=push_spd)

        if step % LOG_EVERY == 0 and phase == 'PUSH':
            elapsed = time.time() - t0
            print(f"  spd={push_spd:.1f}  step={step:4d}  "
                  f"t={elapsed:5.1f}s  d={d_rally:.3f}m  "
                  f"hdg_err={math.degrees(h_err):.1f}°")

        step += 1

    sim.stopSimulation()
    time.sleep(0.5)  # let sim fully stop before next run
    return result


def main():
    print("=== SPEED SWEEP TEST ===\n")
    print(f"Testing speeds: {SPEEDS_TO_TEST}\n")

    client = RemoteAPIClient()
    sim    = client.getObject('sim')

    results = []
    for spd in SPEEDS_TO_TEST:
        print(f"\n── PUSH_SPD = {spd:.1f} rad/s ──────────────────────")
        r = run_once(sim, spd)
        results.append(r)
        status = r['outcome']
        if status == 'SUCCESS':
            print(f"  ✓ SUCCESS  steps={r['steps']}  time={r['time']:.1f}s  "
                  f"push={r['push_time']:.1f}s  "
                  f"dist={r['final_dist']:.3f}m  "
                  f"max_hdg={r['max_hdg_err']:.1f}°  "
                  f"drift={r['lateral_drift']:.3f}m")
        else:
            print(f"  ✗ {status}")

    # Summary table
    print(f"\n{'='*72}")
    print(f"{'speed':>7} {'outcome':>9} {'steps':>6} {'time(s)':>8} "
          f"{'push(s)':>8} {'dist(m)':>8} {'hdg_err°':>9} {'drift(m)':>9}")
    print(f"{'-'*72}")
    for r in results:
        print(f"{r['speed']:>7.1f} {r['outcome']:>9} {r['steps']:>6} "
              f"{r['time']:>8.1f} {r['push_time']:>8.1f} "
              f"{r['final_dist']:>8.3f} {r['max_hdg_err']:>9.1f} "
              f"{r['lateral_drift']:>9.3f}")
    print(f"{'='*72}\n")


if __name__ == '__main__':
    main()