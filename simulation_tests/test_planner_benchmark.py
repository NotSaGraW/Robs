"""
test_planner_benchmark.py

Multi-scenario benchmark for ContactPlanner.
Runs the same push task for N rally positions without restarting CoppeliaSim.

Each scenario:
  1. Resets payload and robots to start positions
  2. Sets rally_point to scenario position
  3. Runs until success or timeout
  4. Prints summary

Run: python -m simulation_tests.test_planner_benchmark
"""

import math
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from src.robot import Robot
from src import scene
from src.planner import ContactPlanner, _dist, _norm


# ── Constants ─────────────────────────────────────────────────────
PUSH_SPD      = 2.0
APPROACH_SPD  = 2.0
POSITION_THR  = 0.15
K_CENTER      = 3.0
DETECT_DIST   = 0.80
EMA_ALPHA     = 0.15
MIN_PROGRESS  = 0.0002
STALL_STEPS   = 100
REPLAN_EVERY  = 10
MAX_STEPS     = 4000
LOG_EVERY     = 50

# Start positions (from scene geometry)
R1_START  = [-1.725, -1.475, 0.195]
R2_START  = [-1.750,  1.850, 0.195]
PAY_START = [ 0.000,  0.000, 0.250]

# Scenarios: (name, rally_x, rally_y)
SCENARIOS = [
    ('NE  (baseline)', 1.125,  0.225),
    ('E   (east)',     1.800,  0.000),
    ('N   (north)',    0.000,  1.800),
    ('NW  (northwest)',-1.500, 1.000),
    ('SE  (southeast)', 1.200,-1.200),
]


# ── Sensor helper ─────────────────────────────────────────────────

def read_sensors(sim, robot):
    d3 = d4 = float('inf')
    if len(robot.sensors) > 3:
        try:
            res = sim.readProximitySensor(robot.sensors[3])
            if res[0] > 0: d3 = float(res[1])
        except Exception: pass
    if len(robot.sensors) > 4:
        try:
            res = sim.readProximitySensor(robot.sensors[4])
            if res[0] > 0: d4 = float(res[1])
        except Exception: pass
    return d3, d4


def apply_force(robot, fx, fy, speed):
    hdg = math.atan2(fy, fx)
    yaw = robot.get_yaw()
    err = math.atan2(math.sin(hdg-yaw), math.cos(hdg-yaw))
    w   = max(-1.5, min(1.5, 3.0*err))
    robot.set_velocity(speed-w, speed+w)


# ── Agent ─────────────────────────────────────────────────────────

class RobotAgent:
    def __init__(self, robot, name):
        self.robot      = robot
        self.name       = name
        self.phase      = 'NAVIGATE'
        self.assignment = None
        self.wp_idx     = 0

    def reset(self):
        self.phase      = 'NAVIGATE'
        self.assignment = None
        self.wp_idx     = 0
        self.robot.stop()

    def update_assignment(self, assignment):
        face_changed = (self.assignment is None or
                        self.assignment['face'] != assignment['face'])
        if face_changed:
            self.phase = 'NAVIGATE'
        self.assignment = assignment
        self.wp_idx = 0

    def step(self, d3, d4):
        if self.assignment is None or self.assignment['face'] is None:
            self.robot.stop()
            return 'IDLE'

        rpos = self.robot.get_position()
        asgn = self.assignment

        if self.phase == 'NAVIGATE':
            waypoints = asgn['waypoints']
            if self.wp_idx < len(waypoints):
                wp = waypoints[self.wp_idx]
                if _dist(rpos, wp) < POSITION_THR:
                    self.wp_idx += 1
                else:
                    self.robot.drive_to([wp[0], wp[1], 0.0], APPROACH_SPD)
                return 'NAVIGATE'
            app = asgn['approach']
            if _dist(rpos, app) < POSITION_THR:
                self.phase = 'PUSH'
                return 'PUSH'
            self.robot.drive_to([app[0], app[1], 0.0], APPROACH_SPD)
            return 'NAVIGATE'

        elif self.phase == 'PUSH':
            pd = asgn['push_dir']
            fx, fy = float(pd[0]), float(pd[1])
            if d3 < DETECT_DIST and d4 < DETECT_DIST:
                ux, uy   = fx, fy
                px_, py_ = -uy, ux
                c        = K_CENTER * (d4 - d3)
                fx = ux + px_*c
                fy = uy + py_*c
                n  = math.sqrt(fx*fx+fy*fy) + 1e-9
                fx, fy = fx/n, fy/n
            speed = PUSH_SPD * max(0.3, asgn['force_scale'])
            apply_force(self.robot, fx, fy, speed)
            return 'PUSH'

        self.robot.stop()
        return 'IDLE'


# ── Reset helpers ─────────────────────────────────────────────────

def reset_object(sim, handle, pos):
    """Move object to start position and zero velocity."""
    sim.setObjectPosition(handle, -1, pos)
    try:
        sim.resetDynamicObject(handle)
    except Exception:
        pass


def run_scenario(sim, r1, r2, payload_h, rally_h,
                 agent1, agent2, scenario_name, rally_xy):
    """
    Run a single scenario. Returns dict with results.
    Resets all objects before starting.
    """
    rx, ry = rally_xy
    print(f"\n{'─'*60}")
    print(f"  SCENARIO: {scenario_name}")
    print(f"  Rally: ({rx:.3f}, {ry:.3f})")
    print(f"{'─'*60}")

    # Reset positions
    reset_object(sim, sim.getObject('/p3dx_1'), R1_START)
    reset_object(sim, sim.getObject('/p3dx_2'), R2_START)
    reset_object(sim, payload_h, PAY_START)
    sim.setObjectPosition(rally_h, -1, [rx, ry, 0.01])

    agent1.reset()
    agent2.reset()
    planner = ContactPlanner()

    ema_progress  = 0.0
    stall_counter = 0
    prev_payload  = None
    stall_count   = 0
    step          = 0
    result        = 'TIMEOUT'
    final_dist    = None
    first_assign  = None

    while step < MAX_STEPS:
        sim.step()

        payload_pos = scene.get_position(sim, payload_h)
        rally_pos   = scene.get_position(sim, rally_h)
        d_rally     = scene.dist2d(payload_pos, rally_pos)
        px, py      = payload_pos[0], payload_pos[1]

        d3_r1, d4_r1 = read_sensors(sim, r1)
        d3_r2, d4_r2 = read_sensors(sim, r2)

        if scene.payload_reached_rally_point(payload_pos, rally_pos):
            result     = 'SUCCESS'
            final_dist = d_rally
            break

        if step % REPLAN_EVERY == 0:
            robots_pos = [r1.get_position()[:2], r2.get_position()[:2]]
            plan = planner.plan(robots_pos, [px, py], [rx, ry])
            for i, (agent, p) in enumerate(zip([agent1, agent2], plan)):
                old_face = agent.assignment['face'] if agent.assignment else None
                agent.update_assignment(p)
                if first_assign is None and p['face'] is not None and old_face is None:
                    fs = [plan[j]['force_scale'] for j in range(2)]
                    ratio = min(fs)/(max(fs)+1e-9) if max(fs)>0 else 0
                    first_assign = {
                        'R1': plan[0]['face'],
                        'R2': plan[1]['face'],
                        'f':  (plan[0]['force_scale'], plan[1]['force_scale']),
                        'ratio': ratio,
                    }

        both_pushing = all(a.phase == 'PUSH' for a in [agent1, agent2])
        if both_pushing and prev_payload is not None:
            dpx_ = px - prev_payload[0]
            dpy_ = py - prev_payload[1]
            ux_, uy_ = _norm(rx-px, ry-py)
            progress      = dpx_*ux_ + dpy_*uy_
            ema_progress  = EMA_ALPHA*progress + (1-EMA_ALPHA)*ema_progress
            stall_counter = stall_counter+1 if ema_progress < MIN_PROGRESS else 0

            if stall_counter >= STALL_STEPS:
                agent1.reset(); agent2.reset()
                planner.reset()
                ema_progress = 0.0; stall_counter = 0; stall_count += 1
                print(f"  [STALL] step={step} d_rally={d_rally:.3f}m → reset #{stall_count}")

        prev_payload = (px, py) if both_pushing else None

        agent1.step(d3_r1, d4_r1)
        agent2.step(d3_r2, d4_r2)

        if step % LOG_EVERY == 0 and step > 0:
            f1 = agent1.assignment['face'] if agent1.assignment else '-'
            f2 = agent2.assignment['face'] if agent2.assignment else '-'
            print(f"  step={step:4d}  d_rally:{d_rally:.3f}m  "
                  f"R1:{agent1.phase[:3]}|{f1 or '-':3s}  "
                  f"R2:{agent2.phase[:3]}|{f2 or '-':3s}")

        step += 1

    r1.stop(); r2.stop()

    # Print result
    status = '✓' if result == 'SUCCESS' else '✗'
    print(f"\n  {status} {result}  steps={step}  "
          f"stall_resets={stall_count}  "
          f"final_dist={final_dist:.4f}m" if final_dist else
          f"\n  {status} {result}  steps={step}  stall_resets={stall_count}")
    if first_assign:
        fa = first_assign
        print(f"    Assignment: R1→{fa['R1']}  R2→{fa['R2']}  "
              f"f=({fa['f'][0]:.3f},{fa['f'][1]:.3f})  ratio={fa['ratio']:.2f}")

    return {
        'scenario': scenario_name,
        'rally':    rally_xy,
        'result':   result,
        'steps':    step,
        'stalls':   stall_count,
        'dist':     final_dist,
        'assign':   first_assign,
    }


# ── Main ──────────────────────────────────────────────────────────

def main():
    print("=== PLANNER BENCHMARK — multi-scenario ===\n")

    client    = RemoteAPIClient()
    sim       = client.getObject('sim')
    r1        = Robot(sim, '/p3dx_1', name='1')
    r2        = Robot(sim, '/p3dx_2', name='2')
    payload_h = sim.getObject('/payload')
    rally_h   = sim.getObject('/rally_point')

    agent1 = RobotAgent(r1, 'R1')
    agent2 = RobotAgent(r2, 'R2')

    sim.setStepping(True)
    sim.startSimulation()

    results = []
    try:
        for name, rx, ry in SCENARIOS:
            res = run_scenario(
                sim, r1, r2, payload_h, rally_h,
                agent1, agent2, name, (rx, ry)
            )
            results.append(res)

    except KeyboardInterrupt:
        print("\nInterrupted.")
        r1.stop(); r2.stop()
    finally:
        sim.stopSimulation()
        print("\nSimulation stopped.")

    # Summary table
    print(f"\n{'='*68}")
    print(f"  BENCHMARK SUMMARY")
    print(f"{'='*68}")
    print(f"  {'Scenario':<20} {'Result':<8} {'Steps':>6} {'Stalls':>6} "
          f"{'Dist':>7}  Assignment")
    print(f"  {'─'*64}")
    for r in results:
        fa  = r['assign']
        asg = f"R1→{fa['R1']} R2→{fa['R2']} ratio={fa['ratio']:.2f}" if fa else '-'
        dist_str = f"{r['dist']:.4f}m" if r['dist'] else 'N/A'
        print(f"  {r['scenario']:<20} {r['result']:<8} {r['steps']:>6} "
              f"{r['stalls']:>6} {dist_str:>7}  {asg}")
    print(f"{'='*68}\n")


if __name__ == '__main__':
    main()