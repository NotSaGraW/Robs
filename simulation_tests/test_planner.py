"""
test_planner.py — v3

Two-robot cooperative push using force-decomposition ContactPlanner.
Run: python -m simulation_tests.test_planner

Changes vs v2:
  - Sensor routing unified: both agents receive their own d3/d4 observation
  - Planner uses 8-direction contact frames (see planner.py)
"""

import math
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from src.robot import Robot
from src import scene
from src.planner import ContactPlanner, _dist, _norm


PUSH_SPD      = 2.0
APPROACH_SPD  = 2.0
POSITION_THR  = 0.15
K_CENTER      = 3.0
DETECT_DIST   = 0.80
EMA_ALPHA     = 0.15
MIN_PROGRESS  = 0.0002
STALL_STEPS   = 100
REPLAN_EVERY  = 10
MAX_STEPS     = 6000
LOG_EVERY     = 20


def read_sensors(sim, robot):
    """Read frontal sensors d3, d4 for a given robot."""
    d3 = float('inf')
    d4 = float('inf')
    if len(robot.sensors) > 3:
        try:
            res = sim.readProximitySensor(robot.sensors[3])
            if res[0] > 0:
                d3 = float(res[1])
        except Exception:
            pass
    if len(robot.sensors) > 4:
        try:
            res = sim.readProximitySensor(robot.sensors[4])
            if res[0] > 0:
                d4 = float(res[1])
        except Exception:
            pass
    return d3, d4


def apply_force(robot, fx, fy, speed):
    hdg = math.atan2(fy, fx)
    yaw = robot.get_yaw()
    err = math.atan2(math.sin(hdg-yaw), math.cos(hdg-yaw))
    w   = max(-1.5, min(1.5, 3.0*err))
    robot.set_velocity(speed-w, speed+w)


class RobotAgent:
    def __init__(self, robot, name):
        self.robot      = robot
        self.name       = name
        self.phase      = 'NAVIGATE'
        self.assignment = None
        self.wp_idx     = 0

    def update_assignment(self, assignment):
        face_changed = (self.assignment is None or
                        self.assignment['face'] != assignment['face'])
        if face_changed:
            self.phase = 'NAVIGATE'
        self.assignment = assignment
        self.wp_idx = 0   # always reset — waypoints shift with payload

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

    def reset(self):
        self.phase      = 'NAVIGATE'
        self.wp_idx     = 0
        self.assignment = None


def main():
    print("=== PLANNER-BASED TWO ROBOT PUSH v3 (8-direction frames) ===\n")

    client    = RemoteAPIClient()
    sim       = client.getObject('sim')
    r1        = Robot(sim, '/p3dx_1', name='1')
    r2        = Robot(sim, '/p3dx_2', name='2')
    payload_h = sim.getObject('/payload')
    rally_h   = sim.getObject('/rally_point')

    rally_pos  = scene.get_position(sim, rally_h)
    payload_gt = scene.get_position(sim, payload_h)

    print(f"Payload GT  : {payload_gt[:2]}")
    print(f"Rally point : {rally_pos[:2]}")
    print(f"R1 start    : {r1.get_position()[:2]}")
    print(f"R2 start    : {r2.get_position()[:2]}")
    print(f"PUSH_SPD={PUSH_SPD}  K_CENTER={K_CENTER}\n")

    planner = ContactPlanner()
    agent1  = RobotAgent(r1, 'R1')
    agent2  = RobotAgent(r2, 'R2')
    agents  = [agent1, agent2]

    sim.setStepping(True)
    sim.startSimulation()

    step          = 0
    ema_progress  = 0.0
    stall_counter = 0
    prev_payload  = None
    stall_count   = 0

    try:
        while step < MAX_STEPS:
            sim.step()

            payload_real = scene.get_position(sim, payload_h)
            rally_pos    = scene.get_position(sim, rally_h)
            d_rally      = scene.dist2d(payload_real, rally_pos)
            px, py       = payload_real[0], payload_real[1]

            # Unified sensor observation — each robot reads its own sensors
            d3_r1, d4_r1 = read_sensors(sim, r1)
            d3_r2, d4_r2 = read_sensors(sim, r2)

            if scene.payload_reached_rally_point(payload_real, rally_pos):
                print(f"\n{'='*52}")
                print(f"  PAYLOAD REACHED RALLY POINT")
                print(f"  Steps       : {step}")
                print(f"  Stall resets: {stall_count}")
                print(f"  Final dist  : {d_rally:.4f}m")
                print(f"{'='*52}\n")
                r1.stop(); r2.stop()
                break

            if step % REPLAN_EVERY == 0:
                robots_pos = [r1.get_position()[:2], r2.get_position()[:2]]
                plan = planner.plan(robots_pos, [px, py], rally_pos[:2])
                for i, (agent, p) in enumerate(zip(agents, plan)):
                    old_face = agent.assignment['face'] if agent.assignment else None
                    agent.update_assignment(p)
                    if old_face != p['face'] and p['face'] is not None:
                        pd  = p['push_dir']
                        nav = ('direct' if not p['waypoints']
                               else str([(round(w[0],2),round(w[1],2))
                                         for w in p['waypoints']]))
                        print(f"  [{agent.name}] face={p['face']:3s}  "
                              f"push=({pd[0]:+.3f},{pd[1]:+.3f})  "
                              f"f={p['force_scale']:.3f}  nav={nav}")

            both_pushing = all(a.phase == 'PUSH' for a in agents)
            if both_pushing and prev_payload is not None:
                dpx_ = px - prev_payload[0]
                dpy_ = py - prev_payload[1]
                ux_, uy_ = _norm(rally_pos[0]-px, rally_pos[1]-py)
                progress      = dpx_*ux_ + dpy_*uy_
                ema_progress  = EMA_ALPHA*progress + (1-EMA_ALPHA)*ema_progress
                stall_counter = stall_counter+1 if ema_progress < MIN_PROGRESS else 0

                if stall_counter >= STALL_STEPS:
                    print(f"\n  [STALL] step={step}  ema={ema_progress:.5f}  "
                          f"d_rally={d_rally:.3f}m → reset")
                    for a in agents:
                        a.reset()
                    planner.reset()
                    ema_progress = 0.0; stall_counter = 0; stall_count += 1

            prev_payload = (px, py) if both_pushing else None

            ph1 = agent1.step(d3_r1, d4_r1)
            ph2 = agent2.step(d3_r2, d4_r2)

            if step % LOG_EVERY == 0:
                r1p = r1.get_position()
                r2p = r2.get_position()
                f1  = agent1.assignment['face'] if agent1.assignment else '-'
                f2  = agent2.assignment['face'] if agent2.assignment else '-'
                fs1 = agent1.assignment['force_scale'] if agent1.assignment else 0
                fs2 = agent2.assignment['force_scale'] if agent2.assignment else 0
                print(f"step={step:4d}  d_rally:{d_rally:.3f}m  "
                      f"R1:[{ph1:3s}|{f1 or '-':3s}|f={fs1:.2f}]"
                      f"({r1p[0]:.2f},{r1p[1]:.2f}) "
                      f"d3:{d3_r1:.3f} d4:{d4_r1:.3f}  "
                      f"R2:[{ph2:3s}|{f2 or '-':3s}|f={fs2:.2f}]"
                      f"({r2p[0]:.2f},{r2p[1]:.2f}) "
                      f"d3:{d3_r2:.3f} d4:{d4_r2:.3f}  "
                      f"ema:{ema_progress:.5f}")

            step += 1

        if step >= MAX_STEPS:
            print(f"\nTIMEOUT ({MAX_STEPS} steps).")
            r1.stop(); r2.stop()

    except KeyboardInterrupt:
        print("\nInterrupted.")
        r1.stop(); r2.stop()
    finally:
        sim.stopSimulation()
        print("Simulation stopped.")


if __name__ == '__main__':
    main()