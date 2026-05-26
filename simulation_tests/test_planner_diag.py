"""
test_planner_diag.py — v2

Causal diagnostic: force decomposition + torque proxy.

Per step logs:
  KINEMATICS  : payload pos, velocity, angular velocity
  ACTUATION   : per-robot push direction, force scale
  CONTACT     : proxy distance robot→payload (is robot really pushing?)
  FORCE MODEL : net force along F_des, net torque about payload center
  PROGRESS    : progress along FIXED F_des (not recalculated), lateral drift

F_des is fixed at t=0 (initial payload→rally direction).
Torque proxy: τ_i = cross2d(robot_pos - payload_pos, push_dir_i) * force_scale_i

Run: python -m simulation_tests.test_planner_diag
"""

import math
import sys
import os
import csv
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from src.robot import Robot
from src import scene
from src.planner import ContactPlanner, _dist, _norm

PUSH_SPD     = 2.0
APPROACH_SPD = 2.0
POSITION_THR = 0.15
K_CENTER     = 3.0
DETECT_DIST  = 0.80
EMA_ALPHA    = 0.15
MIN_PROGRESS = 0.0002
STALL_STEPS  = 100
REPLAN_EVERY = 10
MAX_STEPS    = 4000
NAV_TIMEOUT  = 400  # steps in NAVIGATE without reaching approach → reset and force replan
LOG_DIR      = os.path.join(os.path.dirname(__file__), 'logs')

R1_START  = [-1.725, -1.475, 0.195]
R2_START  = [-1.750,  1.850, 0.195]
PAY_START = [ 0.000,  0.000, 0.250]

SCENARIOS = [
    ('NE_baseline',  1.125,  0.225),
    ('N_north',      0.000,  1.800),
    ('SE_southeast', 1.200, -1.200),
]


def cross2d(rx, ry, fx, fy):
    """2D cross product (scalar torque): r × f = rx*fy - ry*fx"""
    return rx*fy - ry*fx


def read_sensors(sim, robot):
    d3 = d4 = float('inf')
    try:
        res = sim.readProximitySensor(robot.sensors[3])
        if res[0] > 0: d3 = float(res[1])
    except Exception: pass
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


class RobotAgent:
    def __init__(self, robot, name):
        self.robot = robot; self.name = name
        self.phase = 'NAVIGATE'; self.assignment = None; self.wp_idx = 0
        self.nav_steps = 0

    def reset(self):
        self.phase = 'NAVIGATE'; self.assignment = None; self.wp_idx = 0
        self.nav_steps = 0
        self.robot.stop()

    def update_assignment(self, assignment):
        face_changed = (self.assignment is None or
                        self.assignment['face'] != assignment['face'])
        if face_changed:
            self.phase = 'NAVIGATE'
            self.wp_idx = 0
            self.nav_steps = 0
        self.assignment = assignment

    def effective_push(self, d3, d4):
        """Returns (fx, fy, speed, in_contact) accounting for centering."""
        if not self.assignment or not self.assignment['face']:
            return 0.0, 0.0, 0.0, False
        pd = self.assignment['push_dir']
        fx, fy = float(pd[0]), float(pd[1])
        if d3 < DETECT_DIST and d4 < DETECT_DIST:
            ux, uy = fx, fy; px_, py_ = -uy, ux
            c = K_CENTER * (d4 - d3)
            fx = ux + px_*c; fy = uy + py_*c
            n = math.sqrt(fx*fx+fy*fy)+1e-9; fx, fy = fx/n, fy/n
        speed = PUSH_SPD * max(0.3, self.assignment['force_scale'])
        in_contact = (d3 < DETECT_DIST and d4 < DETECT_DIST)
        return fx, fy, speed, in_contact

    def step(self, d3, d4):
        if self.assignment is None or self.assignment['face'] is None:
            self.robot.stop(); return 'IDLE'
        rpos = self.robot.get_position(); asgn = self.assignment
        if self.phase == 'NAVIGATE':
            self.nav_steps += 1
            if self.nav_steps >= NAV_TIMEOUT:
                # stuck in NAVIGATE — reset to force replanning from current position
                self.reset()
                return 'IDLE'
            wps = asgn['waypoints']
            if self.wp_idx < len(wps):
                wp = wps[self.wp_idx]
                if _dist(rpos, wp) < POSITION_THR: self.wp_idx += 1
                else: self.robot.drive_to([wp[0], wp[1], 0.0], APPROACH_SPD)
                return 'NAVIGATE'
            app = asgn['approach']
            if _dist(rpos, app) < POSITION_THR:
                self.phase = 'PUSH'; self.nav_steps = 0; return 'PUSH'
            self.robot.drive_to([app[0], app[1], 0.0], APPROACH_SPD)
            return 'NAVIGATE'
        elif self.phase == 'PUSH':
            fx, fy, speed, _ = self.effective_push(d3, d4)
            apply_force(self.robot, fx, fy, speed)
            return 'PUSH'
        self.robot.stop(); return 'IDLE'


def reset_object(sim, handle, pos):
    sim.setObjectPosition(handle, -1, pos)
    try: sim.resetDynamicObject(handle)
    except Exception: pass


def run_diag(sim, r1, r2, payload_h, rally_h, agent1, agent2, name, rally_xy):
    rx, ry = rally_xy
    print(f"\n{'─'*55}\n  DIAG: {name}  rally=({rx},{ry})\n{'─'*55}")

    reset_object(sim, sim.getObject('/p3dx_1'), R1_START)
    reset_object(sim, sim.getObject('/p3dx_2'), R2_START)
    reset_object(sim, payload_h, PAY_START)
    sim.setObjectPosition(rally_h, -1, [rx, ry, 0.01])
    agent1.reset(); agent2.reset()
    planner = ContactPlanner()

    # Fixed F_des reference (initial payload→rally, does NOT change)
    n0 = math.sqrt(rx*rx+ry*ry)+1e-9
    F_fixed_x, F_fixed_y = rx/n0, ry/n0

    os.makedirs(LOG_DIR, exist_ok=True)
    csv_path = os.path.join(LOG_DIR, f'diag_{name}.csv')
    fields = [
        # Identity
        'step', 'scenario',
        # Payload kinematics
        'px','py','pvx','pvy','pwz',
        # Progress (FIXED F_des reference)
        'progress_fixed', 'lateral_fixed',
        # Progress (dynamic F_des — for comparison)
        'progress_dyn', 'lateral_dyn',
        'd_rally',
        # Agent states
        'r1_phase','r1_face','r1_fs',
        'r2_phase','r2_face','r2_fs',
        # Effective push directions (after centering correction)
        'r1_fx','r1_fy', 'r2_fx','r2_fy',
        # Contact proxy (sensor distance)
        'd3_r1','d4_r1', 'd3_r2','d4_r2',
        'r1_contact','r2_contact',
        # Robot positions (for torque arm calculation)
        'r1_px','r1_py', 'r2_px','r2_py',
        # Torque proxy: cross(robot_pos - payload_pos, push_dir) * force_scale
        'tau1','tau2','tau_net',
        # Net force decomposition
        'Fx_net','Fy_net',
        'F_along_goal','F_lateral',
        # Stall state
        'ema_progress','stall_counter',
        # NAV timeout tracking
        'r1_nav_steps','r2_nav_steps',
    ]

    ema_progress = 0.0; stall_counter = 0
    prev_payload = None; stall_count = 0; step = 0; result = 'TIMEOUT'
    prev_pvx = prev_pvy = 0.0

    with open(csv_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()

        while step < MAX_STEPS:
            sim.step()

            pay_pos = scene.get_position(sim, payload_h)
            rally_pos = scene.get_position(sim, rally_h)
            d_rally = scene.dist2d(pay_pos, rally_pos)
            px, py = pay_pos[0], pay_pos[1]

            # Payload velocity
            try:
                lin, ang = sim.getObjectVelocity(payload_h)
                pvx, pvy, pwz = lin[0], lin[1], ang[2]
            except Exception:
                pvx = pvy = pwz = 0.0

            # Progress relative to FIXED F_des
            progress_fixed = pvx*F_fixed_x + pvy*F_fixed_y
            lateral_fixed  = pvx*(-F_fixed_y) + pvy*F_fixed_x

            # Progress relative to DYNAMIC F_des (current payload→rally)
            ddx = rx-px; ddy = ry-py
            dn = math.sqrt(ddx*ddx+ddy*ddy)+1e-9
            fux, fuy = ddx/dn, ddy/dn
            progress_dyn = pvx*fux + pvy*fuy
            lateral_dyn  = pvx*(-fuy) + pvy*fux

            d3_r1, d4_r1 = read_sensors(sim, r1)
            d3_r2, d4_r2 = read_sensors(sim, r2)

            if scene.payload_reached_rally_point(pay_pos, rally_pos):
                result = 'SUCCESS'; print(f"  SUCCESS at step {step}"); break

            if step % REPLAN_EVERY == 0:
                robots_pos = [r1.get_position()[:2], r2.get_position()[:2]]
                plan = planner.plan(robots_pos, [px, py], [rx, ry])
                for agent, p in zip([agent1, agent2], plan):
                    agent.update_assignment(p)

            both_pushing = all(a.phase == 'PUSH' for a in [agent1, agent2])
            if both_pushing and prev_payload is not None:
                dpx_ = px - prev_payload[0]; dpy_ = py - prev_payload[1]
                prog = dpx_*fux + dpy_*fuy
                ema_progress  = EMA_ALPHA*prog + (1-EMA_ALPHA)*ema_progress
                stall_counter = stall_counter+1 if ema_progress < MIN_PROGRESS else 0
                if stall_counter >= STALL_STEPS:
                    agent1.reset(); agent2.reset(); planner.reset()
                    ema_progress=0.0; stall_counter=0; stall_count+=1
                    print(f"  [STALL] step={step} d_rally={d_rally:.3f}")
            prev_payload = (px,py) if both_pushing else None

            ph1 = agent1.step(d3_r1, d4_r1)
            ph2 = agent2.step(d3_r2, d4_r2)

            # Get effective push directions
            fx1, fy1, spd1, con1 = agent1.effective_push(d3_r1, d4_r1)
            fx2, fy2, spd2, con2 = agent2.effective_push(d3_r2, d4_r2)

            # Only consider force if robot is in PUSH
            if ph1 != 'PUSH': fx1=fy1=0.0; con1=False
            if ph2 != 'PUSH': fx2=fy2=0.0; con2=False

            fs1 = agent1.assignment['force_scale'] if agent1.assignment else 0.0
            fs2 = agent2.assignment['force_scale'] if agent2.assignment else 0.0

            # Torque proxy: cross(r_i - payload, push_dir_i) * force_scale
            r1p = r1.get_position(); r2p = r2.get_position()
            arm1x, arm1y = r1p[0]-px, r1p[1]-py
            arm2x, arm2y = r2p[0]-px, r2p[1]-py
            tau1 = cross2d(arm1x, arm1y, fx1, fy1) * fs1 if ph1=='PUSH' else 0.0
            tau2 = cross2d(arm2x, arm2y, fx2, fy2) * fs2 if ph2=='PUSH' else 0.0
            tau_net = tau1 + tau2

            # Net force decomposition
            Fx_net = fx1*fs1 + fx2*fs2
            Fy_net = fy1*fs1 + fy2*fs2
            F_along  = Fx_net*F_fixed_x + Fy_net*F_fixed_y
            F_lat    = Fx_net*(-F_fixed_y) + Fy_net*F_fixed_x

            f1  = agent1.assignment['face'] if agent1.assignment else '-'
            f2  = agent2.assignment['face'] if agent2.assignment else '-'

            writer.writerow({
                'step': step, 'scenario': name,
                'px': round(px,4), 'py': round(py,4),
                'pvx': round(pvx,5), 'pvy': round(pvy,5), 'pwz': round(pwz,5),
                'progress_fixed': round(progress_fixed,5),
                'lateral_fixed':  round(lateral_fixed,5),
                'progress_dyn':   round(progress_dyn,5),
                'lateral_dyn':    round(lateral_dyn,5),
                'd_rally': round(d_rally,4),
                'r1_phase': ph1, 'r1_face': f1 or '-', 'r1_fs': round(fs1,3),
                'r2_phase': ph2, 'r2_face': f2 or '-', 'r2_fs': round(fs2,3),
                'r1_fx': round(fx1,3), 'r1_fy': round(fy1,3),
                'r2_fx': round(fx2,3), 'r2_fy': round(fy2,3),
                'd3_r1': round(d3_r1,3) if d3_r1<9 else 'inf',
                'd4_r1': round(d4_r1,3) if d4_r1<9 else 'inf',
                'd3_r2': round(d3_r2,3) if d3_r2<9 else 'inf',
                'd4_r2': round(d4_r2,3) if d4_r2<9 else 'inf',
                'r1_contact': int(con1), 'r2_contact': int(con2),
                'r1_px': round(r1p[0],3), 'r1_py': round(r1p[1],3),
                'r2_px': round(r2p[0],3), 'r2_py': round(r2p[1],3),
                'tau1': round(tau1,5), 'tau2': round(tau2,5),
                'tau_net': round(tau_net,5),
                'Fx_net': round(Fx_net,4), 'Fy_net': round(Fy_net,4),
                'F_along_goal': round(F_along,4), 'F_lateral': round(F_lat,4),
                'ema_progress': round(ema_progress,6),
                'stall_counter': stall_counter,
                'r1_nav_steps': agent1.nav_steps,
                'r2_nav_steps': agent2.nav_steps,
            })

            step += 1

    r1.stop(); r2.stop()
    print(f"  {result}  steps={step}  stalls={stall_count}")
    print(f"  CSV → {csv_path}")
    return csv_path


def main():
    print("=== PLANNER DIAGNOSTIC v2 ===\n")
    client    = RemoteAPIClient()
    sim       = client.getObject('sim')
    r1        = Robot(sim, '/p3dx_1', name='1')
    r2        = Robot(sim, '/p3dx_2', name='2')
    payload_h = sim.getObject('/payload')
    rally_h   = sim.getObject('/rally_point')
    agent1 = RobotAgent(r1, 'R1'); agent2 = RobotAgent(r2, 'R2')

    sim.setStepping(True); sim.startSimulation()
    csvs = []
    try:
        for name, rx, ry in SCENARIOS:
            csv_path = run_diag(sim, r1, r2, payload_h, rally_h,
                                agent1, agent2, name, (rx, ry))
            csvs.append(csv_path)
    except KeyboardInterrupt:
        print("\nInterrupted."); r1.stop(); r2.stop()
    finally:
        sim.stopSimulation(); print("\nSimulation stopped.")

    print(f"\nCSVs saved:")
    for p in csvs: print(f"  {p}")


if __name__ == '__main__':
    main()