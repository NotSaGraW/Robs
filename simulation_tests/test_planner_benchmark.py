"""
test_planner_benchmark.py

Route and sequence correctness across 5 canonical rally directions.

Question: Can the system move the payload to each rally direction within the
step budget?

Purpose: Acceptance test. Detects regression in planner assignment, navigation
routing, and step budget adequacy. Failure diagnosis requires reading the CSV —
the console log alone shows what failed, not why.

Output:
  logs/benchmark/benchmark_YYYYMMDD_HHMMSS.log  — full stdout + summary table
  logs/benchmark/benchmark_<name>_YYYYMMDD_HHMMSS.csv  — per-step causal data

Run:
  python -m simulation_tests.test_planner_benchmark
"""

import math
import sys
import os
import csv
from datetime import datetime
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from src.robot import Robot
from src import scene
from src.planner import ContactPlanner, _dist, _norm

LOG_DIR = os.path.join(os.path.dirname(__file__), 'logs', 'benchmark')


class _Tee:
    """Duplicate stdout to a log file without modifying any print calls."""
    def __init__(self, path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self._file   = open(path, 'w', encoding='utf-8')
        self._stdout = sys.stdout
        sys.stdout   = self
    def write(self, data):
        self._stdout.write(data); self._file.write(data)
    def flush(self):
        self._stdout.flush(); self._file.flush()
    def close(self):
        sys.stdout = self._stdout; self._file.close()


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
NAV_TIMEOUT   = 400   # steps in NAVIGATE without reaching approach → reset
LOG_EVERY     = 50

CONTACT_STALL_THR = 30    # steps in contact with no robot movement → BACKOFF
BACKOFF_STEPS     = 25    # steps to reverse before re-approaching
ROBOT_MOVE_THR    = 0.005 # m/step — below this = robot stuck despite pushing
VIABLE_MARGIN     = 0.80  # trigger replan when steps_to_goal > remaining * this
SPEED_BOOST       = 1.30  # speed multiplier for better-aligned robot when not viable

# Start positions (from scene geometry)
R1_START  = [-1.725, -1.475, 0.195]
R2_START  = [-1.750,  1.850, 0.195]
PAY_START = [ 0.000,  0.000, 0.250]

# Scenarios: (name, rally_x, rally_y)
SCENARIOS = [
    ('NE_baseline',   1.125,  0.225),
    ('E_east',        1.800,  0.000),
    ('N_north',       0.000,  1.800),
    ('NW_northwest', -1.500,  1.000),
    ('SE_southeast',  1.200, -1.200),
]


# ── Helpers ───────────────────────────────────────────────────────

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


def get_payload_angle(sim, payload_h):
    """Z-axis rotation of payload from initial orientation (rad)."""
    try:
        euler = sim.getObjectOrientation(payload_h, -1)
        return float(euler[2])
    except Exception:
        return 0.0


def get_object_velocity_2d(sim, handle):
    """Actual XY velocity of an object from physics engine (m/step)."""
    try:
        lin, _ = sim.getObjectVelocity(handle)
        return float(lin[0]), float(lin[1])
    except Exception:
        return 0.0, 0.0


def compute_heading_error(robot, push_dir):
    """Angle between robot yaw and push direction (rad). 0 = perfectly aligned."""
    if push_dir is None:
        return float('nan')
    hdg = math.atan2(float(push_dir[1]), float(push_dir[0]))
    yaw = robot.get_yaw()
    return math.atan2(math.sin(hdg - yaw), math.cos(hdg - yaw))


def fmt(v, n=4):
    return round(v, n) if v == v else 'nan'  # nan check


def plan_viable(d_rally, ema_progress, step, max_steps):
    """False when the current push rate cannot close d_rally within the remaining budget."""
    if ema_progress < MIN_PROGRESS * 2:
        return True  # rate unmeasurable — let stall detection handle it
    return (d_rally / ema_progress) < (max_steps - step) * VIABLE_MARGIN


# ── Agent ─────────────────────────────────────────────────────────

class RobotAgent:
    def __init__(self, robot, name):
        self.robot               = robot
        self.name                = name
        self.phase               = 'NAVIGATE'
        self.assignment          = None
        self.wp_idx              = 0
        self.nav_steps           = 0
        self._prev_face          = None
        self._contact_stall_ctr  = 0
        self._backoff_ctr        = 0
        self._last_rpos          = None

    def reset(self):
        self._prev_face          = self.assignment['face'] if self.assignment else None
        self.phase               = 'NAVIGATE'
        self.assignment          = None
        self.wp_idx              = 0
        self.nav_steps           = 0
        self._contact_stall_ctr  = 0
        self._backoff_ctr        = 0
        self._last_rpos          = None

    def update_assignment(self, assignment):
        self._prev_face  = self.assignment['face'] if self.assignment else None
        face_changed     = (self.assignment is None or
                            self.assignment['face'] != assignment['face'])
        if face_changed:
            self.phase               = 'NAVIGATE'
            self.nav_steps           = 0
            self.wp_idx              = 0
            self._contact_stall_ctr  = 0
            self._backoff_ctr        = 0
            self._last_rpos          = None
            self.assignment = assignment
        elif self.phase in ('PUSH', 'BACKOFF'):
            # In PUSH/BACKOFF update push parameters freely — approach is no longer used.
            self.assignment = assignment
        else:
            # In NAVIGATE: freeze approach+waypoints so the robot converges on a stable
            # target. Without this, every replan shifts the approach with the payload and
            # R2 chases a moving target indefinitely.
            self.assignment = {
                'face':        assignment['face'],
                'push_dir':    assignment['push_dir'],
                'force_scale': assignment['force_scale'],
                'score':       assignment['score'],
                'approach':    self.assignment['approach'],
                'waypoints':   self.assignment['waypoints'],
            }

    def face_changed(self):
        """True if the face assignment changed on the last update_assignment call."""
        if self.assignment is None:
            return False
        return self._prev_face != self.assignment['face']

    def effective_push(self, d3, d4):
        """Returns (fx, fy) effective push direction after centering correction."""
        if not self.assignment or not self.assignment['face']:
            return 0.0, 0.0
        pd = self.assignment['push_dir']
        fx, fy = float(pd[0]), float(pd[1])
        if d3 < DETECT_DIST and d4 < DETECT_DIST:
            ux, uy   = fx, fy; px_, py_ = -uy, ux
            c        = K_CENTER * (d4 - d3)
            fx = ux + px_*c; fy = uy + py_*c
            n  = math.sqrt(fx*fx+fy*fy)+1e-9; fx, fy = fx/n, fy/n
        return fx, fy

    def step(self, d3, d4, speed_scale=1.0, both_pushing=False):
        if self.assignment is None or self.assignment['face'] is None:
            self.robot.stop()
            return 'IDLE'

        rpos = self.robot.get_position()
        asgn = self.assignment

        if self.phase == 'NAVIGATE':
            self.nav_steps += 1
            if self.nav_steps >= NAV_TIMEOUT:
                self.reset()
                return 'IDLE'
            waypoints = asgn['waypoints']
            if self.wp_idx < len(waypoints):
                wp = waypoints[self.wp_idx]
                if _dist(rpos, wp) < POSITION_THR: self.wp_idx += 1
                else: self.robot.drive_to([wp[0], wp[1], 0.0], APPROACH_SPD)
                return 'NAVIGATE'
            app = asgn['approach']
            if _dist(rpos, app) < POSITION_THR:
                self.phase = 'PUSH'; self.nav_steps = 0; return 'PUSH'
            self.robot.drive_to([app[0], app[1], 0.0], APPROACH_SPD)
            return 'NAVIGATE'

        elif self.phase == 'PUSH':
            in_contact = (d3 < DETECT_DIST and d4 < DETECT_DIST)
            # Only count stall when both robots are pushing: a solo robot pressing
            # against a heavy payload naturally stays still — that is not stuck.
            if in_contact and both_pushing:
                moved = _dist(rpos, self._last_rpos) if self._last_rpos is not None else ROBOT_MOVE_THR + 1
                self._contact_stall_ctr = self._contact_stall_ctr + 1 if moved < ROBOT_MOVE_THR else 0
            else:
                self._contact_stall_ctr = 0
            self._last_rpos = list(rpos)

            if self._contact_stall_ctr >= CONTACT_STALL_THR:
                self._contact_stall_ctr = 0
                self._backoff_ctr       = BACKOFF_STEPS
                self.phase              = 'BACKOFF'
                self.robot.set_velocity(-APPROACH_SPD * 0.5, -APPROACH_SPD * 0.5)
                return 'BACKOFF'

            fx, fy = self.effective_push(d3, d4)
            speed  = PUSH_SPD * max(0.3, asgn['force_scale']) * speed_scale
            apply_force(self.robot, fx, fy, speed)
            return 'PUSH'

        elif self.phase == 'BACKOFF':
            self._backoff_ctr -= 1
            self.robot.set_velocity(-APPROACH_SPD * 0.5, -APPROACH_SPD * 0.5)
            if self._backoff_ctr <= 0:
                self.phase     = 'NAVIGATE'
                self.nav_steps = 0
                self.wp_idx    = 0
            return 'BACKOFF'

        self.robot.stop()
        return 'IDLE'


# ── Reset helpers ─────────────────────────────────────────────────

def reset_object(sim, handle, pos):
    sim.setObjectPosition(handle, -1, pos)
    try:
        sim.resetDynamicObject(handle)
    except Exception:
        pass


# ── Scenario runner ───────────────────────────────────────────────

def run_scenario(sim, r1, r2, r1_h, r2_h, payload_h, rally_h,
                 agent1, agent2, scenario_name, rally_xy, ts):
    rx, ry = rally_xy
    print(f"\n{'─'*60}")
    print(f"  SCENARIO: {scenario_name}")
    print(f"  Rally: ({rx:.3f}, {ry:.3f})")
    print(f"{'─'*60}")

    reset_object(sim, r1_h, R1_START)
    reset_object(sim, r2_h, R2_START)
    reset_object(sim, payload_h, PAY_START)
    sim.setObjectPosition(rally_h, -1, [rx, ry, 0.01])
    agent1.reset(); agent2.reset()
    planner     = ContactPlanner()
    pay0_angle  = get_payload_angle(sim, payload_h)  # reference orientation

    n0 = math.sqrt(rx*rx+ry*ry)+1e-9
    F_des_x, F_des_y = rx/n0, ry/n0   # fixed goal direction

    os.makedirs(LOG_DIR, exist_ok=True)
    csv_path = os.path.join(LOG_DIR, f'benchmark_{scenario_name}_{ts}.csv')

    fields = [
        'step', 'scenario',
        # Payload state
        'px', 'py', 'payload_angle',
        'pvx', 'pvy', 'pwz',
        # Progress
        'd_rally', 'delta_d_rally',
        # R1 state
        'r1_phase', 'r1_face', 'r1_fs',
        'r1_x', 'r1_y', 'r1_yaw',
        'r1_app_x', 'r1_app_y', 'r1_dist_to_app',
        'r1_wp_idx', 'r1_wp_total', 'r1_nav_steps',
        'r1_fx', 'r1_fy', 'r1_heading_err',
        'r1_vx', 'r1_vy',
        'd3_r1', 'd4_r1', 'r1_contact', 'face_change_r1',
        # R2 state
        'r2_phase', 'r2_face', 'r2_fs',
        'r2_x', 'r2_y', 'r2_yaw',
        'r2_app_x', 'r2_app_y', 'r2_dist_to_app',
        'r2_wp_idx', 'r2_wp_total', 'r2_nav_steps',
        'r2_fx', 'r2_fy', 'r2_heading_err',
        'r2_vx', 'r2_vy',
        'd3_r2', 'd4_r2', 'r2_contact', 'face_change_r2',
        # Cooperative force
        'both_pushing', 'Fx_net', 'Fy_net', 'F_along_goal', 'F_lateral',
        # Stall state
        'ema_progress', 'stall_counter',
        # Planner score landscape (updated every REPLAN_EVERY steps)
        'plan_best_score', 'plan_margin', 'plan_epsilon_held',
        # Adaptive speed scale (ratio: required_rate / ema_progress, clipped)
        'speed_scale',
    ]

    ema_progress     = 0.0; stall_counter = 0
    prev_payload     = None; stall_count = 0
    step             = 0; result = 'TIMEOUT'
    final_dist       = None; first_assign = None
    prev_d_rally     = None; speed_scale = 1.0
    plan_best_score  = 0.0; plan_margin = 0.0; plan_epsilon_held = 0

    with open(csv_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()

        while step < MAX_STEPS:
            sim.step()

            payload_pos = scene.get_position(sim, payload_h)
            rally_pos   = scene.get_position(sim, rally_h)
            d_rally     = scene.dist2d(payload_pos, rally_pos)
            px, py      = payload_pos[0], payload_pos[1]

            try:
                lin, ang = sim.getObjectVelocity(payload_h)
                pvx, pvy, pwz = lin[0], lin[1], ang[2]
            except Exception:
                pvx = pvy = pwz = 0.0

            payload_angle = get_payload_angle(sim, payload_h) - pay0_angle
            d3_r1, d4_r1  = read_sensors(sim, r1)
            d3_r2, d4_r2  = read_sensors(sim, r2)
            r1p = r1.get_position(); r2p = r2.get_position()
            r1_vx, r1_vy  = get_object_velocity_2d(sim, r1_h)
            r2_vx, r2_vy  = get_object_velocity_2d(sim, r2_h)

            if scene.payload_reached_rally_point(payload_pos, rally_pos):
                result = 'SUCCESS'; final_dist = d_rally
                print(f"  SUCCESS at step {step}"); break

            both_pushing = all(a.phase == 'PUSH' for a in [agent1, agent2])

            needs_replan = (step % REPLAN_EVERY == 0)
            if not needs_replan and both_pushing:
                needs_replan = not plan_viable(d_rally, ema_progress, step, MAX_STEPS)
            if needs_replan:
                robots_pos = [r1.get_position()[:2], r2.get_position()[:2]]
                plan = planner.plan(robots_pos, [px, py], [rx, ry])
                plan_best_score   = planner.last_best_score
                plan_margin       = planner.last_margin
                plan_epsilon_held = int(planner.last_epsilon_held)
                for _, (agent, p) in enumerate(zip([agent1, agent2], plan)):
                    agent.update_assignment(p)
                    if first_assign is None and p['face'] is not None:
                        fs = [plan[j]['force_scale'] for j in range(2)]
                        ratio = min(fs)/(max(fs)+1e-9) if max(fs) > 0 else 0
                        first_assign = {
                            'R1': plan[0]['face'], 'R2': plan[1]['face'],
                            'f': (plan[0]['force_scale'], plan[1]['force_scale']),
                            'ratio': ratio,
                        }

            if both_pushing and prev_payload is not None:
                dpx_ = px - prev_payload[0]; dpy_ = py - prev_payload[1]
                ux_, uy_ = _norm(rx-px, ry-py)
                prog          = dpx_*ux_ + dpy_*uy_
                ema_progress  = EMA_ALPHA*prog + (1-EMA_ALPHA)*ema_progress
                stall_counter = stall_counter+1 if ema_progress < MIN_PROGRESS else 0
                if stall_counter >= STALL_STEPS:
                    agent1.reset(); agent2.reset()
                    planner.reset(keep_locked=True)
                    ema_progress = 0.0; stall_counter = 0; stall_count += 1
                    print(f"  [STALL] step={step} d_rally={d_rally:.3f}m → reset #{stall_count}")
            prev_payload = (px, py) if both_pushing else None

            speed_scale = 1.0
            if both_pushing and ema_progress > MIN_PROGRESS * 2 and (MAX_STEPS - step) > 20:
                required_rate = d_rally / (MAX_STEPS - step)
                speed_scale   = max(0.70, min(SPEED_BOOST, required_rate / ema_progress))

            ph1 = agent1.step(d3_r1, d4_r1, speed_scale, both_pushing)
            ph2 = agent2.step(d3_r2, d4_r2, speed_scale, both_pushing)

            # Effective push directions (zeroed if not in PUSH)
            fx1, fy1 = agent1.effective_push(d3_r1, d4_r1) if ph1 == 'PUSH' else (0.0, 0.0)
            fx2, fy2 = agent2.effective_push(d3_r2, d4_r2) if ph2 == 'PUSH' else (0.0, 0.0)
            fs1 = agent1.assignment['force_scale'] if agent1.assignment else 0.0
            fs2 = agent2.assignment['force_scale'] if agent2.assignment else 0.0

            # Heading errors (nan if not pushing)
            pd1 = agent1.assignment['push_dir'] if agent1.assignment and ph1 == 'PUSH' else None
            pd2 = agent2.assignment['push_dir'] if agent2.assignment and ph2 == 'PUSH' else None
            he1 = compute_heading_error(r1, pd1)
            he2 = compute_heading_error(r2, pd2)

            # Approach targets and navigation distances
            app1   = agent1.assignment['approach'] if agent1.assignment else [float('nan')]*2
            app2   = agent2.assignment['approach'] if agent2.assignment else [float('nan')]*2
            d2app1 = _dist(r1p, app1) if agent1.assignment else float('nan')
            d2app2 = _dist(r2p, app2) if agent2.assignment else float('nan')
            wps1   = agent1.assignment['waypoints'] if agent1.assignment else []
            wps2   = agent2.assignment['waypoints'] if agent2.assignment else []

            # Net force and decomposition
            Fx_net   = fx1*fs1 + fx2*fs2
            Fy_net   = fy1*fs1 + fy2*fs2
            F_along  = Fx_net*F_des_x + Fy_net*F_des_y
            F_lat    = Fx_net*(-F_des_y) + Fy_net*F_des_x

            f1_label = agent1.assignment['face'] if agent1.assignment else '-'
            f2_label = agent2.assignment['face'] if agent2.assignment else '-'
            delta_d  = (d_rally - prev_d_rally) if prev_d_rally is not None else 0.0

            if step % LOG_EVERY == 0 and step > 0:
                print(f"  step={step:4d}  d_rally:{d_rally:.3f}m  "
                      f"R1:{ph1[:3]}|{f1_label or '-':3s}  "
                      f"R2:{ph2[:3]}|{f2_label or '-':3s}")

            writer.writerow({
                'step': step, 'scenario': scenario_name,
                'px': fmt(px), 'py': fmt(py),
                'payload_angle': fmt(payload_angle, 5),
                'pvx': fmt(pvx, 5), 'pvy': fmt(pvy, 5), 'pwz': fmt(pwz, 5),
                'd_rally': fmt(d_rally), 'delta_d_rally': fmt(delta_d, 5),
                'r1_phase': ph1, 'r1_face': f1_label or '-', 'r1_fs': fmt(fs1, 3),
                'r1_x': fmt(r1p[0], 3), 'r1_y': fmt(r1p[1], 3),
                'r1_yaw': fmt(r1.get_yaw(), 4),
                'r1_app_x': fmt(app1[0], 3), 'r1_app_y': fmt(app1[1], 3),
                'r1_dist_to_app': fmt(d2app1, 4),
                'r1_wp_idx': agent1.wp_idx, 'r1_wp_total': len(wps1),
                'r1_nav_steps': agent1.nav_steps,
                'r1_fx': fmt(fx1, 3), 'r1_fy': fmt(fy1, 3),
                'r1_heading_err': fmt(he1, 4),
                'r1_vx': fmt(r1_vx, 4), 'r1_vy': fmt(r1_vy, 4),
                'd3_r1': fmt(d3_r1, 3) if d3_r1 < 9 else 'inf',
                'd4_r1': fmt(d4_r1, 3) if d4_r1 < 9 else 'inf',
                'r1_contact': int(d3_r1 < DETECT_DIST and d4_r1 < DETECT_DIST),
                'face_change_r1': int(agent1.face_changed()),
                'r2_phase': ph2, 'r2_face': f2_label or '-', 'r2_fs': fmt(fs2, 3),
                'r2_x': fmt(r2p[0], 3), 'r2_y': fmt(r2p[1], 3),
                'r2_yaw': fmt(r2.get_yaw(), 4),
                'r2_app_x': fmt(app2[0], 3), 'r2_app_y': fmt(app2[1], 3),
                'r2_dist_to_app': fmt(d2app2, 4),
                'r2_wp_idx': agent2.wp_idx, 'r2_wp_total': len(wps2),
                'r2_nav_steps': agent2.nav_steps,
                'r2_fx': fmt(fx2, 3), 'r2_fy': fmt(fy2, 3),
                'r2_heading_err': fmt(he2, 4),
                'r2_vx': fmt(r2_vx, 4), 'r2_vy': fmt(r2_vy, 4),
                'd3_r2': fmt(d3_r2, 3) if d3_r2 < 9 else 'inf',
                'd4_r2': fmt(d4_r2, 3) if d4_r2 < 9 else 'inf',
                'r2_contact': int(d3_r2 < DETECT_DIST and d4_r2 < DETECT_DIST),
                'face_change_r2': int(agent2.face_changed()),
                'both_pushing': int(both_pushing),
                'Fx_net': fmt(Fx_net, 4), 'Fy_net': fmt(Fy_net, 4),
                'F_along_goal': fmt(F_along, 4), 'F_lateral': fmt(F_lat, 4),
                'ema_progress': fmt(ema_progress, 6),
                'stall_counter': stall_counter,
                'plan_best_score': fmt(plan_best_score, 5),
                'plan_margin': fmt(plan_margin, 5),
                'plan_epsilon_held': plan_epsilon_held,
                'speed_scale': fmt(speed_scale, 3),
            })

            prev_d_rally = d_rally
            step += 1

    r1.stop(); r2.stop()
    status = 'SUCCESS ✓' if result == 'SUCCESS' else 'TIMEOUT ✗'
    dist_str = f"{final_dist:.4f}m" if final_dist is not None else 'N/A'
    print(f"\n  {status}  steps={step}  stall_resets={stall_count}  final_dist={dist_str}")
    if first_assign:
        fa = first_assign
        print(f"    Assignment: R1→{fa['R1']}  R2→{fa['R2']}  "
              f"f=({fa['f'][0]:.3f},{fa['f'][1]:.3f})  ratio={fa['ratio']:.2f}")
    print(f"    CSV → {csv_path}")

    return {
        'scenario': scenario_name, 'rally': rally_xy, 'result': result,
        'steps': step, 'stalls': stall_count, 'dist': final_dist,
        'assign': first_assign, 'csv': csv_path,
    }


# ── Main ──────────────────────────────────────────────────────────

def main():
    ts       = datetime.now().strftime('%Y%m%d_%H%M%S')
    log_path = os.path.join(LOG_DIR, f'benchmark_{ts}.log')
    tee      = _Tee(log_path)
    try:
        print(f"=== PLANNER BENCHMARK — multi-scenario ===")
        print(f"Log: {log_path}\n")

        client    = RemoteAPIClient()
        sim       = client.getObject('sim')
        r1        = Robot(sim, '/p3dx_1', name='1')
        r2        = Robot(sim, '/p3dx_2', name='2')
        r1_h      = sim.getObject('/p3dx_1')
        r2_h      = sim.getObject('/p3dx_2')
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
                    sim, r1, r2, r1_h, r2_h, payload_h, rally_h,
                    agent1, agent2, name, (rx, ry), ts
                )
                results.append(res)

        except KeyboardInterrupt:
            print("\nInterrupted.")
            r1.stop(); r2.stop()
        finally:
            sim.stopSimulation()
            print("\nSimulation stopped.")

        print(f"\n{'='*70}")
        print(f"  BENCHMARK SUMMARY")
        print(f"{'='*70}")
        print(f"  {'Scenario':<20} {'Result':<10} {'Steps':>6} {'Stalls':>6} "
              f"{'Dist':>7}  Assignment")
        print(f"  {'─'*66}")
        for r in results:
            fa  = r['assign']
            asg = f"R1→{fa['R1']} R2→{fa['R2']} ratio={fa['ratio']:.2f}" if fa else '-'
            dist_str = f"{r['dist']:.4f}m" if r['dist'] else 'N/A'
            status = 'SUCCESS ✓' if r['result'] == 'SUCCESS' else 'TIMEOUT ✗'
            print(f"  {r['scenario']:<20} {status:<10} {r['steps']:>6} "
                  f"{r['stalls']:>6} {dist_str:>7}  {asg}")
        print(f"{'='*70}\n")

        print(f"CSVs saved:")
        for r in results:
            print(f"  {r['csv']}")
    finally:
        tee.close()


if __name__ == '__main__':
    main()
