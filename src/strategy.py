"""
strategy.py
Multi-robot cooperative payload transport.

EXPLORING phase: both robots navigate toward rally point until payload detected.
ACTIVE phase: ContactPlanner assigns faces; per-robot agents navigate then push.

Cooperative force requirement:
  F_des = normalize(payload → rally)
  NNLS: min ||f0*n0 + f1*n1 - F_des||²  s.t. f0,f1 ≥ 0
  n0, n1 = push directions; f0, f1 ≥ 0 (push only).
  Both robots push simultaneously — forces compose to move payload toward goal.
"""

import math
from enum import Enum, auto
from . import scene
from .planner import ContactPlanner, _dist, _norm


class Phase(Enum):
    EXPLORING = auto()
    ACTIVE    = auto()
    SUCCESS   = auto()


EXPLORE_SPD       = 2.0
APPROACH_SPD      = 2.0
PUSH_SPD          = 2.0
POSITION_THR      = 0.15
K_CENTER          = 3.0
DETECT_DIST       = 0.80
REPLAN_EVERY      = 10
EMA_ALPHA         = 0.15
MIN_PROGRESS      = 0.0002
STALL_STEPS       = 100
NAV_TIMEOUT       = 400
CONTACT_STALL_THR = 30
BACKOFF_STEPS     = 25
ROBOT_MOVE_THR    = 0.005
VIABLE_MARGIN     = 0.80
SPEED_BOOST       = 1.30
ACTIVE_MAX_STEPS  = 5000


def _plan_viable(d_rally, ema_progress, step, max_steps):
    if ema_progress < MIN_PROGRESS * 2:
        return True
    return (d_rally / ema_progress) < (max_steps - step) * VIABLE_MARGIN


class _RobotAgent:
    """Per-robot NAVIGATE → PUSH → BACKOFF state machine."""

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
        # robot.stop() intentionally omitted — causes CoppeliaSim physics freeze

    def update_assignment(self, assignment):
        self._prev_face  = self.assignment['face'] if self.assignment else None
        face_changed     = (self.assignment is None or
                            self.assignment['face'] != assignment['face'])
        if face_changed:
            self.phase               = 'NAVIGATE'
            self.wp_idx              = 0
            self.nav_steps           = 0
            self._contact_stall_ctr  = 0
            self._backoff_ctr        = 0
            self._last_rpos          = None
            self.assignment = assignment
        elif self.phase in ('PUSH', 'BACKOFF'):
            self.assignment = assignment
        else:
            # NAVIGATE: freeze approach+waypoints — without this, every replan
            # shifts the approach with the payload and robot chases moving target
            self.assignment = {
                'face':        assignment['face'],
                'push_dir':    assignment['push_dir'],
                'force_scale': assignment['force_scale'],
                'score':       assignment['score'],
                'approach':    self.assignment['approach'],
                'waypoints':   self.assignment['waypoints'],
            }

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
                if _dist(rpos, wp) < POSITION_THR:
                    self.wp_idx += 1
                else:
                    self.robot.drive_to([wp[0], wp[1], 0.0], APPROACH_SPD)
                return 'NAVIGATE'
            app = asgn['approach']
            if _dist(rpos, app) < POSITION_THR:
                self.phase = 'PUSH'
                self.nav_steps = 0
                return 'PUSH'
            self.robot.drive_to([app[0], app[1], 0.0], APPROACH_SPD)
            return 'NAVIGATE'

        if self.phase == 'PUSH':
            in_contact = (d3 < DETECT_DIST and d4 < DETECT_DIST)
            if in_contact and both_pushing:
                moved = (_dist(rpos, self._last_rpos)
                         if self._last_rpos is not None else ROBOT_MOVE_THR + 1)
                self._contact_stall_ctr = (self._contact_stall_ctr + 1
                                           if moved < ROBOT_MOVE_THR else 0)
            else:
                self._contact_stall_ctr = 0
            self._last_rpos = list(rpos)
            if self._contact_stall_ctr >= CONTACT_STALL_THR:
                self._contact_stall_ctr = 0
                self._backoff_ctr       = BACKOFF_STEPS
                self.phase              = 'BACKOFF'
                self.robot.set_velocity(-APPROACH_SPD * 0.5, -APPROACH_SPD * 0.5)
                return 'BACKOFF'
            pd = asgn['push_dir']
            fx, fy = float(pd[0]), float(pd[1])
            if d3 < DETECT_DIST and d4 < DETECT_DIST:
                ux, uy         = fx, fy
                perp_x, perp_y = -uy, ux
                c              = K_CENTER * (d4 - d3)
                fx = ux + perp_x * c
                fy = uy + perp_y * c
                n  = math.sqrt(fx*fx + fy*fy) + 1e-9
                fx, fy = fx/n, fy/n
            speed = PUSH_SPD * max(0.3, asgn['force_scale']) * speed_scale
            hdg = math.atan2(fy, fx)
            yaw = self.robot.get_yaw()
            err = math.atan2(math.sin(hdg - yaw), math.cos(hdg - yaw))
            w   = max(-1.5, min(1.5, 3.0 * err))
            self.robot.set_velocity(speed - w, speed + w)
            return 'PUSH'

        if self.phase == 'BACKOFF':
            self._backoff_ctr -= 1
            self.robot.set_velocity(-APPROACH_SPD * 0.5, -APPROACH_SPD * 0.5)
            if self._backoff_ctr <= 0:
                self.phase = 'NAVIGATE'
                self.nav_steps = 0
                self.wp_idx = 0
            return 'BACKOFF'

        self.robot.stop()
        return 'IDLE'


class Strategy:

    def __init__(self, team, sim):
        self.team = team
        self.sim  = sim

        self.phase             = Phase.EXPLORING
        self.payload_known_pos = None
        self.payload_start     = None

        self._planner = ContactPlanner()
        self._agents  = [
            _RobotAgent(team.get('p3dx_1'), 'R1'),
            _RobotAgent(team.get('p3dx_2'), 'R2'),
        ]

        self._active_step   = 0
        self._ema_progress  = 0.0
        self._stall_counter = 0
        self._stall_count   = 0
        self._prev_payload  = None

    # ------------------------------------------------------------------
    # Sensors
    # ------------------------------------------------------------------

    def _read_sensors(self, robot):
        """Read frontal sensors d3, d4 for centering correction."""
        d3 = float('inf')
        d4 = float('inf')
        if len(robot.sensors) > 3:
            try:
                res = self.sim.readProximitySensor(robot.sensors[3])
                if res[0] > 0:
                    d3 = float(res[1])
            except Exception:
                pass
        if len(robot.sensors) > 4:
            try:
                res = self.sim.readProximitySensor(robot.sensors[4])
                if res[0] > 0:
                    d4 = float(res[1])
            except Exception:
                pass
        return d3, d4

    def _wall_ahead(self, robot, threshold: float = 0.40) -> bool:
        for i in range(8):
            if i >= len(robot.sensors):
                continue
            try:
                res = self.sim.readProximitySensor(robot.sensors[i])
                if res[0] > 0 and res[1] < threshold:
                    if self.team.identify(int(res[3])) == 'wall':
                        return True
            except Exception:
                pass
        return False

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def step(self) -> Phase:
        payload_real = scene.get_position(self.sim, self.team.payload_h)
        rally_pos    = scene.get_position(self.sim, self.team.rally_point_h)

        self.team.update_map()

        if self.payload_known_pos is None:
            self._detect(payload_real)

        if self.payload_known_pos is not None:
            self.payload_known_pos = payload_real[:]

            if scene.payload_reached_rally_point(payload_real, rally_pos):
                for agent in self._agents:
                    agent.robot.stop()
                self.phase = Phase.SUCCESS
                return self.phase

        if   self.phase == Phase.EXPLORING: self._exploring(rally_pos)
        elif self.phase == Phase.ACTIVE:    self._active(payload_real, rally_pos)

        return self.phase

    def _detect(self, payload_real):
        for name in ['p3dx_1', 'p3dx_2']:
            robot = self.team.get(name)
            for i, h in enumerate(robot.sensors):
                try:
                    res = self.sim.readProximitySensor(h)
                    if res[0] > 0 and self.team.is_payload(int(res[3])):
                        self.payload_known_pos = payload_real[:]
                        self.payload_start     = payload_real[:]
                        self._active_step      = 0
                        self.phase = Phase.ACTIVE
                        print(f"    [{name.upper()}] Payload detected "
                              f"via sensor [{i}] at {payload_real[:2]}")
                        return
                except Exception:
                    pass

    def _exploring(self, rally_pos):
        for robot in self.team.all_robots():
            if self._wall_ahead(robot):
                vl, vr = robot.braitenberg()
                robot.set_velocity(vl, vr)
            else:
                robot.drive_to(rally_pos, EXPLORE_SPD)

    def _active(self, payload_real, rally_pos):
        px, py   = payload_real[0], payload_real[1]
        rally_2d = rally_pos[:2]
        d_rally  = scene.dist2d(payload_real, rally_pos)

        both_pushing = all(a.phase == 'PUSH' for a in self._agents)

        needs_replan = (self._active_step % REPLAN_EVERY == 0)
        if not needs_replan and both_pushing:
            needs_replan = not _plan_viable(
                d_rally, self._ema_progress, self._active_step, ACTIVE_MAX_STEPS)
        if needs_replan:
            robots_pos = [
                self._agents[0].robot.get_position()[:2],
                self._agents[1].robot.get_position()[:2],
            ]
            plan = self._planner.plan(robots_pos, [px, py], rally_2d)
            for agent, assignment in zip(self._agents, plan):
                old_face = agent.assignment['face'] if agent.assignment else None
                agent.update_assignment(assignment)
                if old_face != assignment['face'] and assignment['face'] is not None:
                    pd  = assignment['push_dir']
                    nav = ('direct' if not assignment['waypoints']
                           else str([(round(w[0], 2), round(w[1], 2))
                                     for w in assignment['waypoints']]))
                    print(f"  [{agent.name}] face={assignment['face']:3s}  "
                          f"push=({pd[0]:+.3f},{pd[1]:+.3f})  "
                          f"f={assignment['force_scale']:.3f}  nav={nav}")

        if both_pushing and self._prev_payload is not None:
            ux, uy   = _norm(rally_2d[0] - px, rally_2d[1] - py)
            progress = ((px - self._prev_payload[0]) * ux
                        + (py - self._prev_payload[1]) * uy)
            self._ema_progress  = EMA_ALPHA * progress + (1 - EMA_ALPHA) * self._ema_progress
            self._stall_counter = (self._stall_counter + 1
                                   if self._ema_progress < MIN_PROGRESS else 0)

            if self._stall_counter >= STALL_STEPS:
                print(f"\n  [STALL] step={self._active_step}  "
                      f"ema={self._ema_progress:.5f} → reset")
                for a in self._agents:
                    a.reset()
                self._planner.reset(keep_locked=True)
                self._ema_progress  = 0.0
                self._stall_counter = 0
                self._stall_count  += 1

        self._prev_payload = (px, py) if both_pushing else None

        speed_scale = 1.0
        if (both_pushing and self._ema_progress > MIN_PROGRESS * 2
                and (ACTIVE_MAX_STEPS - self._active_step) > 20):
            required_rate = d_rally / (ACTIVE_MAX_STEPS - self._active_step)
            speed_scale   = max(0.70, min(SPEED_BOOST,
                                          required_rate / self._ema_progress))

        d3_1, d4_1 = self._read_sensors(self._agents[0].robot)
        d3_2, d4_2 = self._read_sensors(self._agents[1].robot)
        self._agents[0].step(d3_1, d4_1, speed_scale, both_pushing)
        self._agents[1].step(d3_2, d4_2, speed_scale, both_pushing)

        self._active_step += 1

    # ------------------------------------------------------------------
    # Status
    # ------------------------------------------------------------------

    def status(self, rally_pos) -> str:
        if self.payload_known_pos:
            d   = scene.dist2d(self.payload_known_pos, rally_pos)
            dev = scene.lateral_deviation(
                self.payload_known_pos, rally_pos,
                self.payload_start or self.payload_known_pos)
            phases = '/'.join(a.phase[0] for a in self._agents)
            faces  = '/'.join(
                (a.assignment['face'] or '-') if a.assignment else '-'
                for a in self._agents)
            payload_str = (f"payload->rally:{d:.3f}m dev:{dev:+.3f}m "
                           f"[{phases}|{faces}] stall:{self._stall_counter}")
        else:
            payload_str = "payload:UNKNOWN"

        r1 = self._agents[0].robot
        r2 = self._agents[1].robot
        p1 = r1.get_position()
        p2 = r2.get_position()
        d3_1, d4_1 = self._read_sensors(r1)
        d3_2, d4_2 = self._read_sensors(r2)

        return (
            f"[{self.phase.name:9s}] {payload_str} "
            f"P1:({p1[0]:.2f},{p1[1]:.2f})[d3:{d3_1:.2f}|d4:{d4_1:.2f}] "
            f"P2:({p2[0]:.2f},{p2[1]:.2f})[d3:{d3_2:.2f}|d4:{d4_2:.2f}] "
            f"map:{self.team.map.coverage_percent():.0f}%"
        )
