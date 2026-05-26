"""
strategy.py
Multi-robot cooperative payload transport — local contact control (v4).

Architecture: no FSM, no role assignment, no synchronized phases.
Each robot makes an independent local decision every step:

  error_dir = normalize(Kp*(rally - payload) - Kd*payload_vel)
  alignment = dot(normalize(payload - robot), error_dir)

  if in_contact AND alignment > ALIGN_ENTER:
      push in error_dir at speed ∝ alignment
  else:
      navigate to approach_pos (behind payload, ±lateral offset)

Team coordination (conservative, 3 cases only):
  1. If vel_toward_goal > 0 → do nothing
  2. Stall detected (both in contact, no progress) → perturb worse-aligned robot
  3. One robot loses contact → other continues solo, aims directly at rally_point

Reference: architecture_proposal_v4.md
"""

import math
from enum import Enum, auto
from . import scene


class Phase(Enum):
    EXPLORING = auto()   # navigating toward payload, detecting
    ACTIVE    = auto()   # payload detected, approach + push
    SUCCESS   = auto()


class Strategy:

    # Speeds
    EXPLORE_SPD    = 2.0
    APPROACH_SPD   = 2.0
    PUSH_SPD       = 1.2
    PERTURB_SPD    = 0.8

    # Geometry
    APPROACH_DIST  = 0.60    # robot center to payload center, approach target
    SIDE_OFFSET    = 0.28    # lateral separation — wider keeps P2 from blocking

    # Contact
    CONTACT_DIST   = 0.30    # sensor threshold for "in contact"

    # Alignment hysteresis
    ALIGN_ENTER    = 0.30    # start pushing when alignment > this
    ALIGN_EXIT     = 0.10    # stop pushing when alignment drops below this

    # PD control on error direction
    Kp             = 1.0
    Kd             = 0.35
    VEL_WINDOW     = 6       # steps for raw velocity window
    EMA_ALPHA      = 0.15    # EMA smoothing factor (lower = smoother)

    # Stall detection
    STALL_WINDOW   = 80      # steps of no progress → perturbation (4s at 20Hz)
    STALL_EPS      = 0.0003  # accel threshold (m/step²) — near-zero = frictional equilibrium
    PERTURB_STEPS  = 20      # steps to back off during perturbation
    PERTURB_COOLDOWN = 250   # minimum steps between perturbations (~12s at 20Hz)

    # Structural role separation
    # p3dx_1 = LONGITUDINAL: pushes directly in error_dir (primary force)
    # p3dx_2 = LATERAL:      pushes perpendicular to correct drift (torque control)
    LATERAL_APPROACH_DIST = 0.55   # p3dx_2 approach distance from payload side
    LATERAL_PUSH_GAIN     = 3.0    # speed = GAIN * |dev|, capped at PUSH_SPD
    LATERAL_DEADBAND      = 0.02   # m — ignore drift smaller than this

    # Contribution estimator — retained for logging only
    CONTRIB_DECAY       = 0.995
    CONTRIB_SENSITIVITY = 10.0
    PUSH_SPD_MIN        = 0.40

    def __init__(self, team, sim):
        self.team = team
        self.sim  = sim

        self.phase             = Phase.EXPLORING
        self.payload_known_pos = None
        self.payload_start     = None

        # Per-robot push state (independent, no barrier synchronization)
        self._pushing = {'p3dx_1': False, 'p3dx_2': False}

        # Payload velocity history (for Kd term)
        self._payload_history = []

        # Stall detection (acceleration-based)
        self._stall_counter      = 0
        self._perturb_robot      = None
        self._perturb_steps      = 0
        self._perturb_cooldown   = 0             # steps before next perturbation allowed
        self._last_dist_to_rally = None
        self._prev_payload_vel   = (0.0, 0.0)   # for acceleration estimate

        # EMA-smoothed velocity (Fix 1)
        self._ema_vx = 0.0
        self._ema_vy = 0.0

        # Contribution estimator — projected work accumulated per robot
        self._contrib = {'p3dx_1': 0.0, 'p3dx_2': 0.0}

    # ------------------------------------------------------------------
    # Accessors
    # ------------------------------------------------------------------

    @property
    def p3dx_1(self):
        return self.team.get('p3dx_1')

    @property
    def p3dx_2(self):
        return self.team.get('p3dx_2')

    # ------------------------------------------------------------------
    # Error direction (PD damped)
    # ------------------------------------------------------------------

    # Drift correction
    K_LATERAL      = 0.40    # correction gain for growing lateral drift

    def _error_dir(self, rally_pos: list) -> tuple:
        """
        Damped + drift-corrected error direction.
        error = Kp*(rally-payload) - Kd*vel - K_lat*dev*perp
        Returns normalized (ex, ey).
        """
        p  = self.payload_known_pos
        ex = self.Kp * (rally_pos[0] - p[0])
        ey = self.Kp * (rally_pos[1] - p[1])

        # Kd: EMA-smoothed velocity damping (Fix 1)
        # Raw finite difference is noisy — EMA reduces jitter in error_dir
        ex -= self.Kd * self._ema_vx
        ey -= self.Kd * self._ema_vy

        # Lateral drift correction: steer back toward ideal trajectory
        if self.payload_start is not None:
            dev = scene.lateral_deviation(p, rally_pos, self.payload_start)
            # perpendicular to raw push direction
            raw_norm = math.sqrt((rally_pos[0]-p[0])**2 + (rally_pos[1]-p[1])**2) + 1e-9
            px = -(rally_pos[1]-p[1]) / raw_norm
            py =  (rally_pos[0]-p[0]) / raw_norm
            ex -= self.K_LATERAL * dev * px
            ey -= self.K_LATERAL * dev * py

        norm = math.sqrt(ex*ex + ey*ey) + 1e-9
        return ex/norm, ey/norm

    # ------------------------------------------------------------------
    # Geometry helpers
    # ------------------------------------------------------------------

    def _approach_pos(self, robot_name: str, error_dir: tuple) -> list:
        """
        Structural role approach positions:
          p3dx_1 (longitudinal): directly behind payload along error_dir
          p3dx_2 (lateral):      to the side of payload along perpendicular
        """
        p      = self.payload_known_pos
        ux, uy = error_dir
        px, py = -uy, ux   # perpendicular (90° CCW)

        if robot_name == 'p3dx_1':
            # Behind payload — primary push direction
            return [
                p[0] - ux * self.APPROACH_DIST,
                p[1] - uy * self.APPROACH_DIST,
                0.0
            ]
        else:
            # To the side — approach from +perp direction (corrects -dev drift)
            # Side chosen to oppose current drift: if dev < 0 (drifted left),
            # p3dx_2 approaches from the left (px side) to push right
            return [
                p[0] - px * self.LATERAL_APPROACH_DIST,
                p[1] - py * self.LATERAL_APPROACH_DIST,
                0.0
            ]

    def _alignment(self, robot, error_dir: tuple) -> float:
        """
        dot(contact_dir, error_dir)
        contact_dir = normalize(payload - robot_pos)
        +1 = robot directly behind payload pushing toward goal
         0 = perpendicular
        -1 = robot on wrong side (would push away)
        """
        pos  = robot.get_position()
        p    = self.payload_known_pos
        dx   = p[0] - pos[0]
        dy   = p[1] - pos[1]
        norm = math.sqrt(dx*dx + dy*dy) + 1e-9
        return (dx/norm) * error_dir[0] + (dy/norm) * error_dir[1]

    # ------------------------------------------------------------------
    # Contact sensing
    # ------------------------------------------------------------------

    def _in_contact(self, robot) -> bool:
        """True if any frontal sensor detects payload within CONTACT_DIST."""
        for i in range(8):
            if i >= len(robot.sensors):
                continue
            try:
                res = self.sim.readProximitySensor(robot.sensors[i])
                if res[0] > 0 and res[1] < self.CONTACT_DIST:
                    if self.team.is_payload(int(res[3])):
                        return True
            except Exception:
                pass
        return False

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

    def _sensor_contact_with_payload(self, robot) -> tuple:
        """For status display only. Returns (contact_bool, min_dist)."""
        min_dist = float('inf')
        contact  = False
        for idx in [3, 4]:
            if idx >= len(robot.sensors):
                continue
            try:
                res = self.sim.readProximitySensor(robot.sensors[idx])
                if res[0] > 0 and self.team.is_payload(int(res[3])):
                    contact = True
                    if res[1] < min_dist:
                        min_dist = res[1]
            except Exception:
                pass
        return contact, min_dist

    # ------------------------------------------------------------------
    # Contribution estimator
    # ------------------------------------------------------------------

    def _update_contributions(self, error_dir: tuple, contacts: dict):
        """
        Accumulate projected work per robot each step.

        progress = dot(payload_vel_ema, error_dir)  — useful motion this step
        Each robot in contact gets credit weighted by 1/n_contact,
        so cooperation is not penalised relative to solo pushing.
        EMA decay keeps recent behaviour dominant.
        """
        progress  = (self._ema_vx * error_dir[0]
                     + self._ema_vy * error_dir[1])
        n_contact = sum(1 for c in contacts.values() if c)
        weight    = progress / max(n_contact, 1)

        for name in ['p3dx_1', 'p3dx_2']:
            self._contrib[name] *= self.CONTRIB_DECAY
            if contacts[name]:
                self._contrib[name] += weight

    def _push_speed_for(self, robot_name: str) -> float:
        """
        Scale push speed by relative contribution.
        Better contributor → faster; worse → slower (never below PUSH_SPD_MIN).
        tanh of contribution difference gives smooth, bounded scaling.
        """
        other  = 'p3dx_2' if robot_name == 'p3dx_1' else 'p3dx_1'
        diff   = self._contrib[robot_name] - self._contrib[other]
        # factor ∈ (0, 2) centred at 1.0 when contributions are equal
        factor = 1.0 + math.tanh(diff * self.CONTRIB_SENSITIVITY)
        return max(self.PUSH_SPD_MIN, self.PUSH_SPD * factor)

    # ------------------------------------------------------------------
    # Velocity push
    # ------------------------------------------------------------------

    def _push_by_velocity(self, robot, heading: float, speed: float):
        """
        Velocity control: align heading, apply forward speed.
        No waypoint chasing — sustained force application.
        """
        yaw = robot.get_yaw()
        err = math.atan2(math.sin(heading - yaw), math.cos(heading - yaw))
        angular = max(-1.5, min(1.5, 3.0 * err))
        robot.set_velocity(speed - angular, speed + angular)

    # ------------------------------------------------------------------
    # Payload history
    # ------------------------------------------------------------------

    def _update_payload_history(self):
        p = self.payload_known_pos
        self._payload_history.append((p[0], p[1]))
        if len(self._payload_history) > self.VEL_WINDOW:
            self._payload_history.pop(0)

        # Update EMA velocity each step (Fix 1)
        if len(self._payload_history) >= 2:
            n      = max(len(self._payload_history) - 1, 1)
            raw_vx = (self._payload_history[-1][0] - self._payload_history[0][0]) / n
            raw_vy = (self._payload_history[-1][1] - self._payload_history[0][1]) / n
            self._ema_vx = self.EMA_ALPHA * raw_vx + (1 - self.EMA_ALPHA) * self._ema_vx
            self._ema_vy = self.EMA_ALPHA * raw_vy + (1 - self.EMA_ALPHA) * self._ema_vy

    # ------------------------------------------------------------------
    # Stall detection
    # ------------------------------------------------------------------

    def _update_stall(self, rally_pos: list, error_dir: tuple):
        """
        FIX 3: acceleration-based stall detection.
        Stall = both in contact + payload not accelerating under applied force.
        Uses velocity delta (accel proxy) instead of position progress counter.
        """
        # Current payload velocity estimate
        if len(self._payload_history) < 2:
            return
        n   = max(len(self._payload_history) - 1, 1)
        vx  = (self._payload_history[-1][0] - self._payload_history[0][0]) / n
        vy  = (self._payload_history[-1][1] - self._payload_history[0][1]) / n

        # Acceleration proxy: change in velocity from previous step
        ax = vx - self._prev_payload_vel[0]
        ay = vy - self._prev_payload_vel[1]
        accel_norm = math.sqrt(ax*ax + ay*ay)
        self._prev_payload_vel = (vx, vy)

        c1 = self._in_contact(self.p3dx_1)
        c2 = self._in_contact(self.p3dx_2)

        # Stall: both in contact, near-zero acceleration (frictional equilibrium)
        if c1 and c2 and accel_norm < self.STALL_EPS:
            self._stall_counter += 1
        else:
            self._stall_counter = 0

        # Fix 3: cooldown prevents rapid successive perturbations
        if self._perturb_cooldown > 0:
            self._perturb_cooldown -= 1

        if (self._stall_counter >= self.STALL_WINDOW
                and self._perturb_steps == 0
                and self._perturb_cooldown == 0):
            a1 = self._alignment(self.p3dx_1, error_dir)
            a2 = self._alignment(self.p3dx_2, error_dir)
            self._perturb_robot    = 'p3dx_1' if a1 <= a2 else 'p3dx_2'
            self._perturb_steps    = self.PERTURB_STEPS
            self._perturb_cooldown = self.PERTURB_COOLDOWN
            self._stall_counter    = 0
            self._pushing[self._perturb_robot] = False
            print(f"    [STALL] Perturbing {self._perturb_robot} "
                  f"(a1={a1:.2f} a2={a2:.2f} accel={accel_norm:.5f})")

    # ------------------------------------------------------------------
    # Per-robot step
    # ------------------------------------------------------------------

    def _step_longitudinal(self, robot_name: str, error_dir: tuple,
                            rally_pos: list, n_pushing: int):
        """
        p3dx_1 role: push directly in error_dir (longitudinal force).
        Approaches from behind payload, applies full PUSH_SPD when in contact.
        """
        robot = self.team.get(robot_name)

        if self._perturb_robot == robot_name and self._perturb_steps > 0:
            ux, uy = error_dir
            self._push_by_velocity(robot, math.atan2(-uy, -ux), self.PERTURB_SPD)
            self._perturb_steps -= 1
            if self._perturb_steps == 0:
                self._perturb_robot = None
                self._pushing[robot_name] = False
            return

        in_contact = self._in_contact(robot)
        alignment  = self._alignment(robot, error_dir)

        if in_contact and alignment > 0.0:
            self._pushing[robot_name] = True
            if n_pushing <= 1:
                p = self.payload_known_pos
                push_heading = math.atan2(rally_pos[1]-p[1], rally_pos[0]-p[0])
            else:
                push_heading = math.atan2(error_dir[1], error_dir[0])
            self._push_by_velocity(robot, push_heading,
                                   self._push_speed_for(robot_name))
        else:
            self._pushing[robot_name] = False
            if self._wall_ahead(robot):
                vl, vr = robot.braitenberg()
                robot.set_velocity(vl, vr)
            else:
                robot.drive_to(self._approach_pos(robot_name, error_dir),
                               self.APPROACH_SPD)

    def _step_lateral(self, robot_name: str, error_dir: tuple, dev: float):
        """
        p3dx_2 role: lateral stabilization (torque control).
        Approaches from the side, pushes perpendicular to error_dir
        to correct drift. Speed proportional to deviation magnitude.
        When dev ≈ 0 (on track), maintains light contact — conserves energy.
        """
        robot  = self.team.get(robot_name)
        ux, uy = error_dir
        px, py = -uy, ux   # perpendicular

        if self._perturb_robot == robot_name and self._perturb_steps > 0:
            self._push_by_velocity(robot, math.atan2(-uy, -ux), self.PERTURB_SPD)
            self._perturb_steps -= 1
            if self._perturb_steps == 0:
                self._perturb_robot = None
                self._pushing[robot_name] = False
            return

        in_contact = self._in_contact(robot)

        if in_contact:
            if abs(dev) > self.LATERAL_DEADBAND:
                # dev > 0 = drifted left (+px direction) → push right (-px)
                correction_dir = (-px, -py) if dev > 0 else (px, py)
                push_heading   = math.atan2(correction_dir[1], correction_dir[0])
                speed = min(self.PUSH_SPD, self.LATERAL_PUSH_GAIN * abs(dev))
                self._pushing[robot_name] = True
                self._push_by_velocity(robot, push_heading, speed)
            else:
                # On track — light contact push in error_dir
                self._pushing[robot_name] = True
                self._push_by_velocity(robot,
                                       math.atan2(error_dir[1], error_dir[0]),
                                       self.PUSH_SPD_MIN)
        else:
            self._pushing[robot_name] = False
            if self._wall_ahead(robot):
                vl, vr = robot.braitenberg()
                robot.set_velocity(vl, vr)
            else:
                robot.drive_to(self._approach_pos(robot_name, error_dir),
                               self.APPROACH_SPD)


    def step(self) -> Phase:
        payload_real = scene.get_position(self.sim, self.team.payload_h)
        rally_pos    = scene.get_position(self.sim, self.team.rally_point_h)

        self.team.update_map()

        if self.payload_known_pos is None:
            self._detect(payload_real)

        if self.payload_known_pos is not None:
            self.payload_known_pos = payload_real[:]
            self._update_payload_history()

            if scene.payload_reached_rally_point(payload_real, rally_pos):
                self.p3dx_1.stop()
                self.p3dx_2.stop()
                self.phase = Phase.SUCCESS
                return self.phase

        if   self.phase == Phase.EXPLORING: self._exploring(rally_pos)
        elif self.phase == Phase.ACTIVE:    self._active(rally_pos)

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
                        self._last_dist_to_rally = None
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
                robot.drive_to(rally_pos, self.EXPLORE_SPD)

    def _active(self, rally_pos: list):
        error_dir = self._error_dir(rally_pos)

        dev = scene.lateral_deviation(
            self.payload_known_pos, rally_pos,
            self.payload_start or self.payload_known_pos)

        n_pushing = sum(
            1 for n in ['p3dx_1', 'p3dx_2']
            if self._pushing[n] and self._in_contact(self.team.get(n))
        )

        contacts = {
            'p3dx_1': self._in_contact(self.p3dx_1),
            'p3dx_2': self._in_contact(self.p3dx_2),
        }
        self._update_contributions(error_dir, contacts)

        # p3dx_1: longitudinal push in error_dir
        self._step_longitudinal('p3dx_1', error_dir, rally_pos, n_pushing)

        # p3dx_2: lateral stabilization perpendicular to push
        self._step_lateral('p3dx_2', error_dir, dev)

        self._update_stall(rally_pos, error_dir)

    # ------------------------------------------------------------------
    # Status
    # ------------------------------------------------------------------

    def status(self, rally_pos) -> str:
        if self.payload_known_pos:
            d   = scene.dist2d(self.payload_known_pos, rally_pos)
            dev = scene.lateral_deviation(
                self.payload_known_pos, rally_pos,
                self.payload_start or self.payload_known_pos)
            # Show push state + winner indicator in log
            states = '/'.join(
                'P' if self._pushing.get(n) else 'A'
                for n in ['p3dx_1', 'p3dx_2']
            )
            c1s = f"{self._contrib.get('p3dx_1', 0):.3f}"
            c2s = f"{self._contrib.get('p3dx_2', 0):.3f}"
            payload_str = (f"payload→rally:{d:.3f}m dev:{dev:+.3f}m "
                           f"[{states}] stall:{self._stall_counter} "
                           f"c:[{c1s}/{c2s}]")
        else:
            payload_str = "payload:UNKNOWN"

        p1 = self.p3dx_1.get_position()
        p2 = self.p3dx_2.get_position()
        c1, d1 = self._sensor_contact_with_payload(self.p3dx_1)
        c2, d2 = self._sensor_contact_with_payload(self.p3dx_2)

        return (
            f"[{self.phase.name:9s}] {payload_str} "
            f"P1:({p1[0]:.2f},{p1[1]:.2f})[{'C' if c1 else '-'}:{d1:.2f}] "
            f"P2:({p2[0]:.2f},{p2[1]:.2f})[{'C' if c2 else '-'}:{d2:.2f}] "
            f"map:{self.team.map.coverage_percent():.0f}%"
        )