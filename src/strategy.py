"""
strategy.py
State machine with shared occupancy map.

Phases:
  EXPLORING   — all agents map without prior knowledge
                robots sweep sectors with active sensors
                drone sweeps systematically
                any agent detecting payload → broadcast

  CONVERGING  — payload known, robots navigate to push positions
                drone maps corridor payload → rally_point

  PUSHING     — push with both robots in confirmed contact
                if either loses contact → reposition

  SUCCESS     — payload at rally point
"""

from enum import Enum, auto
import math
import scene
from .grid import OccupancyGrid


class Phase(Enum):
    EXPLORING  = auto()
    CONVERGING = auto()
    PUSHING    = auto()
    SUCCESS    = auto()


# Exploration waypoints — opposite corner spirals
# Robots have no prior knowledge of environment
EXPLORE_WP_1 = [
    [-1.8,-1.8],[0.0,-1.8],[1.8,-1.8],
    [1.8, 0.0],[1.8, 1.8],[0.0, 1.8],
    [-1.8,1.8],[-1.8,0.0],[-0.6,-0.6],
    [0.6,-0.6],[0.6, 0.6],[-0.6, 0.6],
    [0.0, 0.0],
]
EXPLORE_WP_2 = [
    [1.8, 1.8],[0.0, 1.8],[-1.8,1.8],
    [-1.8,0.0],[-1.8,-1.8],[0.0,-1.8],
    [1.8,-1.8],[1.8, 0.0],[0.6, 0.6],
    [-0.6,0.6],[-0.6,-0.6],[0.6,-0.6],
    [0.0, 0.0],
]


class Strategy:

    # Push positions — both behind payload, staggered in depth
    BACK_1      = 0.35   # metres behind payload — primary contact
    BACK_2      = 0.50   # metres behind payload — support
    SIDE_1      = +0.10  # lateral offset to avoid collision
    SIDE_2      = -0.10

    CONV_OK     = 0.30
    EXPLORE_SPD = 3.5
    APPROACH_SPD= 3.0
    PUSH_SPD    = 2.0

    def __init__(self, p3dx_1, p3dx_2, drone,
                 payload_h, rally_point_h, sim):
        self.p3dx_1        = p3dx_1
        self.p3dx_2        = p3dx_2
        self.drone         = drone
        self.sim           = sim
        self.payload_h     = payload_h
        self.rally_point_h = rally_point_h

        # Shared map — no agent knows it in advance
        self.map = OccupancyGrid(width=5.0, height=5.0, resolution=0.10)

        self.phase             = Phase.EXPLORING
        self.payload_known_pos = None
        self.payload_start     = None
        self._wp_1 = 0
        self._wp_2 = 0
        self.ux = self.uy = self.px = self.py = 0.0

            # CONVERGING: p3dx_2 avoids front side via _on_front_side() — no staging

    # ------------------------------------------------------------------

    def _update_vectors(self, rally_pos):
        self.ux, self.uy, self.px, self.py = scene.push_vector(
            self.payload_known_pos, rally_pos)

    def _pos_1(self, back, side):
        return scene.pusher_position(
            self.payload_known_pos, self.ux, self.uy, side, back)

    def _pos_2(self, back, side):
        return scene.pusher_position(
            self.payload_known_pos, self.ux, self.uy, side, back)

    # ------------------------------------------------------------------
    # Main cycle
    # ------------------------------------------------------------------

    def step(self) -> Phase:
        payload_real = scene.get_position(self.sim, self.payload_h)
        rally_pos    = scene.get_position(self.sim, self.rally_point_h)

        # Update map with robot positions
        p1 = self.p3dx_1.get_position()
        p2 = self.p3dx_2.get_position()
        self.map.mark_robot_path(p1[0], p1[1])
        self.map.mark_robot_path(p2[0], p2[1])

        # Update map with sensor readings — legacy read feeds the grid
        self.map.update_from_sensor(
            p1[0], p1[1], self.p3dx_1.get_yaw(),
            self.p3dx_1.read_sensors_legacy()
        )
        self.map.update_from_sensor(
            p2[0], p2[1], self.p3dx_2.get_yaw(),
            self.p3dx_2.read_sensors_legacy()
        )

        # Drone cycle
        drone_detected = self.drone.step(payload_real, self.map)

        # First detection by drone
        if drone_detected and self.payload_known_pos is None:
            self.payload_known_pos = self.drone.detected_pos[:]
            self.payload_start     = self.payload_known_pos[:]
            self._p2_stage         = 0
            self._p2_staging       = None
            self.phase = Phase.CONVERGING

        # Detection by robot sensor contact
        if self.payload_known_pos is None:
            contact_1, _ = self.p3dx_1.front_contact()
            contact_2, _ = self.p3dx_2.front_contact()
            if contact_1 or contact_2:
                self.payload_known_pos = payload_real[:]
                self.payload_start     = self.payload_known_pos[:]
                self._p2_stage         = 0
                self._p2_staging       = None
                self.phase = Phase.CONVERGING
                detector = "1" if contact_1 else "2"
                print(f"    [INFO] Payload detected by P3DX-{detector} "
                      f"via sensor at {self.payload_known_pos[:2]}")

        if self.payload_known_pos is not None:
            self.payload_known_pos = payload_real[:]
            self._update_vectors(rally_pos)

            if scene.payload_reached_rally_point(payload_real, rally_pos):
                self.p3dx_1.stop()
                self.p3dx_2.stop()
                self.phase = Phase.SUCCESS
                return self.phase

        if   self.phase == Phase.EXPLORING:  self._exploring()
        elif self.phase == Phase.CONVERGING: self._converging(rally_pos)
        elif self.phase == Phase.PUSHING:    self._pushing(rally_pos)

        return self.phase

    # ------------------------------------------------------------------
    # EXPLORING
    # ------------------------------------------------------------------

    def _exploring(self):
        self._explore_step(self.p3dx_1, EXPLORE_WP_1, '_wp_1')
        self._explore_step(self.p3dx_2, EXPLORE_WP_2, '_wp_2')

    def _explore_step(self, robot, wps, attr):
        idx  = getattr(self, attr)
        goal = wps[idx] + [0.0]
        if robot.is_at(goal, 0.30):
            idx = (idx + 1) % len(wps)
            setattr(self, attr, idx)
            goal = wps[idx] + [0.0]

        # Extended range during exploration — more time to react
        if robot.obstacle_ahead(robot.RANGE_EXTENDED):
            vl, vr = robot.braitenberg()
            robot.set_velocity(vl, vr)
        else:
            robot.drive_to(goal, self.EXPLORE_SPD)

    # ------------------------------------------------------------------
    # CONVERGING
    # ------------------------------------------------------------------

    def _on_front_side(self, robot_pos: list) -> bool:
        """
        True if robot is between payload and rally point (front side).
        Computed as dot product of (robot - payload) with push vector.
        """
        dx = robot_pos[0] - self.payload_known_pos[0]
        dy = robot_pos[1] - self.payload_known_pos[1]
        return (dx * self.ux + dy * self.uy) > 0.1

    def _lateral_bypass(self, robot_pos: list, side: float) -> list:
        """Bypass point to avoid crossing in front of payload."""
        return [
            self.payload_known_pos[0] - self.ux * 0.1 + self.px * side * 0.8,
            self.payload_known_pos[1] - self.uy * 0.1 + self.py * side * 0.8,
            0.0
        ]

    def _converging(self, rally_pos):
        pos_1 = self._pos_1(self.BACK_1 + 0.20, self.SIDE_1)
        pos_2 = self._pos_2(self.BACK_2 + 0.20, self.SIDE_2)

        p1 = self.p3dx_1.get_position()
        p2 = self.p3dx_2.get_position()

        contact_1, _ = self.p3dx_1.front_contact()
        contact_2, _ = self.p3dx_2.front_contact()
        at_1 = self.p3dx_1.is_at(pos_1, self.CONV_OK)
        at_2 = self.p3dx_2.is_at(pos_2, self.CONV_OK)

        # --- P3DX 1 ---
        if at_1 or (contact_1 and not self._on_front_side(p1)):
            self.p3dx_1.stop()
        elif self._on_front_side(p1):
            bypass = self._lateral_bypass(p1, side=+1.0)
            self.p3dx_1.drive_to(bypass, self.APPROACH_SPD)
        elif self.p3dx_1.obstacle_ahead() and not contact_1:
            vl, vr = self.p3dx_1.braitenberg()
            self.p3dx_1.set_velocity(vl, vr)
        else:
            self.p3dx_1.drive_to(pos_1, self.APPROACH_SPD)

        # --- P3DX 2 ---
        if at_2 or (contact_2 and not self._on_front_side(p2)):
            self.p3dx_2.stop()
        elif self._on_front_side(p2):
            bypass = self._lateral_bypass(p2, side=-1.0)
            self.p3dx_2.drive_to(bypass, self.APPROACH_SPD)
        elif self.p3dx_2.obstacle_ahead() and not contact_2:
            vl, vr = self.p3dx_2.braitenberg()
            self.p3dx_2.set_velocity(vl, vr)
        else:
            self.p3dx_2.drive_to(pos_2, self.APPROACH_SPD)

        if (at_1 or (contact_1 and not self._on_front_side(p1))) and \
           (at_2 or (contact_2 and not self._on_front_side(p2))):
            self.phase = Phase.PUSHING

    # ------------------------------------------------------------------
    # PUSHING
    # ------------------------------------------------------------------

    def _pushing(self, rally_pos):
        pos_1 = self._pos_1(self.BACK_1, self.SIDE_1)
        pos_2 = self._pos_2(self.BACK_2, self.SIDE_2)

        contact_1, _ = self.p3dx_1.front_contact()
        contact_2, _ = self.p3dx_2.front_contact()

        # Lost contact → approach faster; confirmed contact → maintain speed
        spd_1 = self.PUSH_SPD if contact_1 else self.PUSH_SPD * 1.5
        spd_2 = self.PUSH_SPD * 0.8 if contact_2 else self.PUSH_SPD

        self.p3dx_1.drive_to(pos_1, spd_1)
        self.p3dx_2.drive_to(pos_2, spd_2)

    # ------------------------------------------------------------------
    # Status log
    # ------------------------------------------------------------------

    def status(self, rally_pos) -> str:
        if self.payload_known_pos:
            d   = scene.dist2d(self.payload_known_pos, rally_pos)
            dev = scene.lateral_deviation(
                self.payload_known_pos, rally_pos,
                self.payload_start or self.payload_known_pos)
            payload_str = f"payload→rally:{d:.3f}m dev:{dev:+.3f}m"
        else:
            payload_str = "payload: UNKNOWN"

        p1 = self.p3dx_1.get_position()
        p2 = self.p3dx_2.get_position()
        c1, d1 = self.p3dx_1.front_contact()
        c2, d2 = self.p3dx_2.front_contact()
        return (
            f"[{self.phase.name:11s}] {payload_str} "
            f"P3DX-1:({p1[0]:.2f},{p1[1]:.2f})[{'C' if c1 else '-'}:{d1:.2f}] "
            f"P3DX-2:({p2[0]:.2f},{p2[1]:.2f})[{'C' if c2 else '-'}:{d2:.2f}] "
            f"map:{self.map.coverage_percent():.0f}%"
        )