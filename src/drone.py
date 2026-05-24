"""
drone.py
Drone class with direct position control and active mapping.

Drone phases:
  PATROL   — sweeps the area in a vertical grid searching for payload
             updates the shared map with its position
  CORRIDOR — detected payload, maps corridor payload → rally_point
             ensures path is clear before pushing
  HOVER    — corridor mapped, hovers over payload as reference
"""

import math


class Drone:

    DETECTION_RADIUS   = 0.35
    PATROL_HEIGHT      = 1.5
    MOVE_SPEED         = 0.08
    WAYPOINT_THRESHOLD = 0.15

    def __init__(self, sim, base_path: str = '/drone'):
        self.sim    = sim
        self.handle = sim.getObject(base_path)

        self.detected     = False
        self.detected_pos = None

        self._corridor_wps  = []
        self._corridor_idx  = 0
        self._corridor_done = False

        self._wp_index  = 0
        self._waypoints = self._build_waypoints()

        wp = self._waypoints[0]
        self.sim.setObjectPosition(
            self.handle, -1, [wp[0], wp[1], self.PATROL_HEIGHT])

    def _build_waypoints(self) -> list:
        wps = []
        x_passes = [-2.0, -1.2, -0.4, 0.0, 0.4, 1.2, 2.0]
        for i, x in enumerate(x_passes):
            if i % 2 == 0:
                wps.append([x, -2.0])
                wps.append([x,  2.0])
            else:
                wps.append([x,  2.0])
                wps.append([x, -2.0])
        return wps

    def _build_corridor_waypoints(self, payload_pos: list,
                                   rally_pos: list) -> list:
        """Waypoints along the corridor payload → rally_point."""
        dx   = rally_pos[0] - payload_pos[0]
        dy   = rally_pos[1] - payload_pos[1]
        dist = math.sqrt(dx*dx + dy*dy)
        if dist < 0.01:
            return []

        steps = max(int(dist / 0.4), 2)
        wps   = []
        for i in range(1, steps + 1):
            t = i / steps
            wps.append([
                payload_pos[0] + t * dx,
                payload_pos[1] + t * dy
            ])
        return wps

    def _set_pos(self, x: float, y: float):
        self.sim.setObjectPosition(
            self.handle, -1, [x, y, self.PATROL_HEIGHT])

    def get_position(self) -> list:
        return self.sim.getObjectPosition(self.handle, -1)

    def _dist2d_to_wp(self, wp: list) -> float:
        pos = self.get_position()
        return math.sqrt((pos[0]-wp[0])**2 + (pos[1]-wp[1])**2)

    def step(self, payload_real_pos: list, grid=None) -> bool:
        pos = self.get_position()

        if grid is not None:
            grid.mark_free(pos[0], pos[1])

        # --- PATROL ---
        if not self.detected:
            wp = self._waypoints[self._wp_index]
            if self._dist2d_to_wp(wp) < self.WAYPOINT_THRESHOLD:
                self._wp_index = (self._wp_index + 1) % len(self._waypoints)
                wp = self._waypoints[self._wp_index]

            cur = self.get_position()
            dx  = wp[0] - cur[0]
            dy  = wp[1] - cur[1]
            d   = math.sqrt(dx*dx + dy*dy)
            if d < self.MOVE_SPEED:
                nx, ny = wp[0], wp[1]
            else:
                nx = cur[0] + (dx/d) * self.MOVE_SPEED
                ny = cur[1] + (dy/d) * self.MOVE_SPEED
            self._set_pos(nx, ny)

            dx2 = nx - payload_real_pos[0]
            dy2 = ny - payload_real_pos[1]
            if math.sqrt(dx2*dx2 + dy2*dy2) < self.DETECTION_RADIUS:
                self.detected     = True
                self.detected_pos = [payload_real_pos[0],
                                     payload_real_pos[1], 0.0]
            return self.detected

        # --- CORRIDOR ---
        if not self._corridor_done:
            if not self._corridor_wps:
                self._corridor_done = True
                return True

            if self._corridor_idx < len(self._corridor_wps):
                wp  = self._corridor_wps[self._corridor_idx]
                cur = self.get_position()
                dx  = wp[0] - cur[0]
                dy  = wp[1] - cur[1]
                d   = math.sqrt(dx*dx + dy*dy)
                if d < self.WAYPOINT_THRESHOLD:
                    if grid is not None:
                        grid.mark_free(wp[0], wp[1])
                    self._corridor_idx += 1
                else:
                    if d < self.MOVE_SPEED:
                        nx, ny = wp[0], wp[1]
                    else:
                        nx = cur[0] + (dx/d) * self.MOVE_SPEED
                        ny = cur[1] + (dy/d) * self.MOVE_SPEED
                    self._set_pos(nx, ny)
                    if grid is not None:
                        grid.mark_free(nx, ny)
            else:
                self._corridor_done = True
            return True

        # --- HOVER over payload ---
        self._set_pos(self.detected_pos[0], self.detected_pos[1])
        return True

    def set_corridor(self, payload_pos: list, rally_pos: list):
        """Called by strategy when payload and rally point are known."""
        self._corridor_wps  = self._build_corridor_waypoints(
            payload_pos, rally_pos)
        self._corridor_idx  = 0
        self._corridor_done = False

    def corridor_ready(self) -> bool:
        return self._corridor_done

    def status(self) -> str:
        pos = self.get_position()
        if not self.detected:
            wp = self._waypoints[self._wp_index]
            return (f"Drone PATROL wp[{self._wp_index}]"
                    f"({wp[0]:.1f},{wp[1]:.1f}) "
                    f"pos({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f})")
        if not self._corridor_done:
            return (f"Drone CORRIDOR [{self._corridor_idx}/"
                    f"{len(self._corridor_wps)}] "
                    f"pos({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f})")
        return (f"Drone HOVER over payload "
                f"pos({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f})")