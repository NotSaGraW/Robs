"""
robot.py
Robot class for Pioneer P3DX in CoppeliaSim via ZMQ Remote API.
Encapsulates handles, motors, sensors and control.

Perception system:
  Two reading modes using sim.checkProximitySensorEx — the sensor hardware
  never changes, only the detection threshold of each query:

  NORMAL   (0.5m) — exploration, wall following, standard navigation
  EXTENDED (1.0m) — repositioning, open-space detection, phase transitions

  Active sensor groups depend on movement direction:
  FORWARD  — frontal sensors [0-7]
  BACKWARD — rear sensors [8-15]
  ALL      — all 16 sensors
  CONTACT  — near-frontal sensors [3,4] only (push confirmation)
"""

import math


class Robot:
    # Pioneer P3DX physical geometry
    WIDTH       = 0.415
    LENGTH      = 0.519
    HALF_LENGTH = 0.260

    # Control
    MAX_SPEED    = 5.0
    ANGULAR_GAIN = 2.2
    ANGULAR_LIMIT= 1.5
    DEAD_ANGLE   = 0.9
    ARRIVAL_THR  = 0.15

    # Perception ranges
    RANGE_NORMAL   = 0.5   # standard navigation
    RANGE_EXTENDED = 1.0   # repositioning, open space, phase transitions

    # Contact threshold — robot touching payload
    CONTACT_DIST = 0.30

    # Sensor index groups
    SENSORS_FORWARD  = list(range(8))        # [0-7]  frontal
    SENSORS_BACKWARD = list(range(8, 16))    # [8-15] rear
    SENSORS_ALL      = list(range(16))       # all 16
    SENSORS_CONTACT  = [3, 4]                # near-frontal only

    # Detection mode: front+back face detection
    _DETECT_MODE = 3

    def __init__(self, sim, base_path: str, name: str):
        self.sim       = sim
        self.name      = name
        self.base_path = base_path

        self.handle      = sim.getObject(base_path)
        self.left_motor  = sim.getObject(f'{base_path}/leftMotor')
        self.right_motor = sim.getObject(f'{base_path}/rightMotor')

        self.sensors = []
        self._setup_sensors()

    # ------------------------------------------------------------------
    # Sensor setup
    # ------------------------------------------------------------------

    def _setup_sensors(self):
        """
        Gets handles for the 16 ultrasonic sensors.
        Detection collection is already configured in Pioneer Lua script.
        """
        for i in range(16):
            try:
                h = self.sim.getObject(
                    f'{self.base_path}/ultrasonicSensor',
                    {'index': i}
                )
                self.sensors.append(h)
            except Exception:
                pass

    # ------------------------------------------------------------------
    # State
    # ------------------------------------------------------------------

    def get_position(self) -> list:
        return self.sim.getObjectPosition(self.handle, -1)

    def get_orientation(self) -> list:
        return self.sim.getObjectOrientation(self.handle, -1)

    def get_yaw(self) -> float:
        return self.sim.getObjectOrientation(self.handle, -1)[2]

    # ------------------------------------------------------------------
    # Actuation
    # ------------------------------------------------------------------

    def set_velocity(self, left: float, right: float):
        self.sim.setJointTargetVelocity(self.left_motor, left)
        self.sim.setJointTargetVelocity(self.right_motor, right)

    def stop(self):
        self.set_velocity(0.0, 0.0)

    # ------------------------------------------------------------------
    # Navigation control
    # ------------------------------------------------------------------

    def compute_control(self, goal: list, max_speed: float = None) -> tuple:
        """Proportional unicycle controller → wheel velocities."""
        if max_speed is None:
            max_speed = self.MAX_SPEED

        pos = self.get_position()
        yaw = self.get_yaw()

        dx   = goal[0] - pos[0]
        dy   = goal[1] - pos[1]
        dist = math.sqrt(dx*dx + dy*dy)

        if dist < self.ARRIVAL_THR:
            return 0.0, 0.0

        desired = math.atan2(dy, dx)
        error   = math.atan2(
            math.sin(desired - yaw),
            math.cos(desired - yaw)
        )

        w = max(-self.ANGULAR_LIMIT,
                min(self.ANGULAR_LIMIT, self.ANGULAR_GAIN * error))

        if abs(error) > self.DEAD_ANGLE:
            v = 0.0
        else:
            v  = 0.15 + (max_speed - 0.15) * (1.0 - abs(error) / self.DEAD_ANGLE)
            v *= (1.0 - math.exp(-2.0 * dist))

        left  = max(-max_speed, min(max_speed, v - w))
        right = max(-max_speed, min(max_speed, v + w))
        return left, right

    def drive_to(self, goal: list, max_speed: float = None):
        vl, vr = self.compute_control(goal, max_speed)
        self.set_velocity(vl, vr)
        return vl, vr

    def is_at(self, goal: list, threshold: float = None) -> bool:
        if threshold is None:
            threshold = self.ARRIVAL_THR
        pos = self.get_position()
        dx  = goal[0] - pos[0]
        dy  = goal[1] - pos[1]
        return (dx*dx + dy*dy) < threshold*threshold

    # ------------------------------------------------------------------
    # Perception — adaptive sensor reading
    #
    # Uses sim.checkProximitySensorEx so the sensor hardware never changes.
    # The detection threshold is specified per query:
    #   - NORMAL   (0.5m): standard navigation
    #   - EXTENDED (1.0m): repositioning, open space, phase transitions
    #
    # Active sensors depend on movement direction:
    #   - FORWARD:  frontal sensors [0-7]
    #   - BACKWARD: rear sensors    [8-15]
    #   - ALL:      all 16
    #   - CONTACT:  near-frontal    [3,4]
    # ------------------------------------------------------------------

    def read_sensors(self, indices: list = None,
                     range_m: float = None) -> list:
        """
        Read sensors using checkProximitySensorEx.

        indices: list of sensor indices to read (default: SENSORS_FORWARD)
        range_m: detection threshold in metres (default: RANGE_NORMAL)

        Returns list of (detected: bool, distance: float, sensor_index: int)
        for each queried sensor.
        """
        if indices is None:
            indices = self.SENSORS_FORWARD
        if range_m is None:
            range_m = self.RANGE_NORMAL

        results = []
        for idx in indices:
            if idx >= len(self.sensors):
                results.append((False, float('inf'), idx))
                continue
            try:
                res = self.sim.checkProximitySensorEx(
                    self.sensors[idx],
                    self.sim.handle_all,
                    self._DETECT_MODE,
                    range_m,
                    math.pi / 4   # 45° max angle — matches cone aperture
                )
                detected = res[0] == 1
                dist     = res[1] if detected else float('inf')
                results.append((detected, dist, idx))
            except Exception:
                results.append((False, float('inf'), idx))
        return results

    def read_sensors_legacy(self) -> list:
        """
        Legacy read using readProximitySensor (Lua-driven, 0.5m range).
        Used for Braitenberg and grid map updates — preserves existing behaviour.
        Returns list of (detected: bool, distance: float) for all 16 sensors.
        """
        results = []
        for h in self.sensors:
            try:
                res      = self.sim.readProximitySensor(h)
                detected = res[0] > 0
                dist     = res[1] if detected else float('inf')
                results.append((detected, dist))
            except Exception:
                results.append((False, float('inf')))
        return results

    # ------------------------------------------------------------------
    # Contextual perception queries
    # ------------------------------------------------------------------

    def obstacle_ahead(self, threshold: float = None) -> bool:
        """
        True if any frontal sensor detects something within threshold.
        Uses NORMAL range by default; pass RANGE_EXTENDED for wider sweep.
        """
        if threshold is None:
            threshold = self.RANGE_NORMAL
        readings = self.read_sensors(self.SENSORS_FORWARD, threshold)
        return any(detected for detected, _, _ in readings)

    def obstacle_behind(self, threshold: float = None) -> bool:
        """True if any rear sensor detects something within threshold."""
        if threshold is None:
            threshold = self.RANGE_NORMAL
        readings = self.read_sensors(self.SENSORS_BACKWARD, threshold)
        return any(detected for detected, _, _ in readings)

    def area_clear(self) -> bool:
        """
        True if no obstacle detected at extended range in any direction.
        Used before repositioning or phase transitions.
        """
        readings = self.read_sensors(self.SENSORS_ALL, self.RANGE_EXTENDED)
        return not any(detected for detected, _, _ in readings)

    def front_contact(self) -> tuple:
        """
        Returns (contact: bool, min_dist: float).
        contact=True if near-frontal sensors [3,4] detect something < CONTACT_DIST.
        Uses minimal threshold — confirms physical contact with payload.
        """
        readings = self.read_sensors(self.SENSORS_CONTACT, self.CONTACT_DIST)
        if not readings:
            return False, float('inf')

        min_dist = float('inf')
        contact  = False
        for detected, dist, _ in readings:
            if detected:
                if dist < min_dist:
                    min_dist = dist
                contact = True

        return contact, min_dist

    def nearest_obstacle(self, indices: list = None,
                          range_m: float = None) -> tuple:
        """
        Returns (detected: bool, distance: float, sensor_index: int)
        for the nearest detected obstacle in the given sensor group.
        """
        if indices is None:
            indices = self.SENSORS_FORWARD
        if range_m is None:
            range_m = self.RANGE_NORMAL

        readings = self.read_sensors(indices, range_m)
        detected_readings = [(d, i) for det, d, i in readings if det]

        if not detected_readings:
            return False, float('inf'), -1

        dist, idx = min(detected_readings, key=lambda x: x[0])
        return True, dist, idx

    # ------------------------------------------------------------------
    # Reactive obstacle avoidance — Braitenberg
    # Weights from the official Pioneer P3DX Lua script.
    # Uses legacy read (0.5m Lua-driven) for consistent behaviour.
    # ------------------------------------------------------------------

    BRAITENBERG_L = [-0.2,-0.4,-0.6,-0.8,-1.0,-1.2,-1.4,-1.6,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    BRAITENBERG_R = [-1.6,-1.4,-1.2,-1.0,-0.8,-0.6,-0.4,-0.2,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    BRAITENBERG_V0        = 2.0
    BRAITENBERG_NO_DETECT = 0.5
    BRAITENBERG_MAX_DETECT= 0.2

    def braitenberg(self) -> tuple:
        """
        Braitenberg obstacle avoidance.
        Returns (vLeft, vRight) in rad/s.
        """
        detect   = [0.0] * 16
        readings = self.read_sensors_legacy()

        for i, (detected, dist) in enumerate(readings):
            if detected and dist < self.BRAITENBERG_NO_DETECT:
                d = max(dist, self.BRAITENBERG_MAX_DETECT)
                detect[i] = 1.0 - (
                    (d - self.BRAITENBERG_MAX_DETECT) /
                    (self.BRAITENBERG_NO_DETECT - self.BRAITENBERG_MAX_DETECT)
                )

        v_left  = self.BRAITENBERG_V0
        v_right = self.BRAITENBERG_V0
        for i in range(16):
            v_left  += self.BRAITENBERG_L[i] * detect[i]
            v_right += self.BRAITENBERG_R[i] * detect[i]

        return v_left, v_right

    def __repr__(self):
        pos = self.get_position()
        return f"Robot({self.name}, pos=({pos[0]:.2f},{pos[1]:.2f}))"