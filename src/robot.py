"""
robot.py
Clase Robot para Pioneer P3DX en CoppeliaSim vía ZMQ Remote API.

Mejora clave respecto a versión anterior:
  Los sensores ultrasónicos se configuran con una colección de detección
  (igual que el script Lua del Pioneer) para que funcionen desde Python.
"""

import math


class Robot:
    # Geometría física del Pioneer P3DX
    WIDTH        = 0.415    # metros
    LENGTH       = 0.519    # metros
    HALF_LENGTH  = 0.260    # metros

    # Control
    MAX_SPEED    = 5.0      # rad/s
    ANGULAR_GAIN = 2.2
    ANGULAR_LIMIT= 1.5
    DEAD_ANGLE   = 0.9      # rad
    ARRIVAL_THR  = 0.15     # metros

    # Sensores — índices frontales del Pioneer P3DX
    # Sensores 3,4,5 (índices 2,3,4) son los más frontales
    FRONT_SENSORS = [2, 3, 4]
    CONTACT_DIST  = 0.35    # metros — robot tocando la caja

    def __init__(self, sim, base_path: str, name: str):
        self.sim       = sim
        self.name      = name
        self.base_path = base_path

        self.handle      = sim.getObject(base_path)
        self.left_motor  = sim.getObject(f'{base_path}/leftMotor')
        self.right_motor = sim.getObject(f'{base_path}/rightMotor')

        # Configurar sensores con colección de detección
        # (replica exactamente lo que hace el Lua del Pioneer)
        self.sensors = []
        self._setup_sensors()

    # ------------------------------------------------------------------
    # Configuración de sensores (clave del rediseño)
    # ------------------------------------------------------------------

    def _setup_sensors(self):
        """
        Obtiene los handles de los 16 sensores ultrasónicos.
        La colección de detección ya está configurada en el Lua del Pioneer.
        No modificamos la configuración desde Python para evitar crashes
        en CoppeliaSim v4.10 con createCollection vía API externa.
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
    # Estado
    # ------------------------------------------------------------------

    def get_position(self) -> list:
        return self.sim.getObjectPosition(self.handle, -1)

    def get_yaw(self) -> float:
        return self.sim.getObjectOrientation(self.handle, -1)[2]

    # ------------------------------------------------------------------
    # Actuación
    # ------------------------------------------------------------------

    def set_velocity(self, left: float, right: float):
        self.sim.setJointTargetVelocity(self.left_motor, left)
        self.sim.setJointTargetVelocity(self.right_motor, right)

    def stop(self):
        self.set_velocity(0.0, 0.0)

    # ------------------------------------------------------------------
    # Control navegación
    # ------------------------------------------------------------------

    def compute_control(self, goal: list, max_speed: float = None) -> tuple:
        if max_speed is None:
            max_speed = self.MAX_SPEED

        pos = self.get_position()
        yaw = self.get_yaw()

        dx = goal[0] - pos[0]
        dy = goal[1] - pos[1]
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
    # Sensores
    # ------------------------------------------------------------------

    def read_sensors(self) -> list:
        """Retorna lista de (detected: bool, distance: float) para 16 sensores."""
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

    def front_contact(self) -> tuple:
        """
        Retorna (contact: bool, min_dist: float) para los sensores frontales.
        contact=True si algún sensor frontal detecta algo a < CONTACT_DIST.
        """
        readings = self.read_sensors()
        if not readings:
            return False, float('inf')

        min_dist = float('inf')
        contact  = False
        for idx in self.FRONT_SENSORS:
            if idx < len(readings):
                detected, dist = readings[idx]
                if detected and dist < min_dist:
                    min_dist = dist
                if detected and dist < self.CONTACT_DIST:
                    contact = True

        return contact, min_dist

    def any_front_obstacle(self, threshold: float = 0.5) -> bool:
        """True si cualquier sensor frontal detecta algo antes de threshold."""
        readings = self.read_sensors()
        for idx in self.FRONT_SENSORS:
            if idx < len(readings):
                detected, dist = readings[idx]
                if detected and dist < threshold:
                    return True
        return False

    # ------------------------------------------------------------------
    # Evasión reactiva — Braitenberg (igual que el Lua del Pioneer)
    # Pesos extraídos directamente del script Lua oficial del P3DX
    # ------------------------------------------------------------------

    BRAITENBERG_L = [-0.2,-0.4,-0.6,-0.8,-1.0,-1.2,-1.4,-1.6,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    BRAITENBERG_R = [-1.6,-1.4,-1.2,-1.0,-0.8,-0.6,-0.4,-0.2,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    BRAITENBERG_V0       = 2.0   # rad/s — velocidad base
    BRAITENBERG_NO_DETECT= 0.5   # metros — distancia sin detección
    BRAITENBERG_MAX_DETECT= 0.2  # metros — distancia máxima de detección

    def braitenberg(self) -> tuple:
        """
        Controlador Braitenberg para evasión de obstáculos.
        Replica exactamente el algoritmo del script Lua del Pioneer P3DX.
        Retorna (vLeft, vRight) en rad/s.
        """
        detect = [0.0] * 16
        readings = self.read_sensors()

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

    def obstacle_ahead(self, threshold: float = 0.45) -> bool:
        """True si algún sensor frontal (0-7) detecta algo antes de threshold."""
        readings = self.read_sensors()
        for i in range(8):
            if i < len(readings):
                detected, dist = readings[i]
                if detected and dist < threshold:
                    return True
        return False

    def __repr__(self):
        pos = self.get_position()
        return f"Robot({self.name}, pos=({pos[0]:.2f},{pos[1]:.2f}))"