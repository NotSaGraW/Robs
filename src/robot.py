"""
robot.py
Clase Robot para Pioneer P3DX en CoppeliaSim vía ZMQ Remote API.
Encapsula handles, motores, sensores y control.
"""

import math


class Robot:
    # Geometría física del Pioneer P3DX
    WIDTH = 0.415       # metros, distancia entre ruedas
    LENGTH = 0.519      # metros, largo del chasis
    WHEEL_RADIUS = 0.0975  # metros

    # Parámetros de control
    MAX_SPEED = 3.0         # rad/s máximo en motores
    PUSH_SPEED = 0.8        # rad/s durante empuje (lento y controlado)
    APPROACH_SPEED = 2.0    # rad/s durante aproximación
    ANGULAR_GAIN = 2.2
    ANGULAR_LIMIT = 1.5
    DEAD_ANGLE = 0.9        # rad — zona muerta angular (no avanza si gira mucho)
    ARRIVAL_THRESHOLD = 0.12  # metros — se considera en posición

    def __init__(self, sim, base_path: str, name: str):
        self.sim = sim
        self.name = name
        self.base_path = base_path

        # Handles principales
        self.handle = sim.getObject(base_path)
        self.left_motor = sim.getObject(f"{base_path}/leftMotor")
        self.right_motor = sim.getObject(f"{base_path}/rightMotor")

        # Sensores ultrasónicos (16)
        self.sensors = []
        for i in range(16):
            try:
                h = sim.getObject(f"{base_path}/ultrasonicSensor", {"index": i})
                self.sensors.append(h)
            except Exception:
                pass  # Si alguno falla, continúa sin él

    # ------------------------------------------------------------------
    # Estado
    # ------------------------------------------------------------------

    def get_position(self) -> list:
        """Posición [x, y, z] en coordenadas mundo."""
        return self.sim.getObjectPosition(self.handle, -1)

    def get_orientation(self) -> list:
        """Orientación [alpha, beta, gamma] en radianes."""
        return self.sim.getObjectOrientation(self.handle, -1)

    def get_yaw(self) -> float:
        """Ángulo yaw (rotación en Z) en radianes."""
        return self.get_orientation()[2]

    # ------------------------------------------------------------------
    # Actuación
    # ------------------------------------------------------------------

    def set_velocity(self, left: float, right: float):
        self.sim.setJointTargetVelocity(self.left_motor, left)
        self.sim.setJointTargetVelocity(self.right_motor, right)

    def stop(self):
        self.set_velocity(0.0, 0.0)

    # ------------------------------------------------------------------
    # Control
    # ------------------------------------------------------------------

    def compute_control(self, goal: list, max_speed: float = None) -> tuple:
        """
        Controlador proporcional unicycle → velocidades ruedas.
        Retorna (vLeft, vRight).
        """
        if max_speed is None:
            max_speed = self.MAX_SPEED

        pos = self.get_position()
        yaw = self.get_yaw()

        dx = goal[0] - pos[0]
        dy = goal[1] - pos[1]
        distance = math.sqrt(dx * dx + dy * dy)

        # Ya estamos en posición
        if distance < self.ARRIVAL_THRESHOLD:
            return 0.0, 0.0

        desired_angle = math.atan2(dy, dx)
        error = math.atan2(
            math.sin(desired_angle - yaw),
            math.cos(desired_angle - yaw)
        )

        # Control angular amortiguado
        w = self.ANGULAR_GAIN * error
        w = max(-self.ANGULAR_LIMIT, min(self.ANGULAR_LIMIT, w))

        # Velocidad lineal condicionada al error angular
        if abs(error) > self.DEAD_ANGLE:
            v = 0.0  # Gira en sitio si el error es grande
        else:
            # Rampa suave: más rápido cuando está alineado y lejos
            v = 0.15 + (max_speed - 0.15) * (1.0 - abs(error) / self.DEAD_ANGLE)
            v *= (1.0 - math.exp(-2.0 * distance))  # Suaviza arranque

        left = max(-max_speed, min(max_speed, v - w))
        right = max(-max_speed, min(max_speed, v + w))

        return left, right

    def drive_to(self, goal: list, max_speed: float = None):
        """Calcula control y aplica velocidades en un paso."""
        vl, vr = self.compute_control(goal, max_speed)
        self.set_velocity(vl, vr)
        return vl, vr

    def is_at(self, goal: list, threshold: float = None) -> bool:
        """True si el robot está dentro del umbral de la posición goal."""
        if threshold is None:
            threshold = self.ARRIVAL_THRESHOLD
        pos = self.get_position()
        dx = goal[0] - pos[0]
        dy = goal[1] - pos[1]
        return (dx * dx + dy * dy) < threshold * threshold

    # ------------------------------------------------------------------
    # Sensores
    # ------------------------------------------------------------------

    def read_sensors(self) -> list:
        """
        Lee los 16 sensores ultrasónicos.
        Retorna lista de (detected: bool, distance: float).
        """
        results = []
        for sensor in self.sensors:
            try:
                res = self.sim.readProximitySensor(sensor)
                detected = res[0] > 0
                distance = res[1] if detected else float('inf')
                results.append((detected, distance))
            except Exception:
                results.append((False, float('inf')))
        return results

    def min_front_distance(self) -> float:
        """Distancia mínima detectada por los 8 sensores frontales."""
        readings = self.read_sensors()
        front = readings[:8]
        distances = [d for _, d in front]
        return min(distances) if distances else float('inf')

    def __repr__(self):
        pos = self.get_position()
        return f"Robot({self.name}, pos=({pos[0]:.2f}, {pos[1]:.2f}))"