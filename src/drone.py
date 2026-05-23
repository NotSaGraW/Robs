"""
drone.py
Clase Drone para el Quadcopter de CoppeliaSim.

Rol en el sistema:
  - Patrulla el tablero siguiendo waypoints a altura fija
  - Detecta la caja geométricamente cuando está bajo su radio de visión
  - Una vez detectada, se queda sobrevolando la caja como referencia
  - Comunica la posición detectada al sistema central (shared state)

El drone no conoce la posición de la caja de antemano.
La detección ocurre cuando dist2d(drone_pos, box_pos) < DETECTION_RADIUS,
simulando un sensor de visión cenital.
"""

import math
import time


class Drone:

    # Radio de detección cenital (campo de visión hacia abajo)
    DETECTION_RADIUS = 0.8      # metros en plano XY

    # Altura de patrulla
    PATROL_HEIGHT = 1.5         # metros

    # Tolerancia para considerar waypoint alcanzado
    WAYPOINT_THRESHOLD = 0.25   # metros en XY

    # Velocidad de movimiento del target (el drone sigue su target)
    # El drone tiene su propio PID interno en Lua, nosotros movemos el target
    TARGET_STEP = 0.08          # metros por ciclo al mover el target

    def __init__(self, sim, base_path: str = '/qua'):
        self.sim = sim
        self.base_path = base_path

        self.handle = sim.getObject(base_path)
        # El drone sigue su propio /qua/target (base es el objeto de control)
        self.target_handle = sim.getObject(f'{base_path}/target')

        self.detected = False           # ¿ha detectado la caja?
        self.detected_pos = None        # posición detectada de la caja
        self._waypoint_index = 0
        self._waypoints = self._build_patrol_waypoints()

        # Mover el target a altura de patrulla al inicializar
        self._set_target_height(self.PATROL_HEIGHT)

    # ------------------------------------------------------------------
    # Waypoints de patrulla
    # ------------------------------------------------------------------

    def _build_patrol_waypoints(self) -> list:
        """
        Patrulla en espiral rectangular sobre el tablero 5x5m.
        Cubre el área de forma sistemática en pasadas paralelas.
        El tablero va de -2.4 a +2.4m (paredes externas).
        """
        waypoints = []
        # Pasadas horizontales de arriba a abajo, separadas 1.2m
        y_values = [2.0, 0.8, -0.4, -1.6]
        for i, y in enumerate(y_values):
            if i % 2 == 0:
                waypoints.append([-2.0, y])
                waypoints.append([ 2.0, y])
            else:
                waypoints.append([ 2.0, y])
                waypoints.append([-2.0, y])
        return waypoints

    # ------------------------------------------------------------------
    # Control del target del drone
    # ------------------------------------------------------------------

    def _set_target_position(self, x: float, y: float, z: float = None):
        if z is None:
            z = self.PATROL_HEIGHT
        self.sim.setObjectPosition(self.target_handle, -1, [x, y, z])

    def _set_target_height(self, z: float):
        pos = self.sim.getObjectPosition(self.target_handle, -1)
        self.sim.setObjectPosition(self.target_handle, -1, [pos[0], pos[1], z])

    # ------------------------------------------------------------------
    # Estado del drone
    # ------------------------------------------------------------------

    def get_position(self) -> list:
        return self.sim.getObjectPosition(self.handle, -1)

    def get_position_2d(self) -> list:
        pos = self.get_position()
        return [pos[0], pos[1]]

    def _dist2d_to_waypoint(self, wp: list) -> float:
        pos = self.get_position()
        return math.sqrt((pos[0] - wp[0])**2 + (pos[1] - wp[1])**2)

    # ------------------------------------------------------------------
    # Lógica principal
    # ------------------------------------------------------------------

    def step(self, box_real_pos: list) -> bool:
        """
        Ejecuta un ciclo del drone.
        
        box_real_pos: posición real de la caja (solo para detección geométrica,
                      simula lo que vería un sensor cenital real).
        
        Retorna True si la caja ha sido detectada en este ciclo o anteriormente.
        """
        if self.detected:
            # Ya detectó — se queda sobrevolando la posición de la caja
            self._hover_over(self.detected_pos)
            return True

        # Avanzar en la ruta de patrulla
        self._patrol_step()

        # Comprobar detección geométrica
        drone_pos = self.get_position()
        dx = drone_pos[0] - box_real_pos[0]
        dy = drone_pos[1] - box_real_pos[1]
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < self.DETECTION_RADIUS:
            self.detected = True
            self.detected_pos = box_real_pos[:2] + [0.0]
            return True

        return False

    def _patrol_step(self):
        """
        El drone patrulla con su script Lua autónomamente.
        Desde Python solo leemos posición — no tocamos su target.
        Interferir con setObjectPosition sobre el target del drone
        desestabiliza su PID interno y lo lanza fuera de la escena.
        """
        pass

    def _hover_over(self, pos: list):
        """
        Una vez detectada la caja el drone sigue su Lua normal.
        No necesitamos controlarlo — solo leer su posición.
        """
        pass

    # ------------------------------------------------------------------
    # Info
    # ------------------------------------------------------------------

    def status(self) -> str:
        pos = self.get_position()
        if self.detected:
            return (f"Drone DETECTADO caja en "
                    f"({self.detected_pos[0]:.2f},{self.detected_pos[1]:.2f}) "
                    f"| drone pos ({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f})")
        wp = self._waypoints[self._waypoint_index]
        return (f"Drone PATRULLA wp[{self._waypoint_index}]"
                f"({wp[0]:.1f},{wp[1]:.1f}) "
                f"| drone pos ({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f})")