"""
drone.py
<<<<<<< HEAD
Drone con control directo de posición y mapeo activo.

Fases del drone:
  PATROL   → barre el tablero en cuadrícula vertical buscando la caja
             actualiza el mapa compartido con su posición
  CORRIDOR → detectó la caja, mapea el corredor box→target
             garantiza que el camino está despejado
  HOVER    → corredor mapeado, sobrevuela la caja como referencia
"""

import math
=======
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
>>>>>>> 5f2f97eed1f565038878880bd489cd56631980e4


class Drone:

<<<<<<< HEAD
    DETECTION_RADIUS   = 0.35
    PATROL_HEIGHT      = 1.5
    MOVE_SPEED         = 0.08
    WAYPOINT_THRESHOLD = 0.15

    def __init__(self, sim, base_path: str = '/qua'):
        self.sim    = sim
        self.handle = sim.getObject(base_path)

        self.detected     = False
        self.detected_pos = None

        # Estado de corredor
        self._corridor_wps    = []
        self._corridor_idx    = 0
        self._corridor_done   = False

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

    def _build_corridor_waypoints(self, box_pos: list,
                                   target_pos: list) -> list:
        """
        Genera waypoints a lo largo del corredor box→target
        para que el drone lo mapee antes del empuje.
        """
        dx   = target_pos[0] - box_pos[0]
        dy   = target_pos[1] - box_pos[1]
        dist = math.sqrt(dx*dx + dy*dy)
        if dist < 0.01:
            return []

        steps = max(int(dist / 0.4), 2)
        wps   = []
        for i in range(1, steps + 1):
            t = i / steps
            wps.append([
                box_pos[0] + t * dx,
                box_pos[1] + t * dy
            ])
        return wps

    def _set_pos(self, x: float, y: float):
        self.sim.setObjectPosition(
            self.handle, -1, [x, y, self.PATROL_HEIGHT])
=======
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
>>>>>>> 5f2f97eed1f565038878880bd489cd56631980e4

    def get_position(self) -> list:
        return self.sim.getObjectPosition(self.handle, -1)

<<<<<<< HEAD
    def _dist2d_to_wp(self, wp: list) -> float:
        pos = self.get_position()
        return math.sqrt((pos[0]-wp[0])**2 + (pos[1]-wp[1])**2)

    def step(self, box_real_pos: list, grid=None) -> bool:
        pos = self.get_position()

        # Actualizar mapa con posición del drone
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

            dx2 = nx - box_real_pos[0]
            dy2 = ny - box_real_pos[1]
            if math.sqrt(dx2*dx2 + dy2*dy2) < self.DETECTION_RADIUS:
                self.detected     = True
                self.detected_pos = [box_real_pos[0], box_real_pos[1], 0.0]
            return self.detected

        # --- CORRIDOR ---
        if not self._corridor_done:
            if not self._corridor_wps:
                # Construir waypoints del corredor la primera vez
                # Necesitamos target — lo inferimos de la detección
                # Por ahora usamos una dirección estimada
                # Se actualizará cuando la strategy llame con target real
                self._corridor_done = True
                return True

            if self._corridor_idx < len(self._corridor_wps):
                wp = self._corridor_wps[self._corridor_idx]
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

        # --- HOVER sobre la caja ---
        self._set_pos(self.detected_pos[0], self.detected_pos[1])
        return True

    def set_corridor(self, box_pos: list, target_pos: list):
        """Llamado por strategy cuando conoce box y target."""
        self._corridor_wps  = self._build_corridor_waypoints(
            box_pos, target_pos)
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
            return (f"Drone CORREDOR [{self._corridor_idx}/"
                    f"{len(self._corridor_wps)}] "
                    f"pos({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f})")
        return (f"Drone HOVER sobre caja "
                f"pos({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f})")
=======
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
>>>>>>> 5f2f97eed1f565038878880bd489cd56631980e4
