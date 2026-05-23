"""
strategy.py
Máquina de estados para el sistema multi-robot.

Agentes:
  - Drone  : patrulla y detecta la caja
  - RobotA : Pioneer P3DX, rol pusher izquierdo
  - RobotB : Pioneer P3DX, rol pusher derecho

Estados:
  SEARCHING    → drone patrulla, robots buscan en sus sectores
  CONVERGING   → caja detectada, robots se posicionan para empujar
  PUSHING      → ambos robots empujan juntos hacia el target
  CORRECTING   → corrección de desviación lateral
  SUCCESS      → objetivo alcanzado
"""

from enum import Enum, auto
import math
import scene


class Phase(Enum):
    SEARCHING  = auto()
    CONVERGING = auto()
    PUSHING    = auto()
    CORRECTING = auto()
    SUCCESS    = auto()


# Sectores de búsqueda para cada robot mientras el drone patrulla
# RobotA cubre el cuadrante inferior, RobotB el superior
SEARCH_WAYPOINTS_A = [
    [-1.5, -1.5], [0.0, -1.5], [1.5, -1.5],
    [1.5,  -0.5], [0.0, -0.5], [-1.5, -0.5],
]
SEARCH_WAYPOINTS_B = [
    [-1.5,  1.5], [0.0,  1.5], [1.5,  1.5],
    [1.5,   0.5], [0.0,  0.5], [-1.5,  0.5],
]


class Strategy:

    # Parámetros de posicionamiento
    # Caja 0.5m → BOX_HALF=0.25, robot largo=0.52 → back=0.25+0.05+0.26=0.56
    BACK_DISTANCE   = 0.58      # metros detrás de la caja
    SIDE_OFFSET     = 0.20      # metros lateral (cada robot ocupa un lado)

    # Umbrales
    POSITION_OK     = 0.28      # metros — robot en posición de empuje
    DEVIATION_ALERT = 0.15      # metros — desviación que activa corrección
    DEVIATION_OK    = 0.07      # metros — desviación aceptable

    # Velocidades
    SEARCH_SPEED    = 2.5       # rad/s durante búsqueda
    APPROACH_SPEED  = 2.0       # rad/s durante convergencia
    PUSH_SPEED      = 0.7       # rad/s durante empuje (lento y sincronizado)
    CORRECT_SPEED   = 0.5       # rad/s durante corrección

    def __init__(self, robot_a, robot_b, drone, box_handle, target_handle, sim):
        self.robot_a = robot_a
        self.robot_b = robot_b
        self.drone = drone
        self.sim = sim
        self.box_handle = box_handle
        self.target_handle = target_handle

        self.phase = Phase.SEARCHING
        self.box_known_pos = None       # None hasta que el drone la detecte
        self.box_start = None           # posición inicial para calcular desviación

        # Índices de waypoints de búsqueda
        self._wp_a = 0
        self._wp_b = 0

        # Vectores de empuje (se actualizan cuando se conoce la caja)
        self.ux = self.uy = self.px = self.py = 0.0

    # ------------------------------------------------------------------
    # Ciclo principal
    # ------------------------------------------------------------------

    def step(self) -> Phase:
        # La posición real de la caja solo se usa en el drone para detección
        # Los robots NO tienen acceso directo a esta llamada
        box_real_pos = scene.get_position(self.sim, self.box_handle)
        target_pos   = scene.get_position(self.sim, self.target_handle)

        # Ciclo del drone — detecta geométricamente
        drone_detected = self.drone.step(box_real_pos)

        if drone_detected and self.box_known_pos is None:
            # Primera detección: el drone comunica la posición a los robots
            self.box_known_pos = self.drone.detected_pos[:]
            self.box_start = self.box_known_pos[:]
            self.phase = Phase.CONVERGING

        # Comprobación de éxito (desde cualquier fase activa)
        if self.box_known_pos is not None:
            # Actualizar posición conocida de la caja una vez detectada
            # (los robots la ven directamente al estar en contacto — distancia ~0)
            self.box_known_pos = box_real_pos[:]
            self._update_vectors(target_pos)

            if scene.box_reached_target(box_real_pos, target_pos):
                self.robot_a.stop()
                self.robot_b.stop()
                self.phase = Phase.SUCCESS
                return self.phase

        # Ejecutar fase
        if self.phase == Phase.SEARCHING:
            self._phase_searching()

        elif self.phase == Phase.CONVERGING:
            self._phase_converging(target_pos)

        elif self.phase == Phase.PUSHING:
            self._phase_pushing(target_pos)

        elif self.phase == Phase.CORRECTING:
            self._phase_correcting(target_pos)

        return self.phase

    # ------------------------------------------------------------------
    # Vectores
    # ------------------------------------------------------------------

    def _update_vectors(self, target_pos):
        self.ux, self.uy, self.px, self.py = scene.push_vector(
            self.box_known_pos, target_pos
        )

    # ------------------------------------------------------------------
    # FASE 0: Búsqueda
    # ------------------------------------------------------------------

    def _phase_searching(self):
        """Robots cubren sus sectores mientras el drone patrulla."""
        self._search_move(self.robot_a, SEARCH_WAYPOINTS_A, '_wp_a')
        self._search_move(self.robot_b, SEARCH_WAYPOINTS_B, '_wp_b')

    def _search_move(self, robot, waypoints, wp_attr):
        idx = getattr(self, wp_attr)
        goal = waypoints[idx] + [0.0]
        if robot.is_at(goal, threshold=0.3):
            new_idx = (idx + 1) % len(waypoints)
            setattr(self, wp_attr, new_idx)
            goal = waypoints[new_idx] + [0.0]
        robot.drive_to(goal, self.SEARCH_SPEED)

    # ------------------------------------------------------------------
    # FASE 1: Convergencia
    # ------------------------------------------------------------------

    def _phase_converging(self, target_pos):
        """Robots se posicionan detrás de la caja, lado a lado."""
        pos_a = scene.pusher_position(
            self.box_known_pos, self.ux, self.uy,
            side_offset=+self.SIDE_OFFSET,
            back_distance=self.BACK_DISTANCE
        )
        pos_b = scene.pusher_position(
            self.box_known_pos, self.ux, self.uy,
            side_offset=-self.SIDE_OFFSET,
            back_distance=self.BACK_DISTANCE
        )

        a_ready = self.robot_a.is_at(pos_a, self.POSITION_OK)
        b_ready = self.robot_b.is_at(pos_b, self.POSITION_OK)

        if not a_ready:
            self.robot_a.drive_to(pos_a, self.APPROACH_SPEED)
        else:
            self.robot_a.stop()

        if not b_ready:
            self.robot_b.drive_to(pos_b, self.APPROACH_SPEED)
        else:
            self.robot_b.stop()

        if a_ready and b_ready:
            self.phase = Phase.PUSHING

    # ------------------------------------------------------------------
    # FASE 2: Empuje conjunto
    # ------------------------------------------------------------------

    def _phase_pushing(self, target_pos):
        """
        Ambos robots apuntan al target directamente.
        La caja está entre ellos y el target — la física hace el resto.
        Velocidades idénticas para mantener alineación simétrica.
        """
        self.robot_a.drive_to(target_pos, self.PUSH_SPEED)
        self.robot_b.drive_to(target_pos, self.PUSH_SPEED)

        deviation = scene.lateral_deviation(
            self.box_known_pos, target_pos, self.box_start
        )
        if abs(deviation) > self.DEVIATION_ALERT:
            self.phase = Phase.CORRECTING

    # ------------------------------------------------------------------
    # FASE 3: Corrección
    # ------------------------------------------------------------------

    def _phase_correcting(self, target_pos):
        """
        Si la caja se desvía, el robot del lado opuesto a la desviación
        frena para dejar que el otro corrija empujando más.
        """
        deviation = scene.lateral_deviation(
            self.box_known_pos, target_pos, self.box_start
        )

        # El robot en el lado de la desviación frena
        # El del lado opuesto mantiene velocidad para corregir
        if deviation > 0:
            # Desviación hacia la izquierda (px positivo)
            # Robot A está en +side_offset (izquierda) → frena
            # Robot B mantiene velocidad
            self.robot_a.drive_to(target_pos, self.CORRECT_SPEED * 0.3)
            self.robot_b.drive_to(target_pos, self.CORRECT_SPEED)
        else:
            # Desviación hacia la derecha
            self.robot_a.drive_to(target_pos, self.CORRECT_SPEED)
            self.robot_b.drive_to(target_pos, self.CORRECT_SPEED * 0.3)

        if abs(deviation) < self.DEVIATION_OK:
            self.phase = Phase.PUSHING

    # ------------------------------------------------------------------
    # Log
    # ------------------------------------------------------------------

    def status(self, target_pos) -> str:
        if self.box_known_pos:
            d = scene.dist2d(self.box_known_pos, target_pos)
            dev = scene.lateral_deviation(
                self.box_known_pos, target_pos,
                self.box_start or self.box_known_pos
            )
            box_str = f"caja→target: {d:.3f}m  desv: {dev:+.3f}m"
        else:
            box_str = "caja: NO DETECTADA"

        pos_a = self.robot_a.get_position()
        pos_b = self.robot_b.get_position()
        return (
            f"[{self.phase.name:11s}] {box_str}  "
            f"A:({pos_a[0]:.2f},{pos_a[1]:.2f})  "
            f"B:({pos_b[0]:.2f},{pos_b[1]:.2f})"
        )