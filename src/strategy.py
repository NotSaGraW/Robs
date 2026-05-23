"""
strategy.py
Máquina de estados con mapa de ocupación compartido.

Fases:
  EXPLORING   → todos los agentes mapean sin conocimiento previo
                robots barren sectores con sensores activos
                drone barre en cuadrícula sistemática
                cualquier agente que detecte box → broadcast

  CONVERGING  → box conocida, robots navegan hacia posiciones
                de empuje, drone mapea corredor box→target

  PUSHING     → empuje con ambos robots en contacto confirmado
                A detrás a 0.35m, B detrás a 0.45m
                si alguno pierde contacto → reajusta

  SUCCESS     → caja en target
"""

from enum import Enum, auto
import math
import scene
from .grid import OccupancyGrid, FREE


class Phase(Enum):
    EXPLORING  = auto()
    CONVERGING = auto()
    PUSHING    = auto()
    SUCCESS    = auto()


# Rutas de exploración — espirales desde esquinas opuestas
# Los robots no conocen el entorno, barren sistemáticamente
EXPLORE_WP_A = [
    [-1.8,-1.8],[0.0,-1.8],[1.8,-1.8],
    [1.8, 0.0],[1.8, 1.8],[0.0, 1.8],
    [-1.8,1.8],[-1.8,0.0],[-0.6,-0.6],
    [0.6,-0.6],[0.6, 0.6],[-0.6, 0.6],
    [0.0, 0.0],
]
EXPLORE_WP_B = [
    [1.8, 1.8],[0.0, 1.8],[-1.8,1.8],
    [-1.8,0.0],[-1.8,-1.8],[0.0,-1.8],
    [1.8,-1.8],[1.8, 0.0],[0.6, 0.6],
    [-0.6,0.6],[-0.6,-0.6],[0.6,-0.6],
    [0.0, 0.0],
]


class Strategy:

    # Empuje en línea — ambos detrás de la caja, escalonados en profundidad
    # A más cerca (contacto primario), B más lejos (refuerzo)
    BACK_A      = 0.35   # metros detrás de la caja — contacto directo
    BACK_B      = 0.50   # metros detrás de la caja — refuerzo
    SIDE_A      = +0.10  # offset lateral mínimo para no colisionar entre sí
    SIDE_B      = -0.10

    CONV_OK     = 0.30
    EXPLORE_SPD = 3.5
    APPROACH_SPD= 3.0
    PUSH_SPD    = 2.0

    def __init__(self, robot_a, robot_b, drone,
                 box_handle, target_handle, sim):
        self.robot_a  = robot_a
        self.robot_b  = robot_b
        self.drone    = drone
        self.sim      = sim
        self.box_h    = box_handle
        self.target_h = target_handle

        # Mapa compartido — ningún agente lo conoce de antemano
        self.map = OccupancyGrid(width=5.0, height=5.0, resolution=0.10)

        self.phase         = Phase.EXPLORING
        self.box_known_pos = None
        self.box_start     = None
        self._wp_a = 0
        self._wp_b = 0
        self.ux = self.uy = self.px = self.py = 0.0

        # CONVERGING: B usa staging para no cruzar la caja
        self._b_stage   = 0
        self._b_staging = None

    # ------------------------------------------------------------------

    def _update_vectors(self, target_pos):
        self.ux, self.uy, self.px, self.py = scene.push_vector(
            self.box_known_pos, target_pos)

    def _pos_a(self, back, side):
        return scene.pusher_position(
            self.box_known_pos, self.ux, self.uy,
            self.px, self.py, side, back)

    def _pos_b(self, back, side):
        return scene.pusher_position(
            self.box_known_pos, self.ux, self.uy,
            self.px, self.py, side, back)

    # ------------------------------------------------------------------
    # Ciclo principal
    # ------------------------------------------------------------------

    def step(self) -> Phase:
        box_real    = scene.get_position(self.sim, self.box_h)
        target_pos  = scene.get_position(self.sim, self.target_h)

        # Actualizar mapa con posición actual de los robots
        pa = self.robot_a.get_position()
        pb = self.robot_b.get_position()
        self.map.mark_robot_path(pa[0], pa[1])
        self.map.mark_robot_path(pb[0], pb[1])

        # Actualizar mapa con lecturas de sensores
        self.map.update_from_sensor(
            pa[0], pa[1], self.robot_a.get_yaw(),
            self.robot_a.read_sensors()
        )
        self.map.update_from_sensor(
            pb[0], pb[1], self.robot_b.get_yaw(),
            self.robot_b.read_sensors()
        )

        # Ciclo del drone — también mapea
        drone_detected = self.drone.step(box_real, self.map)

        # Primera detección de la caja
        if drone_detected and self.box_known_pos is None:
            self.box_known_pos = self.drone.detected_pos[:]
            self.box_start     = self.box_known_pos[:]
            self._b_stage      = 0
            self._b_staging    = None
            self.phase = Phase.CONVERGING

        # Detección por robot A o B via sensor de contacto.
        # El robot no sabía que la caja estaba ahí (exploración ciega),
        # pero al hacer contacto físico confirma su posición exacta
        # usando getObjectPosition — consistente con enfoque semi-realista
        # (GPS disponible, mapa del entorno no disponible a priori).
        if self.box_known_pos is None:
            contact_a, _ = self.robot_a.front_contact()
            contact_b, _ = self.robot_b.front_contact()
            if contact_a or contact_b:
                # Posición real confirmada al hacer contacto
                self.box_known_pos = box_real[:]
                self.box_start     = self.box_known_pos[:]
                self._b_stage      = 0
                self._b_staging    = None
                self.phase = Phase.CONVERGING
                detector = "A" if contact_a else "B"
                print(f"    [INFO] Caja detectada por Robot {detector} "
                      f"via sensor en {self.box_known_pos[:2]}")

        if self.box_known_pos is not None:
            self.box_known_pos = box_real[:]
            self._update_vectors(target_pos)

            if scene.box_reached_target(box_real, target_pos):
                self.robot_a.stop()
                self.robot_b.stop()
                self.phase = Phase.SUCCESS
                return self.phase

        if   self.phase == Phase.EXPLORING:  self._exploring()
        elif self.phase == Phase.CONVERGING: self._converging(target_pos)
        elif self.phase == Phase.PUSHING:    self._pushing(target_pos)

        return self.phase

    # ------------------------------------------------------------------
    # EXPLORING — mapeo activo sin conocimiento previo
    # ------------------------------------------------------------------

    def _exploring(self):
        self._explore_step(self.robot_a, EXPLORE_WP_A, '_wp_a')
        self._explore_step(self.robot_b, EXPLORE_WP_B, '_wp_b')

    def _explore_step(self, robot, wps, attr):
        idx  = getattr(self, attr)
        goal = wps[idx] + [0.0]
        if robot.is_at(goal, 0.30):
            idx = (idx + 1) % len(wps)
            setattr(self, attr, idx)
            goal = wps[idx] + [0.0]

        # Evasión Braitenberg si hay obstáculo frontal
        if robot.obstacle_ahead(0.45):
            vl, vr = robot.braitenberg()
            robot.set_velocity(vl, vr)
        else:
            robot.drive_to(goal, self.EXPLORE_SPD)

    # ------------------------------------------------------------------
    # CONVERGING — robots navegan a posiciones de empuje
    #
    # Problema clave: si un robot detectó la caja estando en el lado
    # frontal (entre caja y target), ir directo a su posición trasera
    # cruzaría la caja y la empujaría en dirección incorrecta.
    #
    # Solución: comprobar si el robot está en el lado frontal usando
    # el dot product (robot-caja)·(vector_empuje). Si es positivo,
    # el robot está delante — necesita rodear lateralmente primero.
    # ------------------------------------------------------------------

    def _on_front_side(self, robot_pos: list) -> bool:
        """
        True si el robot está entre la caja y el target (lado frontal).
        Calculado como dot product de (robot-caja) con vector de empuje.
        Si > 0.1, el robot está claramente en el lado frontal.
        """
        dx = robot_pos[0] - self.box_known_pos[0]
        dy = robot_pos[1] - self.box_known_pos[1]
        return (dx * self.ux + dy * self.uy) > 0.1

    def _lateral_bypass(self, robot_pos: list, side: float) -> list:
        """
        Punto de desvío lateral para que el robot evite pasar por
        delante de la caja. Se aleja lateralmente y luego por detrás.
        side: +1 o -1 según el lado del robot.
        """
        # Punto lateral: perpendicular a la dirección de empuje,
        # a la altura de la caja, suficientemente alejado
        return [
            self.box_known_pos[0] - self.ux * 0.1 + self.px * side * 0.8,
            self.box_known_pos[1] - self.uy * 0.1 + self.py * side * 0.8,
            0.0
        ]

    def _converging(self, target_pos):
        pos_a = self._pos_a(self.BACK_A + 0.20, self.SIDE_A)
        pos_b = self._pos_b(self.BACK_B + 0.20, self.SIDE_B)

        pa = self.robot_a.get_position()
        pb = self.robot_b.get_position()

        contact_a, _ = self.robot_a.front_contact()
        contact_b, _ = self.robot_b.front_contact()
        at_a = self.robot_a.is_at(pos_a, self.CONV_OK)
        at_b = self.robot_b.is_at(pos_b, self.CONV_OK)

        # --- Robot A ---
        if at_a or (contact_a and not self._on_front_side(pa)):
            self.robot_a.stop()
        elif self._on_front_side(pa):
            # A está en el lado frontal — rodear lateralmente
            bypass_a = self._lateral_bypass(pa, side=+1.0)
            self.robot_a.drive_to(bypass_a, self.APPROACH_SPD)
        elif self.robot_a.obstacle_ahead(0.40) and not contact_a:
            vl, vr = self.robot_a.braitenberg()
            self.robot_a.set_velocity(vl, vr)
        else:
            self.robot_a.drive_to(pos_a, self.APPROACH_SPD)

        # --- Robot B — staging por debajo ---
        if self._b_staging is None:
            self._b_staging = [-0.50, -1.5, 0.0]

        if at_b or (contact_b and not self._on_front_side(pb)):
            self.robot_b.stop()
        elif self._on_front_side(pb):
            bypass_b = self._lateral_bypass(pb, side=-1.0)
            self.robot_b.drive_to(bypass_b, self.APPROACH_SPD)
        elif self._b_stage == 0:
            if self.robot_b.is_at(self._b_staging, 0.28):
                self._b_stage = 1
            elif self.robot_b.obstacle_ahead(0.40):
                vl, vr = self.robot_b.braitenberg()
                self.robot_b.set_velocity(vl, vr)
            else:
                self.robot_b.drive_to(self._b_staging, self.APPROACH_SPD)
        else:
            if self.robot_b.obstacle_ahead(0.35):
                vl, vr = self.robot_b.braitenberg()
                self.robot_b.set_velocity(vl, vr)
            else:
                self.robot_b.drive_to(pos_b, self.APPROACH_SPD * 0.6)

        if (at_a or (contact_a and not self._on_front_side(pa))) and            (at_b or (contact_b and not self._on_front_side(pb))):
            self.phase = Phase.PUSHING

    # ------------------------------------------------------------------
    # PUSHING — empuje coordinado con contacto confirmado en ambos
    #
    # A detrás a BACK_A (contacto primario, más fuerza)
    # B detrás a BACK_B (refuerzo, mismo lado pero más lejos)
    # Ambos ligeramente offset lateral para no colisionar entre sí
    # ------------------------------------------------------------------

    def _pushing(self, target_pos):
        pos_a = self._pos_a(self.BACK_A, self.SIDE_A)
        pos_b = self._pos_b(self.BACK_B, self.SIDE_B)

        contact_a, dist_a = self.robot_a.front_contact()
        contact_b, dist_b = self.robot_b.front_contact()

        # Si A perdió contacto, avanza más agresivamente
        spd_a = self.PUSH_SPD if contact_a else self.PUSH_SPD * 1.5
        # Si B ya tiene contacto, mantiene velocidad moderada
        spd_b = self.PUSH_SPD * 0.8 if contact_b else self.PUSH_SPD

        self.robot_a.drive_to(pos_a, spd_a)
        self.robot_b.drive_to(pos_b, spd_b)

    # ------------------------------------------------------------------
    # Log
    # ------------------------------------------------------------------

    def status(self, target_pos) -> str:
        if self.box_known_pos:
            d   = scene.dist2d(self.box_known_pos, target_pos)
            dev = scene.lateral_deviation(
                self.box_known_pos, target_pos,
                self.box_start or self.box_known_pos)
            box_str = f"caja→target:{d:.3f}m desv:{dev:+.3f}m"
        else:
            box_str = "caja:DESCONOCIDA"

        pa = self.robot_a.get_position()
        pb = self.robot_b.get_position()
        ca, da = self.robot_a.front_contact()
        cb, db = self.robot_b.front_contact()
        return (
            f"[{self.phase.name:11s}] {box_str} "
            f"A:({pa[0]:.2f},{pa[1]:.2f})[{'C' if ca else '-'}:{da:.2f}] "
            f"B:({pb[0]:.2f},{pb[1]:.2f})[{'C' if cb else '-'}:{db:.2f}] "
            f"mapa:{self.map.coverage_percent():.0f}%"
        )