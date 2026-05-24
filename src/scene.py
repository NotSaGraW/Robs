"""
scene.py
<<<<<<< HEAD
Funciones de escena: geometría, vectores, condición de éxito.
=======
Funciones de escena: geometría, vectores de empuje, condición de éxito.
Sin estado propio — funciones puras sobre datos de simulación.
>>>>>>> 5f2f97eed1f565038878880bd489cd56631980e4
"""

import math

<<<<<<< HEAD
BOX_HALF         = 0.25    # caja 0.5m
SUCCESS_THRESHOLD= 0.33    # BOX_HALF + margen
=======
# Geometría de la caja actualizada (0.5m x 0.5m x 0.5m, 20kg)
BOX_HALF = 0.25         # metros (caja 50x50cm → radio = 25cm)
BOX_HEIGHT = 0.5        # metros

# Umbral de éxito: caja toca el target
SUCCESS_THRESHOLD = BOX_HALF + 0.08  # 33cm — cualquier punto de la caja sobre el target
>>>>>>> 5f2f97eed1f565038878880bd489cd56631980e4


def get_position(sim, handle) -> list:
    return sim.getObjectPosition(handle, -1)


def dist2d(a: list, b: list) -> float:
<<<<<<< HEAD
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)


def push_vector(box_pos: list, target_pos: list) -> tuple:
    dx   = target_pos[0] - box_pos[0]
    dy   = target_pos[1] - box_pos[1]
    norm = math.sqrt(dx*dx + dy*dy) + 1e-9
    ux, uy = dx/norm, dy/norm
    px, py = -uy, ux
=======
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def dist3d(a: list, b: list) -> float:
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)


def push_vector(box_pos: list, target_pos: list) -> tuple:
    """
    Vector unitario desde la caja hacia el target (dirección de empuje).
    Retorna (ux, uy, perp_x, perp_y).
    """
    dx = target_pos[0] - box_pos[0]
    dy = target_pos[1] - box_pos[1]
    norm = math.sqrt(dx * dx + dy * dy) + 1e-9
    ux, uy = dx / norm, dy / norm
    px, py = -uy, ux  # perpendicular 90° antihorario
>>>>>>> 5f2f97eed1f565038878880bd489cd56631980e4
    return ux, uy, px, py


def pusher_position(box_pos: list, ux: float, uy: float,
<<<<<<< HEAD
                    px: float, py: float,
                    side: float, back: float) -> list:
    """
    Posición de empuje para un robot.
    side: offset lateral (+/-) en metros
    back: distancia detrás de la caja en metros
    """
    return [
        box_pos[0] - ux * back + px * side,
        box_pos[1] - uy * back + py * side,
=======
                    side_offset: float, back_distance: float) -> list:
    """
    Posición de empuje para un robot dado.
    side_offset: desplazamiento lateral (+/-) respecto al vector de empuje
    back_distance: distancia detrás de la caja
    
    Con caja de 0.5m y robot de 0.52m de largo:
      back_distance = BOX_HALF(0.25) + margen(0.05) + semilargo_robot(0.26) = 0.56m
    """
    ux_perp, uy_perp = -uy, ux  # perpendicular
    return [
        box_pos[0] - ux * back_distance + ux_perp * side_offset,
        box_pos[1] - uy * back_distance + uy_perp * side_offset,
>>>>>>> 5f2f97eed1f565038878880bd489cd56631980e4
        0.0
    ]


<<<<<<< HEAD
def box_reached_target(box_pos: list, target_pos: list) -> bool:
    return dist2d(box_pos, target_pos) < SUCCESS_THRESHOLD


def lateral_deviation(box_pos, target_pos, box_start) -> float:
    ux, uy, px, py = push_vector(box_start, target_pos)
    dx = box_pos[0] - box_start[0]
    dy = box_pos[1] - box_start[1]
    return dx * px + dy * py
=======
def lateral_deviation(box_pos: list, target_pos: list,
                       ideal_start: list) -> float:
    """
    Desviación lateral de la caja respecto a la trayectoria ideal.
    Positivo = desviado hacia la izquierda del vector de empuje.
    """
    ux, uy, px, py = push_vector(ideal_start, target_pos)
    dx = box_pos[0] - ideal_start[0]
    dy = box_pos[1] - ideal_start[1]
    return dx * px + dy * py


def box_reached_target(box_pos: list, target_pos: list) -> bool:
    return dist2d(box_pos, target_pos) < SUCCESS_THRESHOLD
>>>>>>> 5f2f97eed1f565038878880bd489cd56631980e4
