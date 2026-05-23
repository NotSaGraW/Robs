"""
scene.py
Funciones de escena: geometría, vectores, condición de éxito.
"""

import math

BOX_HALF         = 0.25    # caja 0.5m
SUCCESS_THRESHOLD= 0.33    # BOX_HALF + margen


def get_position(sim, handle) -> list:
    return sim.getObjectPosition(handle, -1)


def dist2d(a: list, b: list) -> float:
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)


def push_vector(box_pos: list, target_pos: list) -> tuple:
    dx   = target_pos[0] - box_pos[0]
    dy   = target_pos[1] - box_pos[1]
    norm = math.sqrt(dx*dx + dy*dy) + 1e-9
    ux, uy = dx/norm, dy/norm
    px, py = -uy, ux
    return ux, uy, px, py


def pusher_position(box_pos: list, ux: float, uy: float,
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
        0.0
    ]


def box_reached_target(box_pos: list, target_pos: list) -> bool:
    return dist2d(box_pos, target_pos) < SUCCESS_THRESHOLD


def lateral_deviation(box_pos, target_pos, box_start) -> float:
    ux, uy, px, py = push_vector(box_start, target_pos)
    dx = box_pos[0] - box_start[0]
    dy = box_pos[1] - box_start[1]
    return dx * px + dy * py