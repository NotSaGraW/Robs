"""
scene.py
Scene functions: geometry, push vectors, success condition.
Pure functions with no internal state.
"""

import math

PAYLOAD_HALF      = 0.25   # payload 0.5m x 0.5m x 0.5m
PAYLOAD_HEIGHT    = 0.5

# Success threshold: payload touches rally point
SUCCESS_THRESHOLD = PAYLOAD_HALF + 0.08  # 33cm


def get_position(sim, handle) -> list:
    return sim.getObjectPosition(handle, -1)


def dist2d(a: list, b: list) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def dist3d(a: list, b: list) -> float:
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)


def push_vector(payload_pos: list, rally_pos: list) -> tuple:
    """
    Unit vector from payload toward rally point (push direction).
    Returns (ux, uy, perp_x, perp_y).
    """
    dx = rally_pos[0] - payload_pos[0]
    dy = rally_pos[1] - payload_pos[1]
    norm = math.sqrt(dx * dx + dy * dy) + 1e-9
    ux, uy = dx / norm, dy / norm
    px, py = -uy, ux  # perpendicular 90° counter-clockwise
    return ux, uy, px, py


def pusher_position(payload_pos: list, ux: float, uy: float,
                    side_offset: float, back_distance: float) -> list:
    """
    Push position for a given robot.
    side_offset:   lateral displacement (+/-) from push vector
    back_distance: distance behind the payload

    With payload 0.5m and robot 0.52m long:
      back_distance = PAYLOAD_HALF(0.25) + margin(0.05) + robot_half(0.26) = 0.56m
    """
    ux_perp = -uy
    uy_perp =  ux
    return [
        payload_pos[0] - ux * back_distance + ux_perp * side_offset,
        payload_pos[1] - uy * back_distance + uy_perp * side_offset,
        0.0
    ]


def lateral_deviation(payload_pos: list, rally_pos: list,
                       ideal_start: list) -> float:
    """
    Lateral deviation of the payload from the ideal push trajectory.
    Positive = deviated left of the push vector.
    """
    ux, uy, px, py = push_vector(ideal_start, rally_pos)
    dx = payload_pos[0] - ideal_start[0]
    dy = payload_pos[1] - ideal_start[1]
    return dx * px + dy * py


def payload_reached_rally_point(payload_pos: list, rally_pos: list) -> bool:
    return dist2d(payload_pos, rally_pos) < SUCCESS_THRESHOLD