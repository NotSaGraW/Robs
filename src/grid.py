"""
grid.py
Mapa de ocupación 2D compartido entre todos los agentes.

El tablero es 5x5m centrado en el origen.
Resolución: 0.1m por celda → grid 50x50.

Sin dependencias externas (no requiere numpy).

Estados de celda:
    UNKNOWN  = 0  — celda no explorada aún
    FREE     = 1  — confirmado libre (robot pasó por aquí o sensor despejado)
    OCCUPIED = 2  — obstáculo detectado por sensor de proximidad
"""

import math


# Constantes de estado de celda
UNKNOWN  = 0
FREE     = 1
OCCUPIED = 2


class OccupancyGrid:
    """
    Mapa de ocupación 2D implementado como lista de listas.
    Compartido entre drone, robot A y robot B — todos lo actualizan,
    todos pueden consultarlo para navegación segura.
    """

    def __init__(self,
                 width: float      = 5.0,
                 height: float     = 5.0,
                 resolution: float = 0.10):
        self.resolution = resolution
        self.width      = width
        self.height     = height
        self.cols       = int(width  / resolution)
        self.rows       = int(height / resolution)

        # Grid inicialmente desconocido — lista de listas de enteros
        self.grid = [[UNKNOWN] * self.cols for _ in range(self.rows)]

        # Origen del mundo en la esquina inferior-izquierda del grid
        self.origin_x = -width  / 2.0
        self.origin_y = -height / 2.0

    # ------------------------------------------------------------------
    # Conversión coordenadas mundo ↔ celda
    # ------------------------------------------------------------------

    def world_to_cell(self, x: float, y: float) -> tuple:
        """Convierte coordenadas mundo (metros) a índices de celda (row, col)."""
        col = int((x - self.origin_x) / self.resolution)
        row = int((y - self.origin_y) / self.resolution)
        col = max(0, min(self.cols - 1, col))
        row = max(0, min(self.rows - 1, row))
        return row, col

    def cell_to_world(self, row: int, col: int) -> tuple:
        """Convierte índices de celda al centro de esa celda en coordenadas mundo."""
        x = self.origin_x + (col + 0.5) * self.resolution
        y = self.origin_y + (row + 0.5) * self.resolution
        return x, y

    def in_bounds(self, row: int, col: int) -> bool:
        return 0 <= row < self.rows and 0 <= col < self.cols

    # ------------------------------------------------------------------
    # Actualización del mapa
    # ------------------------------------------------------------------

    def mark_free(self, x: float, y: float):
        """Marca una celda como libre si era desconocida."""
        r, c = self.world_to_cell(x, y)
        if self.grid[r][c] == UNKNOWN:
            self.grid[r][c] = FREE

    def mark_occupied(self, x: float, y: float):
        """Marca una celda como ocupada (obstáculo detectado)."""
        r, c = self.world_to_cell(x, y)
        self.grid[r][c] = OCCUPIED

    def mark_robot_path(self, x: float, y: float, radius: float = 0.25):
        """
        Marca como FREE un radio alrededor de la posición del robot.
        El robot garantiza que esa zona es transitable porque acaba de pasar.
        """
        cells = int(radius / self.resolution) + 1
        rc, cc = self.world_to_cell(x, y)
        for dr in range(-cells, cells + 1):
            for dc in range(-cells, cells + 1):
                r, c = rc + dr, cc + dc
                if self.in_bounds(r, c):
                    wx, wy = self.cell_to_world(r, c)
                    if math.sqrt((wx-x)**2 + (wy-y)**2) <= radius:
                        if self.grid[r][c] == UNKNOWN:
                            self.grid[r][c] = FREE

    def update_from_sensor(self, robot_x: float, robot_y: float,
                           robot_yaw: float, readings: list):
        """
        Actualiza el mapa con lecturas de sensores ultrasónicos.
        Usa ray casting: marca FREE a lo largo del rayo, OCCUPIED en el hit.

        Ángulos de los 16 sensores del Pioneer P3DX (grados, relativo al robot):
          Frontales (0-7): de izquierda a derecha
          Traseros (8-15): ignorados para el mapa (pesos Braitenberg = 0)
        """
        sensor_angles_deg = [
            90, 50, 30, 10, -10, -30, -50, -90,   # 0-7 frontales
            -90, -130, -150, -170,                  # 8-11 traseros izq
            170, 150, 130, 90                       # 12-15 traseros der
        ]
        max_range = 0.5  # rango máximo del sensor ultrasónico Pioneer

        for i, (detected, dist) in enumerate(readings):
            if i >= len(sensor_angles_deg):
                break
            angle = robot_yaw + math.radians(sensor_angles_deg[i])

            if detected and dist < max_range:
                self._cast_ray(robot_x, robot_y, angle, dist,
                               mark_end=OCCUPIED)
            else:
                self._cast_ray(robot_x, robot_y, angle, max_range,
                               mark_end=FREE)

    def _cast_ray(self, ox: float, oy: float, angle: float,
                  length: float, mark_end: int):
        """
        Ray casting simplificado: avanza en pasos de resolución/2 a lo largo
        del rayo y marca celdas como FREE (intermedias) u OCCUPIED/FREE (final).
        """
        step  = self.resolution / 2.0
        steps = max(int(length / step), 1)
        dx    = math.cos(angle) * step
        dy    = math.sin(angle) * step

        x, y = ox, oy
        for i in range(steps):
            x += dx
            y += dy
            r, c = self.world_to_cell(x, y)
            if not self.in_bounds(r, c):
                break
            if i < steps - 1:
                if self.grid[r][c] == UNKNOWN:
                    self.grid[r][c] = FREE
            else:
                self.grid[r][c] = mark_end

    # ------------------------------------------------------------------
    # Consultas
    # ------------------------------------------------------------------

    def is_free(self, x: float, y: float) -> bool:
        r, c = self.world_to_cell(x, y)
        return self.grid[r][c] == FREE

    def is_path_clear(self, x1: float, y1: float,
                      x2: float, y2: float,
                      clearance: float = 0.30) -> bool:
        """
        Comprueba si el segmento (x1,y1)→(x2,y2) está libre de obstáculos
        con margen lateral de clearance metros.
        Útil para que el drone verifique el corredor box→target.
        """
        dx   = x2 - x1
        dy   = y2 - y1
        dist = math.sqrt(dx*dx + dy*dy)
        if dist < 1e-6:
            return True

        steps = max(int(dist / self.resolution), 1)
        nx    = -dy / dist   # normal al segmento
        ny    =  dx / dist

        for i in range(steps + 1):
            t  = i / steps
            px = x1 + t * dx
            py = y1 + t * dy
            for sign in [-1, 0, 1]:
                cx = px + sign * clearance * nx
                cy = py + sign * clearance * ny
                r, c = self.world_to_cell(cx, cy)
                if self.in_bounds(r, c) and self.grid[r][c] == OCCUPIED:
                    return False
        return True

    def coverage_percent(self) -> float:
        """Porcentaje de celdas exploradas (FREE + OCCUPIED) sobre el total."""
        explored = sum(
            1 for r in range(self.rows)
            for c in range(self.cols)
            if self.grid[r][c] != UNKNOWN
        )
        return 100.0 * explored / (self.rows * self.cols)

    def stats(self) -> str:
        """Resumen del estado del mapa para logs."""
        free = occ = unk = 0
        for r in range(self.rows):
            for c in range(self.cols):
                v = self.grid[r][c]
                if v == FREE:     free += 1
                elif v == OCCUPIED: occ += 1
                else:             unk  += 1
        total = self.rows * self.cols
        return (f"Grid {self.rows}x{self.cols} | "
                f"libre:{free}({100*free//total}%) "
                f"ocup:{occ}({100*occ//total}%) "
                f"desc:{unk}({100*unk//total}%)")

    def save_png(self, filepath: str):
        """
        Exporta el mapa a PNG.
        UNKNOWN=gris, FREE=blanco, OCCUPIED=negro.
        Requiere Pillow (pip install Pillow).
        Si no está disponible, guarda el mapa como CSV en su lugar.
        """
        try:
            from PIL import Image
            img = Image.new('RGB', (self.cols, self.rows), (128, 128, 128))
            pixels = img.load()
            colors = {
                UNKNOWN:  (128, 128, 128),  # gris
                FREE:     (255, 255, 255),  # blanco
                OCCUPIED: (0,   0,   0),    # negro
            }
            for r in range(self.rows):
                for c in range(self.cols):
                    # Flip vertical: row 0 es y negativa, queremos y positiva arriba
                    pixels[c, self.rows - 1 - r] = colors[self.grid[r][c]]
            img.save(filepath)
            return True
        except ImportError:
            # Fallback: guardar como CSV
            csv_path = filepath.replace('.png', '.csv')
            with open(csv_path, 'w') as f:
                for r in range(self.rows - 1, -1, -1):
                    f.write(','.join(str(self.grid[r][c])
                                     for c in range(self.cols)) + '\n')
            return False