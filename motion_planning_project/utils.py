"""Conversions repère monde (m) <-> repère grille (cellules) pour une OccupancyGrid."""

from typing import Tuple


def world_to_grid(x: float, y: float, origin_x: float, origin_y: float,
                   resolution: float) -> Tuple[int, int]:
    gx = int((x - origin_x) / resolution)
    gy = int((y - origin_y) / resolution)
    return gx, gy


def grid_to_world(gx: int, gy: int, origin_x: float, origin_y: float,
                   resolution: float) -> Tuple[float, float]:
    x = origin_x + (gx + 0.5) * resolution
    y = origin_y + (gy + 0.5) * resolution
    return x, y
