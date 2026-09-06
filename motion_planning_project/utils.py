"""Fonctions utilitaires : conversions repère monde <-> repère grille."""

from typing import Tuple


def world_to_grid(x: float, y: float, origin_x: float, origin_y: float,
                   resolution: float) -> Tuple[int, int]:
    """Convertit une coordonnée monde (m) en indices de grille (cellules)."""
    gx = int((x - origin_x) / resolution)
    gy = int((y - origin_y) / resolution)
    return gx, gy


def grid_to_world(gx: int, gy: int, origin_x: float, origin_y: float,
                   resolution: float) -> Tuple[float, float]:
    """Convertit des indices de grille en coordonnée monde (centre de cellule)."""
    x = origin_x + (gx + 0.5) * resolution
    y = origin_y + (gy + 0.5) * resolution
    return x, y
