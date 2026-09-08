"""
Planificateur A* sur grille d'occupation 2D (nav_msgs/OccupancyGrid).

Indépendant de ROS pour rester testable unitairement sans lancer de node.
Utilisé ici pour planifier le déplacement horizontal du Crazyflie à
altitude constante, sur la carte générée par le simple_mapper
(package crazyflie_ros2_multiranger).
"""

import heapq
import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

Cell = Tuple[int, int]


@dataclass(order=True)
class _PQItem:
    priority: float
    cell: Cell = field(compare=False)


class AStarPlanner:
    """Planificateur A* 8-connexe sur une grille 2D.

    grid: liste de listes où 0 = libre, >0 = occupé, -1 = inconnu (traité comme occupé
    par défaut, car on ne veut pas voler dans une zone jamais vue par le multiranger).
    """

    _MOVES = [
        (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
        (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
        (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2)),
    ]

    def __init__(self, grid, occupied_threshold: int = 50, treat_unknown_as_occupied: bool = True):
        self.grid = grid
        self.height = len(grid)
        self.width = len(grid[0]) if self.height > 0 else 0
        self.occupied_threshold = occupied_threshold
        self.treat_unknown_as_occupied = treat_unknown_as_occupied

    def _is_free(self, cell: Cell) -> bool:
        x, y = cell
        if not (0 <= x < self.width and 0 <= y < self.height):
            return False
        value = self.grid[y][x]
        if value < 0:  # inconnu dans une OccupancyGrid ROS
            return not self.treat_unknown_as_occupied
        return value < self.occupied_threshold

    @staticmethod
    def _heuristic(a: Cell, b: Cell) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def plan(self, start: Cell, goal: Cell) -> Optional[List[Cell]]:
        if not self._is_free(start) or not self._is_free(goal):
            return None

        open_heap: List[_PQItem] = []
        heapq.heappush(open_heap, _PQItem(0.0, start))

        came_from = {}
        g_score = {start: 0.0}
        visited = set()

        while open_heap:
            current = heapq.heappop(open_heap).cell

            if current == goal:
                return self._reconstruct_path(came_from, current)

            if current in visited:
                continue
            visited.add(current)

            for dx, dy, cost in self._MOVES:
                neighbor = (current[0] + dx, current[1] + dy)
                if not self._is_free(neighbor) or neighbor in visited:
                    continue

                tentative_g = g_score[current] + cost
                if tentative_g < g_score.get(neighbor, math.inf):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self._heuristic(neighbor, goal)
                    heapq.heappush(open_heap, _PQItem(f_score, neighbor))

        return None

    @staticmethod
    def _reconstruct_path(came_from, current: Cell) -> List[Cell]:
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path
