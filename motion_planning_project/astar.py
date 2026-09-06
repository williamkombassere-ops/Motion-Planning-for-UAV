"""
Implémentation d'un planificateur A* sur grille d'occupation (OccupancyGrid).

Cette classe est volontairement indépendante de ROS afin de pouvoir être
testée unitairement sans lancer de node. Le node ROS2 (planner_node.py)
se charge de la conversion OccupancyGrid <-> grille numpy et de la
publication du chemin résultant.
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

    grid: liste de listes (ou array 2D) où 0 = libre, >0 = occupé.
    """

    # Déplacements possibles (8-connexité) avec leur coût
    _MOVES = [
        (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
        (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
        (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2)),
    ]

    def __init__(self, grid, occupied_threshold: int = 50):
        self.grid = grid
        self.height = len(grid)
        self.width = len(grid[0]) if self.height > 0 else 0
        self.occupied_threshold = occupied_threshold

    def _is_free(self, cell: Cell) -> bool:
        x, y = cell
        if not (0 <= x < self.width and 0 <= y < self.height):
            return False
        return self.grid[y][x] < self.occupied_threshold

    @staticmethod
    def _heuristic(a: Cell, b: Cell) -> float:
        # Distance euclidienne (admissible pour un déplacement 8-connexe)
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def plan(self, start: Cell, goal: Cell) -> Optional[List[Cell]]:
        """Retourne le chemin [start, ..., goal] ou None si aucun chemin trouvé."""
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

        return None  # Aucun chemin trouvé

    @staticmethod
    def _reconstruct_path(came_from, current: Cell) -> List[Cell]:
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path
