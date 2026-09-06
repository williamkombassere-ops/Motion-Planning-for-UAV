"""Tests unitaires purs Python pour AStarPlanner (pas besoin de ROS2 pour les lancer)."""

from motion_planning_project.astar import AStarPlanner


def test_path_found_on_empty_grid():
    grid = [[0] * 5 for _ in range(5)]
    planner = AStarPlanner(grid)
    path = planner.plan((0, 0), (4, 4))
    assert path is not None
    assert path[0] == (0, 0)
    assert path[-1] == (4, 4)


def test_no_path_when_goal_blocked():
    grid = [[0] * 3 for _ in range(3)]
    grid[1][1] = 100  # occupe le centre
    planner = AStarPlanner(grid)
    path = planner.plan((0, 0), (1, 1))
    assert path is None


def test_path_avoids_wall():
    # Mur vertical complet en colonne x=2, sauf une brèche en (2, 2)
    grid = [[0] * 5 for _ in range(5)]
    for y in range(5):
        if y != 2:
            grid[y][2] = 100

    planner = AStarPlanner(grid)
    path = planner.plan((0, 0), (4, 0))
    assert path is not None
    # Le chemin doit nécessairement passer par la brèche
    assert (2, 2) in path
