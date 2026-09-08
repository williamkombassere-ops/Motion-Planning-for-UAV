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
    grid[1][1] = 100
    planner = AStarPlanner(grid)
    path = planner.plan((0, 0), (1, 1))
    assert path is None


def test_path_avoids_wall():
    grid = [[0] * 5 for _ in range(5)]
    for y in range(5):
        if y != 2:
            grid[y][2] = 100
    planner = AStarPlanner(grid)
    path = planner.plan((0, 0), (4, 0))
    assert path is not None
    assert (2, 2) in path


def test_unknown_cells_treated_as_occupied_by_default():
    grid = [[-1] * 3 for _ in range(3)]  # tout inconnu (zone jamais vue par le multiranger)
    planner = AStarPlanner(grid)  # treat_unknown_as_occupied=True par défaut
    path = planner.plan((0, 0), (2, 2))
    assert path is None


def test_unknown_cells_can_be_treated_as_free():
    grid = [[-1] * 3 for _ in range(3)]
    planner = AStarPlanner(grid, treat_unknown_as_occupied=False)
    path = planner.plan((0, 0), (2, 2))
    assert path is not None
