import heapq
import numpy as np


def astar(grid, start, goal, res):
    s = (int(start[1] / res), int(start[0] / res))
    g = (int(goal[1] / res), int(goal[0] / res))
    moves = [(-1, 0, 1), (1, 0, 1), (0, -1, 1), (0, 1, 1),
             (-1, -1, 2**0.5), (-1, 1, 2**0.5), (1, -1, 2**0.5), (1, 1, 2**0.5)]
    h = lambda c: np.hypot(c[0] - g[0], c[1] - g[1])
    pq, cost, parent, explored, closed = [(h(s), s)], {s: 0.0}, {s: None}, [], set()
    while pq:
        _, cur = heapq.heappop(pq)
        if cur in closed:
            continue
        closed.add(cur)
        explored.append(cur)
        if cur == g:
            break
        for dy, dx, c in moves:
            n = (cur[0] + dy, cur[1] + dx)
            if not (0 <= n[0] < grid.shape[0] and 0 <= n[1] < grid.shape[1]) or grid[n]:
                continue
            nc = cost[cur] + c
            if nc < cost.get(n, 1e9):
                cost[n], parent[n] = nc, cur
                heapq.heappush(pq, (nc + h(n), n))
    path, c = [], g if g in parent else None
    while c is not None:
        path.append(c)
        c = parent[c]
    to_xy = lambda cells: np.array([((x + 0.5) * res, (y + 0.5) * res) for y, x in cells])
    return to_xy(path[::-1]), to_xy(explored)
