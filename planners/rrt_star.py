import numpy as np


def rrt_star(grid, start, goal, res, n_iter=1000, step=0.4, radius=0.8, goal_bias=0.08, seed=3):
    rng = np.random.default_rng(seed)
    H, W = grid.shape

    def free(p):
        i, j = int(p[1] / res), int(p[0] / res)
        return 0 <= i < H and 0 <= j < W and not grid[i, j]

    def seg_free(a, b):
        n = max(int(np.hypot(*(b - a)) / (res * 0.5)), 1)
        return all(free(a + (b - a) * t) for t in np.linspace(0, 1, n + 1))

    start, goal = np.array(start, float), np.array(goal, float)
    nodes, parent, cost = [start], [-1], [0.0]
    best, best_cost, snaps = None, 1e9, []
    for it in range(n_iter):
        q = goal if rng.random() < goal_bias else rng.random(2) * [W * res, H * res]
        d = np.array([np.hypot(*(n - q)) for n in nodes])
        i = int(d.argmin())
        v = q - nodes[i]
        new = nodes[i] + v / (np.hypot(*v) + 1e-9) * min(step, np.hypot(*v))
        if not free(new) or not seg_free(nodes[i], new):
            continue
        dn = np.array([np.hypot(*(n - new)) for n in nodes])
        near = [k for k in np.where(dn < radius)[0] if seg_free(nodes[k], new)]
        bk = min(near, key=lambda k: cost[k] + dn[k], default=i)
        nodes.append(new)
        parent.append(bk)
        cost.append(cost[bk] + dn[bk])
        n_id = len(nodes) - 1
        for k in near:
            if cost[n_id] + dn[k] < cost[k]:
                parent[k], cost[k] = n_id, cost[n_id] + dn[k]
        if np.hypot(*(new - goal)) < step and seg_free(new, goal):
            c = cost[n_id] + np.hypot(*(new - goal))
            if c < best_cost:
                best_cost, best = c, n_id
        if it % 50 == 0:
            snaps.append((list(parent), [n.copy() for n in nodes], best))
    path, k = [goal], best
    while k != -1 and k is not None:
        path.append(nodes[k])
        k = parent[k]
    return np.array(path[::-1]), snaps
