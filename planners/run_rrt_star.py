import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.collections import LineCollection
from common.drone import RES, START, GOAL, build_grid, draw_world, DroneArtist
from planners.rrt_star import rrt_star

grid = build_grid()
path, snaps = rrt_star(grid, START, GOAL, RES)
d = np.r_[0, np.cumsum(np.hypot(*np.diff(path, axis=0).T))]
s = np.arange(0, d[-1], 0.06)
traj = np.c_[np.interp(s, d, path[:, 0]), np.interp(s, d, path[:, 1])]

fig, ax = plt.subplots(figsize=(6, 4.2))
draw_world(ax, "RRT*")
tree = LineCollection([], colors="tab:blue", alpha=0.7, lw=1.0)
ax.add_collection(tree)
line, = ax.plot([], [], "r-", lw=2)
drone = DroneArtist(ax)
drone.update(*START)
NS = len(snaps)


def update(f):
    if f < NS:
        par, nd, _ = snaps[f]
        tree.set_segments([[nd[i], nd[p]] for i, p in enumerate(par) if p >= 0])
    else:
        i = min(f - NS, len(traj) - 1)
        line.set_data(path[:, 0], path[:, 1])
        drone.update(*traj[i])


FuncAnimation(fig, update, frames=NS + len(traj) + 10).save(
    "media/rrt_star.gif", writer=PillowWriter(fps=20))
print("OK media/rrt_star.gif")
