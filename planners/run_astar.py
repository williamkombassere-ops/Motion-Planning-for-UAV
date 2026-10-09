import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
from common.drone import RES, START, GOAL, build_grid, draw_world, DroneArtist
from planners.astar import astar

grid = build_grid()
path, explored = astar(grid, START, GOAL, RES)
d = np.r_[0, np.cumsum(np.hypot(*np.diff(path, axis=0).T))]
s = np.arange(0, d[-1], 0.06)
traj = np.c_[np.interp(s, d, path[:, 0]), np.interp(s, d, path[:, 1])]

fig, ax = plt.subplots(figsize=(6, 4.2))
draw_world(ax, "A*")
cloud = ax.scatter([], [], s=6, c="tab:blue", alpha=0.35)
line, = ax.plot([], [], "r-", lw=2)
drone = DroneArtist(ax)
drone.update(*START)
NE = 40


def update(f):
    if f < NE:
        cloud.set_offsets(explored[:int((f + 1) / NE * len(explored))])
    else:
        i = min(f - NE, len(traj) - 1)
        line.set_data(path[:, 0], path[:, 1])
        drone.update(*traj[i])


FuncAnimation(fig, update, frames=NE + len(traj) + 10).save(
    "media/astar.gif", writer=PillowWriter(fps=20))
print("OK media/astar.gif")
