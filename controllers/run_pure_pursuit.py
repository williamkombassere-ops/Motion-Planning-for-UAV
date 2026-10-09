import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
from common.drone import RES, START, GOAL, build_grid, draw_world, DroneArtist
from planners.astar import astar
from controllers.pure_pursuit import pure_pursuit

grid = build_grid()
coarse, _ = astar(grid, START, GOAL, RES)
d = np.r_[0, np.cumsum(np.hypot(*np.diff(coarse, axis=0).T))]
s = np.arange(0, d[-1], 0.03)
path = np.c_[np.interp(s, d, coarse[:, 0]), np.interp(s, d, coarse[:, 1])]

dt, tau, vmax = 0.05, 0.25, 0.8
pos, vel = np.array(START, float), np.zeros(2)
hist, targets = [pos.copy()], []
for _ in range(2000):
    tgt = pure_pursuit(pos, path)
    err = tgt - pos
    v_cmd = err / (np.hypot(*err) + 1e-9) * vmax
    vel += (v_cmd - vel) * dt / tau
    pos = pos + vel * dt
    hist.append(pos.copy())
    targets.append(tgt)
    if np.hypot(*(pos - GOAL)) < 0.1:
        break
hist = np.array(hist)

fig, ax = plt.subplots(figsize=(6, 4.2))
draw_world(ax, "Pure Pursuit")
ax.plot(path[:, 0], path[:, 1], "--", c="gray", lw=1.5)
trail, = ax.plot([], [], c="purple", lw=2)
aim, = ax.plot([], [], "kx", ms=8)
drone = DroneArtist(ax)
drone.update(*START)
step = 2


def update(f):
    i = min(f * step, len(hist) - 1)
    trail.set_data(hist[:i + 1, 0], hist[:i + 1, 1])
    drone.update(*hist[i])
    aim.set_data([targets[min(i, len(targets) - 1)][0]], [targets[min(i, len(targets) - 1)][1]])


FuncAnimation(fig, update, frames=len(hist) // step + 10).save(
    "media/pure_pursuit.gif", writer=PillowWriter(fps=20))
print("OK media/pure_pursuit.gif")
