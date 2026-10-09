import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Circle, Rectangle
from common.drone import RES, START, GOAL, build_grid, draw_world, DroneArtist
from planners.astar import astar
from controllers.mpc import MPC, build_ref

CIRCLES = [(1.2, 1.6, 0.30), (2.9, 2.0, 0.30), (5.0, 1.0, 0.30)]   # x, y, rayon
RECTS = [(4.3, 2.3, 0.8, 0.4), (2.6, 0.5, 0.7, 0.5)]               # x, y, largeur, hauteur
MARGIN = 0.15

grid = build_grid()
yy, xx = (np.mgrid[0:grid.shape[0], 0:grid.shape[1]] + 0.5) * RES
for cx, cy, r in CIRCLES:
    grid |= np.hypot(xx - cx, yy - cy) < r + MARGIN
for x, y, w, h in RECTS:
    grid |= (xx > x - MARGIN) & (xx < x + w + MARGIN) & (yy > y - MARGIN) & (yy < y + h + MARGIN)

coarse, _ = astar(grid, START, GOAL, RES)
d = np.r_[0, np.cumsum(np.hypot(*np.diff(coarse, axis=0).T))]
s = np.arange(0, d[-1], 0.03)
path = np.c_[np.interp(s, d, coarse[:, 0]), np.interp(s, d, coarse[:, 1])]

dt, vmax = 0.05, 0.8
mpc = MPC()
pos, vel = np.array(START, float), np.zeros(2)
hist = [pos.copy()]
for _ in range(2000):
    vel = vel + mpc.accel(pos, vel, build_ref(pos, path, mpc.N)) * dt
    n = np.hypot(*vel)
    if n > vmax:
        vel = vel / n * vmax
    pos = pos + vel * dt
    hist.append(pos.copy())
    if np.hypot(*(pos - GOAL)) < 0.1:
        break
hist = np.array(hist)
print("pas simulés :", len(hist))

fig, ax = plt.subplots(figsize=(6, 4.2))
draw_world(ax, "MPC")
for cx, cy, r in CIRCLES:
    ax.add_patch(Circle((cx, cy), r, color="tab:orange"))
for x, y, w, h in RECTS:
    ax.add_patch(Rectangle((x, y), w, h, color="tab:orange"))
ax.plot(path[:, 0], path[:, 1], "--", c="gray", lw=1.5)
trail, = ax.plot([], [], c="purple", lw=2)
drone = DroneArtist(ax)
drone.update(*START)
step = 4


def update(f):
    i = min(f * step, len(hist) - 1)
    trail.set_data(hist[:i + 1, 0], hist[:i + 1, 1])
    drone.update(*hist[i])


FuncAnimation(fig, update, frames=len(hist) // step + 10).save(
    "media/mpc.gif", writer=PillowWriter(fps=20))
print("OK media/mpc.gif")
