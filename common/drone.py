import numpy as np
from matplotlib.patches import Circle, Rectangle

RES = 0.1                      # taille d'une cellule (m)
WIDTH, HEIGHT = 6.0, 4.0       # pièce (m)
WALLS = [(0, 0, 6, 0.1), (0, 3.9, 6, 0.1), (0, 0, 0.1, 4), (5.9, 0, 0.1, 4),
         (2.0, 0.1, 0.1, 2.0), (3.8, 1.9, 0.1, 2.0)]   # x, y, largeur, hauteur
START, GOAL = (0.5, 0.5), (5.3, 3.3)
ARM = 0.046                    # distance centre-moteur Crazyflie (m)
DRAW_SCALE = 3                 # drone agrandi x3 pour être visible


def build_grid(inflate=0.1):
    ny, nx = int(HEIGHT / RES), int(WIDTH / RES)
    grid = np.zeros((ny, nx), dtype=bool)
    for x, y, w, h in WALLS:
        grid[int(y / RES):int(np.ceil((y + h) / RES)),
             int(x / RES):int(np.ceil((x + w) / RES))] = True
    k = int(np.ceil(inflate / RES))
    out = grid.copy()
    for dy in range(-k, k + 1):
        for dx in range(-k, k + 1):
            out |= np.roll(np.roll(grid, dy, 0), dx, 1)
    return out


def draw_world(ax, title=""):
    for x, y, w, h in WALLS:
        ax.add_patch(Rectangle((x, y), w, h, color="black"))
    ax.plot(*START, "go", ms=6)
    ax.plot(*GOAL, "r*", ms=10)
    ax.set_xlim(-0.2, WIDTH + 0.2)
    ax.set_ylim(-0.2, HEIGHT + 0.2)
    ax.set_aspect("equal")
    ax.set_title(title)


class DroneArtist:
    def __init__(self, ax):
        self.arms = [ax.plot([], [], "k", lw=2)[0] for _ in range(2)]
        r = ARM * DRAW_SCALE * 0.55
        self.rotors = [Circle((0, 0), r, fill=False, ec="k", lw=1.5) for _ in range(4)]
        for c in self.rotors:
            ax.add_patch(c)

    def update(self, x, y, yaw=0.0):
        L = ARM * DRAW_SCALE
        p = [(x + L * np.cos(np.radians(a) + yaw), y + L * np.sin(np.radians(a) + yaw))
             for a in (45, 135, 225, 315)]
        for c, q in zip(self.rotors, p):
            c.center = q
        self.arms[0].set_data([p[0][0], p[2][0]], [p[0][1], p[2][1]])
        self.arms[1].set_data([p[1][0], p[3][0]], [p[1][1], p[3][1]])
