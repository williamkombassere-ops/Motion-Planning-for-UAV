import numpy as np


def pure_pursuit(pos, path, lookahead=0.4):
    """Point à viser : premier point du chemin à `lookahead` m ou plus du drone."""
    pos = np.asarray(pos)
    i = int(np.argmin(np.hypot(*(path - pos).T)))
    for j in range(i, len(path)):
        if np.hypot(*(path[j] - pos)) >= lookahead:
            return path[j]
    return path[-1]
