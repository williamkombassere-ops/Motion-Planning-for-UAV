import numpy as np


class MPC:
    def __init__(self, dt=0.05, N=20, q_pos=20.0, q_vel=2.0, r=0.5, amax=3.0):
        self.N, self.amax = N, amax
        A = np.eye(4)
        A[0, 2] = A[1, 3] = dt
        B = np.zeros((4, 2))
        B[0, 0] = B[1, 1] = dt * dt / 2
        B[2, 0] = B[3, 1] = dt
        Sx, Su = np.zeros((N * 4, 4)), np.zeros((N * 4, N * 2))
        for k in range(N):
            Sx[k * 4:(k + 1) * 4] = np.linalg.matrix_power(A, k + 1)
            for j in range(k + 1):
                Su[k * 4:(k + 1) * 4, j * 2:(j + 1) * 2] = np.linalg.matrix_power(A, k - j) @ B
        Q = np.kron(np.eye(N), np.diag([q_pos, q_pos, q_vel, q_vel]))
        self.Sx = Sx
        self.G = np.linalg.solve(Su.T @ Q @ Su + r * np.eye(N * 2), Su.T @ Q)

    def accel(self, pos, vel, ref):
        U = self.G @ (ref.ravel() - self.Sx @ np.r_[pos, vel])
        a = U[:2]
        n = np.hypot(*a)
        return a if n <= self.amax else a / n * self.amax


def build_ref(pos, path, N, vref=0.6, spacing=0.03):
    i = int(np.argmin(np.hypot(*(path - pos).T)))
    idx = np.minimum(i + 1 + np.arange(N), len(path) - 1)
    nxt = np.minimum(idx + 1, len(path) - 1)
    tan = path[nxt] - path[idx]
    tan = tan / (np.hypot(*tan.T)[:, None] + 1e-9) * vref
    return np.c_[path[idx], tan]
