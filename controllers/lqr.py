import numpy as np
from scipy.linalg import solve_continuous_are


class LQR:
    def __init__(self, q_pos=10.0, q_vel=1.0, r=1.0, vmax=0.8):
        A = np.block([[np.zeros((2, 2)), np.eye(2)], [np.zeros((2, 4))]])
        B = np.vstack([np.zeros((2, 2)), np.eye(2)])
        Q = np.diag([q_pos, q_pos, q_vel, q_vel])
        R = r * np.eye(2)
        self.K = np.linalg.solve(R, B.T @ solve_continuous_are(A, B, Q, R))
        self.vmax = vmax

    def accel(self, pos, vel, target):
        x = np.r_[pos - target, vel]
        return -self.K @ x
