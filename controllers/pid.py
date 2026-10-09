import numpy as np


class PID:
    def __init__(self, kp=2.0, ki=0.0, kd=0.4, vmax=0.8):
        self.kp, self.ki, self.kd, self.vmax = kp, ki, kd, vmax
        self.i, self.prev = np.zeros(2), None

    def step(self, pos, target, dt):
        e = target - pos
        self.i += e * dt
        d = np.zeros(2) if self.prev is None else (e - self.prev) / dt
        self.prev = e
        v = self.kp * e + self.ki * self.i + self.kd * d
        n = np.hypot(*v)
        return v if n <= self.vmax else v / n * self.vmax
