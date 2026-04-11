"""
Smoothing and noise filtering for hand tracking.
- Exponential Moving Average (EMA)
- Dead-zone to ignore micro-movements
- Velocity clamping for safety
"""

import numpy as np


class SignalSmoother:
    """Smooths position (x, y) and rotation signals."""

    def __init__(self, alpha=0.3, dead_zone=0.005, max_vel=0.05, max_rot_vel=0.15):
        self.alpha = alpha
        self.dead_zone = dead_zone
        self.max_vel = max_vel
        self.max_rot_vel = max_rot_vel
        self._px = 0.5
        self._py = 0.5
        self._pr = 0.0
        self._init = False

    def update(self, x, y, rot):
        """Returns (smoothed_x, smoothed_y, smoothed_rotation)."""
        if not self._init:
            self._px, self._py, self._pr = x, y, rot
            self._init = True
            return x, y, rot

        dx = x - self._px
        dy = y - self._py
        dr = self._wrap(rot - self._pr)

        if abs(dx) < self.dead_zone: dx = 0.0
        if abs(dy) < self.dead_zone: dy = 0.0
        if abs(dr) < self.dead_zone * 5: dr = 0.0

        dx = np.clip(dx, -self.max_vel, self.max_vel)
        dy = np.clip(dy, -self.max_vel, self.max_vel)
        dr = np.clip(dr, -self.max_rot_vel, self.max_rot_vel)

        self._px += self.alpha * dx
        self._py += self.alpha * dy
        self._pr += self.alpha * dr

        return self._px, self._py, self._pr

    def reset(self):
        self._init = False

    @staticmethod
    def _wrap(a):
        while a > np.pi: a -= 2 * np.pi
        while a < -np.pi: a += 2 * np.pi
        return a
