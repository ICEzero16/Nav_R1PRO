"""Spatial speed limits for approach and bends on a holonomic path."""
import math

import numpy as np


class PathSpeedProfile:
    def __init__(self, points):
        points = np.asarray(points, dtype=float)
        lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
        self.points = points[np.r_[True, lengths > 1e-6]]
        self.segments = np.diff(self.points, axis=0)
        self.lengths = np.linalg.norm(self.segments, axis=1)
        self.distance = np.r_[0., np.cumsum(self.lengths)]
        self.progress = 0.
        # Measure bends over 15 cm on each side to suppress grid stair steps.
        along = self.distance
        before = self.position(np.maximum(0., along - .15))
        after = self.position(np.minimum(along[-1], along + .15))
        incoming, outgoing = self.points - before, after - self.points
        angle = np.abs(np.arctan2(
            incoming[:, 0] * outgoing[:, 1] - incoming[:, 1] * outgoing[:, 0],
            np.sum(incoming * outgoing, axis=1)))
        self.bends = along[(angle > math.radians(25)) & (along > .12) & (along < along[-1] - .12)]

    def position(self, distance):
        return np.column_stack([np.interp(distance, self.distance, self.points[:, i]) for i in (0, 1)])

    def limit(self, x, y, maximum):
        if not len(self.segments):
            return min(maximum, .05)
        delta = np.array([x, y]) - self.points[:-1]
        fractions = np.clip(np.sum(delta * self.segments, axis=1) / self.lengths**2, 0., 1.)
        projections = self.points[:-1] + fractions[:, None] * self.segments
        nearest = np.argmin(np.sum((projections - [x, y])**2, axis=1))
        self.progress = max(self.progress, self.distance[nearest] + fractions[nearest] * self.lengths[nearest])
        remaining = self.distance[-1] - self.progress
        # Begin approach slowdown at about 40 cm. Keep enough speed for the
        # controller to settle inside the 5 cm goal window.
        limit = min(maximum, max(.05, .25 * remaining))
        bends = self.bends[self.bends >= self.progress - .12]
        if len(bends):
            distance_to_bend = max(0., bends[0] - self.progress - .12)
            limit = min(limit, .05 + .15 * distance_to_bend)
        return float(limit)
