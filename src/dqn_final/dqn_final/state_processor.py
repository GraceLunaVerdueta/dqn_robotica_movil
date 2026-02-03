import numpy as np
from typing import Tuple


class StateProcessor:

    def __init__(self, n_lidar_bins=48):
        self.n_lidar_bins = n_lidar_bins
        self.max_lidar_range = 3.5

    def process_lidar(self, scan):
        arr = np.array(scan)
        arr[np.isinf(arr)] = self.max_lidar_range
        arr[np.isnan(arr)] = self.max_lidar_range
        arr = np.clip(arr, 0, self.max_lidar_range)

        pts = len(arr) // self.n_lidar_bins
        bins = []

        for i in range(self.n_lidar_bins):
            s = i * pts
            e = (i + 1) * pts if i < self.n_lidar_bins - 1 else len(arr)
            bins.append(np.min(arr[s:e]) if e > s else self.max_lidar_range)

        return np.array(bins) / self.max_lidar_range

    def compute_goal_info(self, pos, goal, yaw):
        dx = goal[0] - pos[0]
        dy = goal[1] - pos[1]

        dist = np.sqrt(dx * dx + dy * dy)
        ang = np.arctan2(dy, dx)
        rel = np.arctan2(np.sin(ang - yaw), np.cos(ang - yaw))

        return np.array([
            np.clip(dist / 25.0, 0, 1),
            rel / np.pi
        ])

    def get_state(self, scan, pos, goal, yaw):
        if scan is None:
            return np.zeros(self.n_lidar_bins + 2)
        return np.concatenate([
            self.process_lidar(scan),
            self.compute_goal_info(pos, goal, yaw)
        ])
