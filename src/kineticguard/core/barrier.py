"""
Control Barrier Function (CBF) Definitions for Kinematic & Dynamic Safety Sets.
"""

from abc import ABC, abstractmethod
from typing import Tuple, List, Dict, Any
import math
from kineticguard.core.state import RobotState, SafetyLimits


class BarrierFunction(ABC):
    """Abstract Control Barrier Function defining h(x) >= 0 and Lie derivative constraints."""

    @abstractmethod
    def evaluate(self, state: RobotState) -> float:
        """Returns h(x). Value >= 0 indicates state is safe."""
        pass

    @abstractmethod
    def get_cbf_constraint(self, state: RobotState, gamma: float = 10.0) -> Tuple[List[float], float]:
        pass


class JointLimitBarrier:
    """Barrier ensuring joints stay strictly within [q_min, q_max] and velocities within [-qd_max, qd_max]."""

    def __init__(self, limits: SafetyLimits, gamma: float = 15.0, dt: float = 0.001, enforce_acceleration: bool = False):
        self.limits = limits
        self.gamma = gamma
        self.dt = dt
        self.enforce_acceleration = enforce_acceleration

    def generate_constraints(self, state: RobotState) -> Tuple[List[List[float]], List[float]]:
        n = self.limits.num_dof
        A: List[List[float]] = []
        b: List[float] = []

        for i in range(n):
            # 1. Upper joint position barrier: u_i <= min(qd_max, gamma * (q_max - q))
            row_upper = [0.0] * n
            row_upper[i] = 1.0
            margin_upper = max(0.0, self.limits.q_max[i] - state.q[i])
            A.append(row_upper)
            b.append(min(self.limits.qd_max[i], self.gamma * margin_upper))

            # 2. Lower joint position barrier: -u_i <= min(qd_max, gamma * (q - q_min))
            row_lower = [0.0] * n
            row_lower[i] = -1.0
            margin_lower = max(0.0, state.q[i] - self.limits.q_min[i])
            A.append(row_lower)
            b.append(min(self.limits.qd_max[i], self.gamma * margin_lower))

            # 3. Dynamic acceleration bounds (if enabled)
            if self.enforce_acceleration:
                row_acc_up = [0.0] * n
                row_acc_up[i] = 1.0
                A.append(row_acc_up)
                b.append(state.qd[i] + self.limits.qdd_max[i] * self.dt)

                row_acc_low = [0.0] * n
                row_acc_low[i] = -1.0
                A.append(row_acc_low)
                b.append(-state.qd[i] + self.limits.qdd_max[i] * self.dt)

        return A, b


class ObstacleBarrier(BarrierFunction):
    """Spherical geometric obstacle barrier in end-effector workspace."""

    def __init__(self, obstacle_center: List[float], obstacle_radius: float, safety_margin: float = 0.05):
        self.center = obstacle_center
        self.radius = obstacle_radius + safety_margin

    def evaluate(self, state: RobotState) -> float:
        dx = state.ee_pos[0] - self.center[0]
        dy = state.ee_pos[1] - self.center[1]
        dz = state.ee_pos[2] - self.center[2]
        dist_sq = dx * dx + dy * dy + dz * dz
        return dist_sq - (self.radius * self.radius)

    def get_cbf_constraint(self, state: RobotState, gamma: float = 10.0) -> Tuple[List[float], float]:
        dx = state.ee_pos[0] - self.center[0]
        dy = state.ee_pos[1] - self.center[1]
        dz = state.ee_pos[2] - self.center[2]
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        h_val = dist - self.radius

        normal = [dx / (dist + 1e-6), dy / (dist + 1e-6), dz / (dist + 1e-6)]
        row = [-normal[0], -normal[1], -normal[2]] + [0.0] * (state.num_dof - 3)
        return row, gamma * max(0.0, h_val)
