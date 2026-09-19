"""
Robot State and Safety Limits Representation for KineticGuard.
"""

from dataclasses import dataclass, field, asdict
from typing import List, Dict, Any, Optional
import json


@dataclass
class SafetyLimits:
    """Rigid kinematic, dynamic, and workspace limits for an articulated robot."""
    num_dof: int
    q_min: List[float]
    q_max: List[float]
    qd_max: List[float]
    tau_max: List[float]
    qdd_max: List[float]
    workspace_min: List[float] = field(default_factory=lambda: [-1.5, -1.5, 0.0])
    workspace_max: List[float] = field(default_factory=lambda: [1.5, 1.5, 2.0])
    max_jerk: float = 100.0  # rad/s^3

    def validate(self) -> None:
        if not (len(self.q_min) == len(self.q_max) == len(self.qd_max) == len(self.tau_max) == self.num_dof):
            raise ValueError(f"Limit dimensions mismatch: expected {self.num_dof} DOF")
        for i in range(self.num_dof):
            if self.q_min[i] >= self.q_max[i]:
                raise ValueError(f"Joint {i}: q_min ({self.q_min[i]}) >= q_max ({self.q_max[i]})")
            if self.qd_max[i] <= 0:
                raise ValueError(f"Joint {i}: qd_max must be positive")

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class RobotState:
    """Current dynamic state of the physical robot."""
    num_dof: int
    q: List[float]
    qd: List[float]
    tau: List[float]
    ee_pos: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    timestamp_us: int = 0
    is_emergency: bool = False
    metadata: Dict[str, Any] = field(default_factory=dict)

    def is_within_limits(self, limits: SafetyLimits, tolerance: float = 1e-3) -> bool:
        if self.num_dof != limits.num_dof:
            return False
        for i in range(self.num_dof):
            if self.q[i] < limits.q_min[i] - tolerance or self.q[i] > limits.q_max[i] + tolerance:
                return False
            if abs(self.qd[i]) > limits.qd_max[i] + tolerance:
                return False
            if abs(self.tau[i]) > limits.tau_max[i] + tolerance:
                return False
        return True

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), sort_keys=True)
