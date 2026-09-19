"""
KineticGuard-RT (SafeVLA)
Deterministic Control Barrier Function (CBF) Safety Kernel & Reflex Middleware for Physical AI.
"""

__version__ = "1.0.0"
__author__ = "Ahmed Hassan (A2Z SOC)"

from kineticguard.core.state import RobotState, SafetyLimits
from kineticguard.core.barrier import BarrierFunction, JointLimitBarrier, ObstacleBarrier
from kineticguard.core.qp_solver import FastQPSolver, SafetyFilterResult
from kineticguard.reflex.watchdog import ReflexWatchdog
from kineticguard.compliance.machinery_audit import MachineryAuditCertificate, generate_machinery_audit

__all__ = [
    "RobotState",
    "SafetyLimits",
    "BarrierFunction",
    "JointLimitBarrier",
    "ObstacleBarrier",
    "FastQPSolver",
    "SafetyFilterResult",
    "ReflexWatchdog",
    "MachineryAuditCertificate",
    "generate_machinery_audit",
]
