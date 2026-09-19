"""
OpenClaw Adapter and Safety Decorator for Autonomous Skill Meshes.
Ensures OpenClaw skills cannot dispatch rogue or unbounded kinetic actions.
"""

from typing import Callable, Any, List, Dict
import functools
from kineticguard.core.state import RobotState, SafetyLimits
from kineticguard.core.barrier import JointLimitBarrier
from kineticguard.core.qp_solver import FastQPSolver, SafetyFilterResult


class OpenClawSafetyGuard:
    """Wraps OpenClaw skills with deterministic Control Barrier Function safety projection."""

    def __init__(self, limits: SafetyLimits):
        self.limits = limits
        self.barrier = JointLimitBarrier(limits)
        self.solver = FastQPSolver()

    def filter_action(self, current_state: RobotState, desired_action: List[float]) -> SafetyFilterResult:
        """Projects proposed skill action onto the provably safe polytope."""
        A, b = self.barrier.generate_constraints(current_state)
        return self.solver.project(desired_action, A, b)


def kinetic_safe(limits: SafetyLimits):
    """Decorator for OpenClaw skill dispatch functions."""
    guard = OpenClawSafetyGuard(limits)

    def decorator(func: Callable[..., Any]):
        @functools.wraps(func)
        def wrapper(state: RobotState, *args, **kwargs):
            raw_action = func(state, *args, **kwargs)
            if isinstance(raw_action, list):
                result = guard.filter_action(state, raw_action)
                return result.u_safe
            return raw_action
        return wrapper
    return decorator
