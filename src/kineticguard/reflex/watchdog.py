"""
Hardware-in-the-Loop Reflex Watchdog & Emergency Deceleration Daemon.
Intervenes within microseconds when an upstream agent freezes, oscillates, or drops packets.
"""

from typing import List, Tuple, Dict, Any, Optional
import time
from kineticguard.core.state import RobotState, SafetyLimits


class ReflexWatchdog:
    """Watchdog monitor guarding against agent stalls, latency spikes, and trajectory oscillations."""

    def __init__(self, limits: SafetyLimits, timeout_ms: float = 50.0, max_jerk: float = 80.0):
        self.limits = limits
        self.timeout_ms = timeout_ms
        self.max_jerk = max_jerk
        self.last_heartbeat_us: int = int(time.time() * 1e6)
        self.command_history: List[Tuple[int, List[float]]] = []
        self.is_triggered: bool = False
        self.trigger_reason: str = ""

    def ping(self) -> None:
        """Called by upstream agent upon valid command dispatch."""
        self.last_heartbeat_us = int(time.time() * 1e6)

    def evaluate_heartbeat(self) -> Tuple[bool, str]:
        """Checks if upstream agent has stalled past timeout limit."""
        now_us = int(time.time() * 1e6)
        elapsed_ms = (now_us - self.last_heartbeat_us) / 1000.0
        if elapsed_ms > self.timeout_ms:
            self.is_triggered = True
            self.trigger_reason = f"Agent heartbeat timeout: {elapsed_ms:.2f}ms > {self.timeout_ms:.2f}ms"
            return False, self.trigger_reason
        return True, "Heartbeat healthy"

    def register_command(self, u_cmd: List[float]) -> Tuple[bool, str]:
        """Monitors command trajectory for high-frequency oscillation / jerk spikes."""
        now_us = int(time.time() * 1e6)
        self.command_history.append((now_us, list(u_cmd)))
        # Keep last 10 commands
        if len(self.command_history) > 10:
            self.command_history.pop(0)

        if len(self.command_history) >= 3:
            t_prev, u_prev = self.command_history[-2]
            dt_s = max(1e-4, (now_us - t_prev) / 1e6)
            for i in range(self.limits.num_dof):
                accel = abs(u_cmd[i] - u_prev[i]) / dt_s
                if accel > self.limits.qdd_max[i] * 1.5:
                    self.is_triggered = True
                    self.trigger_reason = f"Violent jerk/acceleration spike on joint {i}: {accel:.1f} rad/s^2"
                    return False, self.trigger_reason

        return True, "Command accepted"

    def compute_reflex_stop(self, state: RobotState, damping_factor: float = 4.0) -> List[float]:
        """
        Computes emergency deceleration velocity command:
        u_stop = -damping * qd (smooth exponential deceleration to zero)
        """
        u_stop = []
        for i in range(state.num_dof):
            # Smoothly decelerate joint velocity towards zero
            safe_dec = -min(self.limits.qd_max[i], damping_factor * state.qd[i])
            u_stop.append(round(safe_dec, 6))
        return u_stop

    def reset(self) -> None:
        self.is_triggered = False
        self.trigger_reason = ""
        self.last_heartbeat_us = int(time.time() * 1e6)
        self.command_history.clear()
