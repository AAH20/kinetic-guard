import unittest
import time
from kineticguard.core.state import RobotState, SafetyLimits
from kineticguard.reflex.watchdog import ReflexWatchdog


class TestReflexWatchdog(unittest.TestCase):

    def setUp(self):
        self.limits = SafetyLimits(
            num_dof=2,
            q_min=[-1.0, -1.0],
            q_max=[1.0, 1.0],
            qd_max=[2.0, 2.0],
            tau_max=[20.0, 20.0],
            qdd_max=[10.0, 10.0]
        )
        self.watchdog = ReflexWatchdog(self.limits, timeout_ms=20.0)

    def test_heartbeat_timeout(self):
        self.watchdog.ping()
        healthy, _ = self.watchdog.evaluate_heartbeat()
        self.assertTrue(healthy)

        # Sleep past timeout
        time.sleep(0.025)
        healthy_after, msg = self.watchdog.evaluate_heartbeat()
        self.assertFalse(healthy_after)
        self.assertTrue(self.watchdog.is_triggered)
        self.assertIn("timeout", msg)

    def test_reflex_stop_computation(self):
        state = RobotState(num_dof=2, q=[0.0, 0.0], qd=[1.5, -1.5], tau=[0.0, 0.0])
        u_stop = self.watchdog.compute_reflex_stop(state, damping_factor=2.0)
        # u_stop should oppose velocity direction
        self.assertLess(u_stop[0], 0.0)
        self.assertGreater(u_stop[1], 0.0)


if __name__ == "__main__":
    unittest.main()
