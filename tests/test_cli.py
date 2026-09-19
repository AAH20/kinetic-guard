import unittest
from kineticguard.core.state import RobotState
from kineticguard.adapters.openclaw_bridge import kinetic_safe
from kineticguard.cli import get_default_7dof_limits, run_benchmark


class TestCLIAndAdapters(unittest.TestCase):

    def test_openclaw_adapter(self):
        limits = get_default_7dof_limits()

        @kinetic_safe(limits=limits)
        def mock_hallucinating_skill(state: RobotState) -> list:
            return [10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Grossly over speed

        s = RobotState(num_dof=7, q=[0.0]*7, qd=[0.0]*7, tau=[0.0]*7)
        safe_action = mock_hallucinating_skill(s)

        # Output should be clamped to qd_max[0] (2.17 rad/s)
        self.assertAlmostEqual(safe_action[0], limits.qd_max[0], places=1)
        self.assertLessEqual(safe_action[0], limits.qd_max[0] + 1e-4)

    def test_benchmark_runs_without_error(self):
        # Quick 50-cycle benchmark test
        run_benchmark(num_cycles=50)


if __name__ == "__main__":
    unittest.main()
