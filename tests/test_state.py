import unittest
from kineticguard.core.state import RobotState, SafetyLimits


class TestRobotState(unittest.TestCase):

    def setUp(self):
        self.limits = SafetyLimits(
            num_dof=3,
            q_min=[-1.0, -1.0, -1.0],
            q_max=[1.0, 1.0, 1.0],
            qd_max=[2.0, 2.0, 2.0],
            tau_max=[50.0, 50.0, 50.0],
            qdd_max=[10.0, 10.0, 10.0]
        )

    def test_validation_success(self):
        self.limits.validate()

    def test_validation_error_on_inversion(self):
        broken = SafetyLimits(
            num_dof=3,
            q_min=[2.0, -1.0, -1.0],
            q_max=[1.0, 1.0, 1.0],
            qd_max=[2.0, 2.0, 2.0],
            tau_max=[50.0, 50.0, 50.0],
            qdd_max=[10.0, 10.0, 10.0]
        )
        with self.assertRaises(ValueError):
            broken.validate()

    def test_within_limits(self):
        s_safe = RobotState(num_dof=3, q=[0.0, 0.5, -0.5], qd=[1.0, 0.0, -1.0], tau=[20.0, 10.0, 0.0])
        s_unsafe = RobotState(num_dof=3, q=[1.5, 0.0, 0.0], qd=[0.0, 0.0, 0.0], tau=[0.0, 0.0, 0.0])

        self.assertTrue(s_safe.is_within_limits(self.limits))
        self.assertFalse(s_unsafe.is_within_limits(self.limits))


if __name__ == "__main__":
    unittest.main()
