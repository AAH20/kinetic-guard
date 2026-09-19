import unittest
from kineticguard.core.state import RobotState, SafetyLimits
from kineticguard.core.barrier import JointLimitBarrier, ObstacleBarrier


class TestBarrier(unittest.TestCase):

    def setUp(self):
        self.limits = SafetyLimits(
            num_dof=2,
            q_min=[-1.0, -1.0],
            q_max=[1.0, 1.0],
            qd_max=[2.0, 2.0],
            tau_max=[20.0, 20.0],
            qdd_max=[5.0, 5.0]
        )

    def test_constraint_generation_velocity_only(self):
        barrier = JointLimitBarrier(self.limits, gamma=10.0, dt=0.01, enforce_acceleration=False)
        state = RobotState(num_dof=2, q=[0.0, 0.0], qd=[0.0, 0.0], tau=[0.0, 0.0])
        A, b = barrier.generate_constraints(state)
        self.assertEqual(len(A), 4)  # 2 upper + 2 lower
        self.assertEqual(len(b), 4)

    def test_constraint_generation_with_acceleration(self):
        barrier = JointLimitBarrier(self.limits, gamma=10.0, dt=0.01, enforce_acceleration=True)
        state = RobotState(num_dof=2, q=[0.0, 0.0], qd=[0.0, 0.0], tau=[0.0, 0.0])
        A, b = barrier.generate_constraints(state)
        self.assertEqual(len(A), 8)  # 2 upper + 2 lower + 4 acc limits
        self.assertEqual(len(b), 8)

    def test_obstacle_barrier_evaluation(self):
        obs = ObstacleBarrier(obstacle_center=[1.0, 0.0, 0.0], obstacle_radius=0.5, safety_margin=0.1)
        s_far = RobotState(num_dof=3, q=[0.0]*3, qd=[0.0]*3, tau=[0.0]*3, ee_pos=[0.0, 0.0, 0.0])
        s_colliding = RobotState(num_dof=3, q=[0.0]*3, qd=[0.0]*3, tau=[0.0]*3, ee_pos=[0.9, 0.0, 0.0])

        self.assertGreater(obs.evaluate(s_far), 0.0)
        self.assertLess(obs.evaluate(s_colliding), 0.0)


if __name__ == "__main__":
    unittest.main()
