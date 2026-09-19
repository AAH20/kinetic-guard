import unittest
from kineticguard.core.qp_solver import FastQPSolver


class TestQPSolver(unittest.TestCase):

    def setUp(self):
        self.solver = FastQPSolver(max_iter=150, step_size=0.05)

    def test_strictly_feasible_action(self):
        u_des = [0.5, 0.5]
        # Constraints: u_0 <= 1.0, u_1 <= 1.0, -u_0 <= 1.0, -u_1 <= 1.0
        A = [[1.0, 0.0], [0.0, 1.0], [-1.0, 0.0], [0.0, -1.0]]
        b = [1.0, 1.0, 1.0, 1.0]

        res = self.solver.project(u_des, A, b)
        self.assertFalse(res.was_filtered)
        self.assertEqual(res.u_safe, [0.5, 0.5])
        self.assertTrue(res.is_safe)

    def test_infeasible_action_projection(self):
        u_des = [3.0, 0.0]  # u_0 exceeds 1.0
        A = [[1.0, 0.0], [0.0, 1.0], [-1.0, 0.0], [0.0, -1.0]]
        b = [1.0, 1.0, 1.0, 1.0]

        res = self.solver.project(u_des, A, b)
        self.assertTrue(res.was_filtered)
        # u_safe[0] should be clamped to 1.0
        self.assertAlmostEqual(res.u_safe[0], 1.0, places=2)
        self.assertAlmostEqual(res.u_safe[1], 0.0, places=2)
        self.assertTrue(res.solve_time_us < 2000.0)  # Sub-2ms execution


if __name__ == "__main__":
    unittest.main()
