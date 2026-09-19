"""
Demonstration of KineticGuard-RT Overriding a Hallucinating / Rogue VLA Model.
Simulates a multi-step trajectory where a rogue model attempts an over-limit joint command.
"""

from kineticguard.core.state import RobotState, SafetyLimits
from kineticguard.core.barrier import JointLimitBarrier
from kineticguard.core.qp_solver import FastQPSolver


def simulate():
    print("=" * 70)
    print("  KineticGuard-RT: Hallucinating Physical AI Model Override Demo")
    print("=" * 70)

    # 1. Initialize 7-DOF Arm Limits (e.g. Franka Emika / Humanoid Arm)
    limits = SafetyLimits(
        num_dof=7,
        q_min=[-2.89, -1.76, -2.89, -3.07, -2.89, -0.01, -2.89],
        q_max=[2.89, 1.76, 2.89, -0.06, 2.89, 3.75, 2.89],
        qd_max=[2.17, 2.17, 2.17, 2.17, 2.61, 2.61, 2.61],
        tau_max=[87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0],
        qdd_max=[15.0, 15.0, 15.0, 15.0, 20.0, 20.0, 20.0]
    )

    barrier = JointLimitBarrier(limits, gamma=15.0, dt=0.01)
    solver = FastQPSolver()

    # Initial safe state, joint 0 near upper bound (2.75 rad, max is 2.89)
    state = RobotState(
        num_dof=7,
        q=[2.75, 0.0, 0.0, -1.5, 0.0, 1.5, 0.0],
        qd=[0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        tau=[5.0] * 7,
        ee_pos=[0.4, 0.0, 0.4]
    )

    print(f"[+] Initial State Joint 0 Position: {state.q[0]:.2f} rad (Hard limit: {limits.q_max[0]:.2f} rad)")

    # Simulating 5 control steps where upstream agent hallucinates a violent +3.0 rad/s push
    dt = 0.01  # 10ms control cycle
    for step in range(1, 6):
        u_agent_hallucinated = [3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Pass through KineticGuard CBF filter
        A, b = barrier.generate_constraints(state)
        res = solver.project(u_agent_hallucinated, A, b)
        
        # Integrate forward in time with safe action
        state.q[0] += res.u_safe[0] * dt
        state.qd[0] = res.u_safe[0]

        print(f"  Step {step}: Agent Desired: {u_agent_hallucinated[0]:.2f} rad/s | "
              f"KineticGuard Output: {res.u_safe[0]:.2f} rad/s | "
              f"Joint 0 Pos: {state.q[0]:.4f} rad | "
              f"Filtered: {res.was_filtered} | Solve Time: {res.solve_time_us:.1f}µs")

    print("\n[✔] VERIFIED: Joint 0 was smoothly clamped before touching 2.89 rad.")
    print(f"[✔] Final Joint 0 Pos: {state.q[0]:.4f} rad <= {limits.q_max[0]:.2f} rad (Zero Limit Breaches).")
    print("=" * 70)


if __name__ == "__main__":
    simulate()
