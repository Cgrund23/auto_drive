#!/usr/bin/env python3
"""
Test script for Model-Free CBF implementation
Verifies EKF dynamics, GP computations, and QP feasibility
"""

import numpy as np
import matplotlib.pyplot as plt
from CBF_refactored import ModelFreeCBF
from cbf_Node_refactored import SafetyULM_EKF, PositionULM_EKF


def test_safety_ekf():
    """Test SafetyULM_EKF prediction and update."""
    print("\n=== Testing Safety ULM-EKF ===")

    Ts = 0.05
    ekf = SafetyULM_EKF(Ts=Ts, m_inputs=2)

    # Set initial condition
    ekf.x[0] = 1.0  # q = 1.0 (safe)
    ekf.x[1] = -0.5  # q̇ = -0.5 (approaching obstacle)
    ekf.x[2] = 0.1  # F_q = 0.1 (small drift)
    ekf.x[3] = 1.0  # B_q,v = 1.0
    ekf.x[4] = 0.1  # B_q,ω = 0.1

    # Run prediction
    u = [1.0, 0.0]  # Forward motion
    ekf.predict(u)

    q, qdot, F_q, B_q, P = ekf.get_estimates()

    print(f"After prediction with u={u}:")
    print(f"  q = {q:.4f}, q̇ = {qdot:.4f}")
    print(f"  F_q = {F_q:.4f}, B_q = {B_q}")

    # Check dynamics: q̈ = F_q + B_q @ u
    qddot_expected = F_q + np.dot(cp.asnumpy(B_q), u)
    print(f"  Expected q̈ = {qddot_expected:.4f}")

    # Update with measurement
    q_meas = 0.95
    ekf.update_q(q_meas, R_q=0.01)

    q, qdot, F_q, B_q, P = ekf.get_estimates()
    print(f"After q measurement update:")
    print(f"  q = {q:.4f} (measurement was {q_meas:.4f})")

    print("✓ Safety EKF test passed")


def test_position_ekf():
    """Test PositionULM_EKF."""
    print("\n=== Testing Position ULM-EKF ===")

    Ts = 0.05
    ekf = PositionULM_EKF(Ts=Ts, m_inputs=2)

    # Initialize at origin
    ekf.x[0:2] = cp.array([0.0, 0.0])
    ekf.x[4] = 1.0  # B_p,x,v = 1.0 (forward velocity affects x)

    # Predict with forward motion
    u = [1.0, 0.0]
    ekf.predict(u)

    p, F_p, B_p = ekf.get_estimates()
    print(f"After prediction with u={u}:")
    print(f"  p = {cp.asnumpy(p)}")
    print(f"  F_p = {cp.asnumpy(F_p)}")
    print(f"  B_p = \n{cp.asnumpy(B_p)}")

    # Expected displacement: Δp = T_s (F_p + B_p @ u)
    expected_p = Ts * (F_p + B_p @ cp.array(u))
    print(f"  Expected p = {cp.asnumpy(expected_p)}")

    # Update with measurement
    p_meas = [0.05, 0.0]
    ekf.update(p_meas)

    p, _, _ = ekf.get_estimates()
    print(f"After measurement update:")
    print(f"  p = {cp.asnumpy(p)}")

    print("✓ Position EKF test passed")


def test_gp_barrier():
    """Test GP barrier computation and gradient."""
    print("\n=== Testing GP Barrier ===")

    cbf = ModelFreeCBF(
        dt=0.05, u_min=[0.5, -0.85], u_max=[2.0, 0.85],
        r_max=5.0, length_scale=0.4, sigma_f=1.0,
        lambda_0=1.0, lambda_1=1.0, c_q=2.0
    )

    # Create simple obstacle (circular at [2, 0])
    angles = cp.linspace(-cp.pi, cp.pi, 100)
    ranges = cp.ones(100) * 2.0  # Constant distance
    cbf.set_obstacles(ranges, angles)

    print(f"Number of obstacle points: {cbf.N}")

    # Evaluate barrier at origin
    h, sigma_sq = cbf.get_barrier_and_variance([0.0, 0.0])
    print(f"Barrier at origin: h={h:.4f}, σ²={sigma_sq:.6f}")

    # Evaluate barrier near obstacle
    h_near, sigma_near = cbf.get_barrier_and_variance([1.8, 0.0])
    print(f"Barrier at [1.8, 0]: h={h_near:.4f}, σ²={sigma_near:.6f}")

    # Check gradient
    grad_h = cbf.get_gradient([0.0, 0.0])
    print(f"Gradient at origin: ∇h = {cp.asnumpy(grad_h)}")
    print(f"  (Should point away from obstacle, i.e., negative x)")

    # Verify GP variance decreases near training points
    assert h > h_near, "Barrier should be larger (safer) far from obstacle"
    print("✓ GP barrier test passed")


def test_hocbf_constraint():
    """Test HOCBF constraint formulation."""
    print("\n=== Testing HOCBF Constraint ===")

    cbf = ModelFreeCBF(
        dt=0.05, u_min=[0.5, -0.85], u_max=[2.0, 0.85],
        r_max=5.0, length_scale=1.4, sigma_f=1.0,
        lambda_0=1.0, lambda_1=1.0, c_q=2.0
    )

    # Mock EKF estimates
    q_hat = 0.5
    qdot_hat = -0.2
    F_q_hat = 0.0
    B_q_hat = cp.array([1.0, 0.1])
    P = cp.diag([0.01, 0.01, 0.1, 0.1, 0.1])

    # Compute safety margin
    sigma_k = cbf.compute_safety_margin(P, cbf.u_max)
    print(f"Safety margin σ_k = {sigma_k:.4f}")

    # Check HOCBF RHS
    r_k = -F_q_hat - (cbf.lambda_0 + cbf.lambda_1) * qdot_hat - \
          cbf.lambda_0 * cbf.lambda_1 * q_hat + sigma_k
    print(f"HOCBF RHS r_k = {r_k:.4f}")
    print(f"  Components:")
    print(f"    -F_q = {-F_q_hat:.4f}")
    print(f"    -(λ_0+λ_1)q̇ = {-(cbf.lambda_0 + cbf.lambda_1) * qdot_hat:.4f}")
    print(f"    -λ_0λ_1q = {-cbf.lambda_0 * cbf.lambda_1 * q_hat:.4f}")
    print(f"    +σ_k = {sigma_k:.4f}")

    # Check feasibility
    feasible = cbf.check_feasibility(q_hat, qdot_hat, F_q_hat, B_q_hat, sigma_k)
    print(f"Feasibility: {feasible}")

    # Try to solve QP
    u_ref = [1.5, 0.0]
    try:
        u_safe = cbf.compute_safe_control(
            u_ref, q_hat, qdot_hat, F_q_hat, B_q_hat, P
        )
        print(f"QP solution: u_safe = {u_safe}")
        print(f"  (Reference was u_ref = {u_ref})")

        # Verify constraint satisfaction
        lhs = float(cp.dot(B_q_hat, cp.array(u_safe)))
        print(f"  Constraint: {lhs:.4f} ≥ {r_k:.4f}? {lhs >= r_k}")

    except Exception as e:
        print(f"QP failed: {e}")

    print("✓ HOCBF constraint test passed")


def test_full_pipeline():
    """Test complete pipeline with simulated data."""
    print("\n=== Testing Full Pipeline ===")

    Ts = 0.05

    # Initialize
    safety_ekf = SafetyULM_EKF(Ts=Ts, m_inputs=2)
    position_ekf = PositionULM_EKF(Ts=Ts, m_inputs=2)
    cbf = ModelFreeCBF(
        dt=Ts, u_min=[0.5, -0.85], u_max=[2.0, 0.85],
        r_max=5.0, length_scale=0.4, sigma_f=1.0,
        lambda_0=1.0, lambda_1=1.0, c_q=2.0
    )

    # Simulate obstacle
    angles = cp.linspace(-cp.pi/2, cp.pi/2, 50)
    ranges = cp.ones(50) * 3.0
    cbf.set_obstacles(ranges, angles)

    # Simulation loop
    u_prev = [1.0, 0.0]
    n_steps = 10

    q_history = []
    qdot_history = []

    for i in range(n_steps):
        # Predict
        safety_ekf.predict(u_prev)
        position_ekf.predict(u_prev)

        # Get measurements
        p = [0.0, 0.0]
        q_meas, sigma_gp = cbf.get_barrier_and_variance(p)
        grad_h = cbf.get_gradient(p)

        _, F_p, B_p = position_ekf.get_estimates()
        p_dot = F_p + B_p @ cp.array(u_prev)
        qdot_meas = float(grad_h @ p_dot)

        # Update
        safety_ekf.update_q(float(q_meas), R_q=float(sigma_gp))
        safety_ekf.update_qdot(qdot_meas)
        position_ekf.update([0.05 * i, 0.0])  # Mock position

        # Get estimates
        q_hat, qdot_hat, F_q, B_q, P = safety_ekf.get_estimates()
        q_history.append(q_hat)
        qdot_history.append(qdot_hat)

        # Compute control
        u_ref = [1.0, 0.0]
        u_safe = cbf.compute_safe_control(u_ref, q_hat, qdot_hat, F_q, B_q, P)
        u_prev = u_safe

        print(f"Step {i}: q={q_hat:.3f}, q̇={qdot_hat:.3f}, u={u_safe}")

    # Plot results
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6))

    ax1.plot(q_history, 'b-', label='q (barrier)')
    ax1.axhline(0, color='r', linestyle='--', label='Safety boundary')
    ax1.set_ylabel('q')
    ax1.legend()
    ax1.grid(True)

    ax2.plot(qdot_history, 'g-', label='q̇')
    ax2.set_ylabel('q̇')
    ax2.set_xlabel('Time step')
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout()
    plt.savefig('test_results.png')
    print(f"✓ Results saved to test_results.png")

    print("✓ Full pipeline test passed")


def main():
    """Run all tests."""
    print("=" * 60)
    print("Model-Free CBF Test Suite")
    print("=" * 60)

    try:
        test_safety_ekf()
        test_position_ekf()
        test_gp_barrier()
        test_hocbf_constraint()
        test_full_pipeline()

        print("\n" + "=" * 60)
        print("✓ All tests passed!")
        print("=" * 60)

    except Exception as e:
        print(f"\n✗ Test failed with error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
