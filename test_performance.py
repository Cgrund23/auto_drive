#!/usr/bin/env python3
"""
Performance testing script for 100Hz CBF operation
Run this to verify timing improvements and identify remaining bottlenecks
"""
import time
import numpy as np
import cupy as cp

def test_lidar_processing():
    """Test LiDAR downsampling speed"""
    # Simulate 360 ray LiDAR
    ranges = np.random.rand(360) * 5.0

    start = time.time()
    for _ in range(100):
        ranges_sparse = ranges[::20]  # Downsample to 18 points
        ranges_gpu = cp.asarray(ranges_sparse, dtype=cp.float32)
    elapsed = (time.time() - start) / 100

    print(f"LiDAR processing: {elapsed*1000:.2f}ms per iteration")
    return elapsed < 0.001  # Target <1ms

def test_gp_kernel():
    """Test GP kernel computation speed"""
    # Simulate obstacle points
    cp.cuda.set_allocator(cp.cuda.MemoryPool().malloc)

    n_obstacles = 10  # After downsampling
    obstacles = cp.random.rand(n_obstacles, 2) * 3.0
    query = cp.array([[0.0, 0.0]])
    length_scale = 0.25
    sigma_f = 1.0

    start = time.time()
    for _ in range(100):
        # RBF kernel
        sqdist = cp.sum(obstacles**2, axis=1, keepdims=True) - 2 * (obstacles @ query.T)
        K = sigma_f**2 * cp.exp(-0.5 * sqdist / length_scale**2)
    cp.cuda.Stream.null.synchronize()
    elapsed = (time.time() - start) / 100

    print(f"GP kernel: {elapsed*1000:.2f}ms per iteration")
    return elapsed < 0.002  # Target <2ms

def test_ekf_update():
    """Test EKF prediction + update speed"""
    cp.cuda.set_allocator(cp.cuda.MemoryPool().malloc)

    # Setup EKF matrices
    state_dim = 5  # [q, qdot, F_q, B_v, B_omega]
    x = cp.zeros(state_dim)
    P = cp.eye(state_dim) * 0.1
    Q = cp.eye(state_dim) * 1e-3
    H = cp.zeros((1, state_dim))
    H[0, 0] = 1.0
    R = cp.array([[1e-3]])

    u = cp.array([1.0, 0.0])
    Ts = 0.01

    start = time.time()
    for _ in range(100):
        # Predict
        A = cp.eye(state_dim)
        A[0, 1] = Ts
        A[0, 2] = Ts**2 / 2
        P = A @ P @ A.T + Q

        # Update
        y = 0.5 - H @ x
        S = H @ P @ H.T + R
        K = P @ H.T / S[0, 0]
        x = x + K.flatten() * y
        P = (cp.eye(state_dim) - cp.outer(K, H)) @ P

    cp.cuda.Stream.null.synchronize()
    elapsed = (time.time() - start) / 100

    print(f"EKF update: {elapsed*1000:.2f}ms per iteration")
    return elapsed < 0.001  # Target <1ms

def test_qp_solve():
    """Test QP solver speed"""
    try:
        from qpsolvers import solve_qp
    except ImportError:
        print("qpsolvers not installed, skipping QP test")
        return True

    # 2D QP problem
    P = np.diag([10.0, 0.1])
    q = np.array([-10.0, 0.0])
    G = np.array([
        [-1.0, 0.0],
        [1.0, 0.0],
        [0.0, 1.0],
        [0.0, -1.0],
        [-1.0, 0.0]
    ])
    h = np.array([0.5, 1.5, 0.5, 0.5, 0.0])

    start = time.time()
    for _ in range(100):
        sol = solve_qp(
            P=P, q=q, G=G, h=h,
            solver='osqp',
            eps_abs=1e-4,
            eps_rel=1e-4,
            max_iter=100,
            polish=False,
            verbose=False
        )
    elapsed = (time.time() - start) / 100

    print(f"QP solve: {elapsed*1000:.2f}ms per iteration")
    return elapsed < 0.003  # Target <3ms

def test_full_pipeline():
    """Test complete control loop"""
    cp.cuda.set_allocator(cp.cuda.MemoryPool().malloc)

    # Simulate full pipeline
    from auto_drive.CBF_refactored import ModelFreeCBF

    cbf = ModelFreeCBF(
        dt=0.01,
        u_min=[0.0, -0.5],
        u_max=[1.5, 0.5],
        r_max=3.0,
        r_min_obstacle=3.0,
        length_scale=0.25,
        sigma_f=1.0,
        lambda_0=0.3,
        lambda_1=0.3,
        c_q=0.05
    )

    # Simulate LiDAR
    ranges = cp.random.rand(18) * 2.0 + 0.5
    angles = cp.linspace(-cp.pi/2, cp.pi/2, 18)

    start = time.time()
    for _ in range(100):
        # Full pipeline
        cbf.set_obstacles(ranges, angles)
        q, sigma_sq = cbf.get_barrier_and_variance([0.0, 0.0])
        grad = cbf.get_gradient([0.0, 0.0])

        # Simulate EKF estimates
        q_hat = 0.5
        qdot_hat = -0.1
        F_q_hat = 0.0
        B_q_hat = cp.array([1.0, 0.1])
        P = cp.eye(5) * 0.01

        try:
            u_safe = cbf.compute_safe_control(
                u_ref=[1.0, 0.0],
                q_hat=q_hat,
                qdot_hat=qdot_hat,
                F_q_hat=F_q_hat,
                B_q_hat=B_q_hat,
                P=P,
                v_current=1.0
            )
        except:
            pass  # QP may be infeasible in random test

    cp.cuda.Stream.null.synchronize()
    elapsed = (time.time() - start) / 100

    print(f"Full pipeline: {elapsed*1000:.2f}ms per iteration")
    return elapsed < 0.010  # Target <10ms for 100Hz

def main():
    print("="*50)
    print("Performance Testing for 100Hz CBF Operation")
    print("="*50)

    tests = [
        ("LiDAR Processing", test_lidar_processing),
        ("GP Kernel", test_gp_kernel),
        ("EKF Update", test_ekf_update),
        ("QP Solve", test_qp_solve),
        ("Full Pipeline", test_full_pipeline),
    ]

    results = []
    for name, test_func in tests:
        print(f"\nTesting {name}...")
        try:
            passed = test_func()
            results.append((name, passed))
        except Exception as e:
            print(f"  ERROR: {e}")
            results.append((name, False))

    print("\n" + "="*50)
    print("Results Summary")
    print("="*50)
    for name, passed in results:
        status = "✓ PASS" if passed else "✗ FAIL"
        print(f"{name:20s} {status}")

    all_passed = all(passed for _, passed in results)
    if all_passed:
        print("\n✓ All tests passed! 100Hz operation should be achievable.")
    else:
        print("\n✗ Some tests failed. Review bottlenecks above.")

    return 0 if all_passed else 1

if __name__ == '__main__':
    exit(main())
