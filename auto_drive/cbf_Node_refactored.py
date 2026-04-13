#!/usr/bin/env python3
"""
Model-Free Control Barrier Function Node for F1Tenth
Implements the MIMO ULM-based CBF synthesis from:
"Safety via Control Barrier Functions Synthesized from Ultra-Local Models"
"""
import rclpy
import cupy as cp
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from auto_drive.CBF_refactored import ModelFreeCBF
import time


class SafetyULM_EKF:
    """
    Extended Kalman Filter for the safety output second-order MIMO ULM:
    q̈ = F_q + B_q @ u

    State: ξ = [q, q̇, F_q, B_q,v, B_q,ω]^T  (for 2 inputs: v, ω)
    """
    def __init__(self, Ts, m_inputs=2, Q=None, R_q=None, R_qdot=None):
        """
        Args:
            Ts: Sampling period
            m_inputs: Number of control inputs (2 for [v, ω])
            Q: Process noise covariance (default provided)
            R_q: Measurement noise for q (can be updated with GP variance)
            R_qdot: Measurement noise for q̇
        """
        self.Ts = Ts
        self.m = m_inputs

        # State: [q, q̇, F_q, B_q,v, B_q,ω]
        state_dim = 3 + m_inputs
        self.x = cp.zeros(state_dim)

        # Initialize parameters
        self.x[2] = 0.0  # F_q
        self.x[3] = 1.0  # B_q,v
        if m_inputs > 1:
            self.x[4] = 0.1  # B_q,ω

        # Covariance matrix
        P_diag = [1.0, 1.0, 10.0] + [10.0] * m_inputs
        self.P = cp.diag(cp.array(P_diag))

        # Process noise (parameters F_q, B_q evolve slowly)
        if Q is None:
            Q_diag = [1e-6, 1e-5, 1e-3] + [1e-3] * m_inputs
            self.Q = cp.diag(cp.array(Q_diag))
        else:
            self.Q = Q

        # Measurement noise
        self.R_q = 1e-3 if R_q is None else R_q
        self.R_qdot = 1e-2 if R_qdot is None else R_qdot

        # Measurement matrices
        self.H_q = cp.zeros((1, state_dim))
        self.H_q[0, 0] = 1.0  # Measure q

        self.H_qdot = cp.zeros((1, state_dim))
        self.H_qdot[0, 1] = 1.0  # Measure q̇

    def predict(self, u):
        """
        Predict step using the ULM dynamics with Euler discretization.

        Args:
            u: Control input [v, ω]
        """
        Ts = self.Ts
        u = cp.array(u)

        # Extract state
        q = self.x[0]
        qdot = self.x[1]
        F_q = self.x[2]
        B_q = self.x[3:3+self.m]

        # Dynamics: q̈ = F_q + B_q @ u
        qddot = F_q + cp.dot(B_q, u)

        # Euler integration
        q_new = q + Ts * qdot + (Ts**2 / 2) * qddot
        qdot_new = qdot + Ts * qddot

        # Parameters remain constant (random walk model)
        self.x[0] = q_new
        self.x[1] = qdot_new
        # self.x[2:] unchanged in prediction

        # Linearized state transition matrix A
        A = cp.eye(3 + self.m)
        A[0, 1] = Ts
        A[0, 2] = Ts**2 / 2
        A[0, 3:3+self.m] = (Ts**2 / 2) * u
        A[1, 2] = Ts
        A[1, 3:3+self.m] = Ts * u

        # Covariance prediction
        self.P = A @ self.P @ A.T + self.Q

    def update_q(self, q_meas, R_q=None):
        """
        Update with q measurement (from GP).

        Args:
            q_meas: Measured barrier value
            R_q: Measurement noise (e.g., GP posterior variance)
        """
        if R_q is not None:
            self.R_q = R_q

        R = cp.array([[self.R_q]])

        # Innovation
        y = q_meas - self.H_q @ self.x
        S = self.H_q @ self.P @ self.H_q.T + R

        # Kalman gain
        K = self.P @ self.H_q.T / S[0, 0]

        # State update
        self.x = self.x + K.flatten() * y

        # Covariance update
        self.P = (cp.eye(len(self.x)) - cp.outer(K, self.H_q)) @ self.P

    def update_qdot(self, qdot_meas, R_qdot=None):
        """
        Update with q̇ measurement (from GP gradient and position ULM).

        Args:
            qdot_meas: Measured barrier derivative
            R_qdot: Measurement noise
        """
        if R_qdot is not None:
            self.R_qdot = R_qdot

        R = cp.array([[self.R_qdot]])

        # Innovation
        y = qdot_meas - self.H_qdot @ self.x
        S = self.H_qdot @ self.P @ self.H_qdot.T + R

        # Kalman gain
        K = self.P @ self.H_qdot.T / S[0, 0]

        # State update
        self.x = self.x + K.flatten() * y

        # Covariance update
        self.P = (cp.eye(len(self.x)) - cp.outer(K, self.H_qdot)) @ self.P

    def get_estimates(self):
        """
        Returns:
            q_hat, qdot_hat, F_q_hat, B_q_hat, P
        """
        return (
            float(self.x[0]),
            float(self.x[1]),
            float(self.x[2]),
            self.x[3:3+self.m].copy(),
            self.P.copy()
        )


class PositionULM_EKF:
    """
    First-order MIMO ULM for position dynamics:
    ṗ = F_p + B_p @ u

    State: [p_x, p_y, F_p,x, F_p,y, B_p,x,v, B_p,x,ω, B_p,y,v, B_p,y,ω]^T
    """
    def __init__(self, Ts, m_inputs=2, Q=None, R=None):
        self.Ts = Ts
        self.m = m_inputs

        # State: [p_x, p_y, F_p (2), B_p (2x2 flattened)]
        # For 2D position and 2 inputs: 2 + 2 + 4 = 8
        state_dim = 2 + 2 + 2 * m_inputs
        self.x = cp.zeros(state_dim)

        # Initialize B_p to reasonable values (forward velocity affects p_x, etc.)
        self.x[4] = 1.0  # B_p,x,v
        self.x[6] = 0.0  # B_p,y,v
        self.x[5] = 0.0  # B_p,x,ω
        self.x[7] = 0.1  # B_p,y,ω

        # Covariance
        P_diag = [1.0, 1.0, 10.0, 10.0] + [10.0] * (2 * m_inputs)
        self.P = cp.diag(cp.array(P_diag))

        # Process noise
        if Q is None:
            Q_diag = [1e-6, 1e-6, 1e-3, 1e-3] + [1e-3] * (2 * m_inputs)
            self.Q = cp.diag(cp.array(Q_diag))
        else:
            self.Q = Q

        # Measurement noise (position measurements from odometry)
        self.R = cp.diag([1e-3, 1e-3]) if R is None else R

        # Measurement matrix (we measure position)
        self.H = cp.zeros((2, state_dim))
        self.H[0, 0] = 1.0
        self.H[1, 1] = 1.0

    def predict(self, u):
        """Predict step for position ULM."""
        Ts = self.Ts
        u = cp.array(u)

        # Extract state
        p = self.x[0:2]
        F_p = self.x[2:4]
        B_p = self.x[4:4+2*self.m].reshape(2, self.m)

        # Dynamics: ṗ = F_p + B_p @ u
        pdot = F_p + B_p @ u

        # Euler integration
        p_new = p + Ts * pdot
        self.x[0:2] = p_new

        # Linearized transition
        A = cp.eye(len(self.x))
        A[0, 2] = Ts
        A[0, 4:4+self.m] = Ts * u
        A[1, 3] = Ts
        A[1, 4+self.m:4+2*self.m] = Ts * u

        # Covariance prediction
        self.P = A @ self.P @ A.T + self.Q

    def update(self, p_meas):
        """Update with position measurement."""
        p_meas = cp.array(p_meas).reshape(2, 1)

        # Innovation
        y = p_meas - (self.H @ self.x).reshape(2, 1)
        S = self.H @ self.P @ self.H.T + self.R

        # Kalman gain
        K = self.P @ self.H.T @ cp.linalg.inv(S)

        # Update
        self.x = self.x + (K @ y).flatten()
        self.P = (cp.eye(len(self.x)) - K @ self.H) @ self.P

    def get_velocity_estimate(self):
        """
        Returns estimated velocity: F_p + B_p @ u_prev
        (You need to store u_prev for this to be accurate)
        """
        F_p = self.x[2:4]
        return F_p.copy()

    def get_estimates(self):
        """Returns p, F_p, B_p"""
        return (
            self.x[0:2].copy(),
            self.x[2:4].copy(),
            self.x[4:4+2*self.m].reshape(2, self.m).copy()
        )


class ControllerNode(Node):
    def __init__(self):
        super().__init__('ModelFreeCBF_Node')

        # Parameters - HARDWARE TUNED
        self.dt = 0.002  # 20 Hz
        self.v_max = 2.0
        self.v_min = 0.0  # CRITICAL: Allow robot to stop! Was 0.5
        self.omega_max = 0.85
        self.omega_min = -0.85
        self.r_max = 5.0
        self.length_scale = 0.25  # Increased for smoother barrier
        self.sigma_f = 1.0

        # HOCBF parameters (from paper, Section II-C) - RELAXED FOR FEASIBILITY
        self.lambda_0 = 0.5  # Reduced from 1.0 for less aggressive constraints
        self.lambda_1 = 0.5  # Reduced from 1.0 for less aggressive constraints
        self.c_q = 1.0  # Reduced from 2.0 (1-sigma instead of 2-sigma) for less conservative margin

        # State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 1.0

        # Reference command
        self.u_ref = [1.0, 0.0]  # [v_ref, ω_ref]
        self.u_prev = [1.0, 0.0]

        # Initialize EKFs
        self.safety_ekf = SafetyULM_EKF(Ts=self.dt, m_inputs=2)
        self.position_ekf = PositionULM_EKF(Ts=self.dt, m_inputs=2)

        # CBF object
        self.cbf = ModelFreeCBF(
            dt=self.dt,
            u_min=[self.v_min, self.omega_min],
            u_max=[self.v_max, self.omega_max],
            r_max=self.r_max,
            length_scale=self.length_scale,
            sigma_f=self.sigma_f,
            lambda_0=self.lambda_0,
            lambda_1=self.lambda_1,
            c_q=self.c_q
        )

        # ROS2 subscriptions and publishers
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        # Subscribe to reference commands (e.g., from planner or teleop)
        self.create_subscription(AckermannDriveStamped, '/drive_ref', self.drive_ref_callback, 10)
        self.cmd_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        self.get_logger().info('Model-Free CBF Node initialized')
        self.get_logger().info(f'  Control bounds: v∈[{self.v_min}, {self.v_max}], ω∈[{self.omega_min}, {self.omega_max}]')
        self.get_logger().info(f'  CBF params: λ0={self.lambda_0}, λ1={self.lambda_1}, cq={self.c_q}')

    def drive_ref_callback(self, msg):
        """Update reference command from planner/teleop."""
        self.u_ref = [float(msg.drive.speed), float(msg.drive.steering_angle)]

    def odom_callback(self, msg):
        """Update position from odometry."""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Extract heading from quaternion (yaw for planar motion)
        quat = msg.pose.pose.orientation
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        self.theta = float(cp.arctan2(siny_cosp, cosy_cosp))

        self.v = msg.twist.twist.linear.x

        # Update position EKF
        self.position_ekf.update(cp.array([self.x, self.y]))

    def lidar_callback(self, msg):
        """
        Main control loop: process LiDAR, update EKFs, compute safe control.
        """
        start_time = time.time()

        # Step 1: Extract LiDAR data
        ranges = cp.array(msg.ranges, dtype=cp.float32)
        angles = cp.linspace(msg.angle_min, msg.angle_max, len(ranges), dtype=cp.float32)

        # Update CBF with obstacle points
        self.cbf.set_obstacles(ranges, angles)

        # Step 2: Predict EKFs
        self.safety_ekf.predict(self.u_prev)
        self.position_ekf.predict(self.u_prev)

        # Step 3: Compute measurements
        # Get GP posterior mean and variance for q
        q_meas, sigma_gp_sq = self.cbf.get_barrier_and_variance([0.0, 0.0])

        # Get GP gradient for q̇
        grad_h = self.cbf.get_gradient([0.0, 0.0])

        # Get velocity estimate from position ULM
        p_est, F_p, B_p = self.position_ekf.get_estimates()
        p_dot = F_p + B_p @ cp.array(self.u_prev)

        # Compute q̇ measurement: q̇ = ∇h^T ṗ
        qdot_meas = float(grad_h @ p_dot)

        # Compute measurement noise for q̇ (Eq. in Section III-B)
        grad_norm = float(cp.linalg.norm(grad_h))
        sigma_pdot = 0.1  # Position velocity uncertainty (tune this)
        epsilon = 1e-6
        R_qdot = grad_norm**2 * sigma_pdot**2 + sigma_gp_sq / (self.length_scale**2 * (grad_norm**2 + epsilon))

        # Step 4: Update safety EKF with measurements
        self.safety_ekf.update_q(float(q_meas), R_q=float(sigma_gp_sq))
        self.safety_ekf.update_qdot(qdot_meas, R_qdot=float(R_qdot))

        # Step 5: Get estimates for control
        q_hat, qdot_hat, F_q_hat, B_q_hat, P_safety = self.safety_ekf.get_estimates()

        # Step 6: Compute safe control
        try:
            u_safe = self.cbf.compute_safe_control(
                u_ref=self.u_ref,
                q_hat=q_hat,
                qdot_hat=qdot_hat,
                F_q_hat=F_q_hat,
                B_q_hat=B_q_hat,
                P=P_safety
            )
        except Exception as e:
            self.get_logger().error(f'CBF QP failed: {e}')
            u_safe = [0.0, 0.0]

        # Step 7: Send command
        self.send_command(u_safe[0], u_safe[1])
        self.u_prev = u_safe

        total_time = time.time() - start_time

        # Log control status
        u_modified = (abs(u_safe[0] - self.u_ref[0]) > 0.01) or (abs(u_safe[1] - self.u_ref[1]) > 0.01)
        status = "CBF ACTIVE" if u_modified else "SAFE"

        self.get_logger().info(
            f'[{status}] t={total_time:.3f}s | '
            f'q={q_hat:.3f} | q̇={qdot_hat:.3f} | '
            f'F_q={F_q_hat:.3f} | B_q={B_q_hat[0]:.3f},{B_q_hat[1]:.3f} | '
            f'u_ref=[{self.u_ref[0]:.2f}, {self.u_ref[1]:.2f}] → '
            f'u_safe=[{u_safe[0]:.2f}, {u_safe[1]:.2f}]'
        )

    def send_command(self, v, omega):
        """Publish Ackermann drive command."""
        msg = AckermannDriveStamped()
        msg.drive.speed = float(v)
        msg.drive.steering_angle = -float(omega)
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
