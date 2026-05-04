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

        # Covariance matrix - HEAVILY REDUCED for faster convergence
        P_diag = [0.01, 0.01, 0.1] + [0.1] * m_inputs
        self.P = cp.diag(cp.array(P_diag))

        # Process noise (parameters F_q, B_q evolve slowly but need correction ability)
        # REDUCED for hardware stability - high process noise causes drift
        if Q is None:
            Q_diag = [1e-6, 1e-5, 1e-3] + [1e-3] * m_inputs  # Reduced from 1e-2 to 1e-3
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

    def reset_if_diverged(self):
        """
        Reset EKF if B_q estimates diverge (become negative or too large).
        B_q[0] should be positive - velocity approaching obstacle decreases barrier.
        """
        B_q = self.x[3:3+self.m]

        # Check if diverged
        B_q_v = float(B_q[0])
        if B_q_v < 0.05 or B_q_v > 5.0:
            # Log the divergence for debugging
            reason = "too small" if B_q_v < 0.05 else "too large"
            print(f"EKF DIVERGENCE: B_q,v = {B_q_v:.4f} ({reason})")

            # Reset parameters to SAME initial values as __init__ for consistency
            self.x[2] = 0.0  # F_q
            self.x[3] = 1.0  # B_q,v (MATCH line 43!)
            if self.m > 1:
                self.x[4] = 0.1  # B_q,ω

            # Reset covariance for parameters only (keep state estimates)
            # Use SAME values as __init__ line 48
            P_diag = [0.01, 0.01, 0.1] + [0.1] * self.m
            self.P = cp.diag(cp.array(P_diag))

            return True
        return False

    def get_estimates(self):
        """
        Returns:
            q_hat, qdot_hat, F_q_hat, B_q_hat, P
        """
        # Constrain B_q to reasonable bounds (clip before converting to float)
        B_q = self.x[3:3+self.m].copy()
        B_q[0] = cp.clip(B_q[0], 0.05, 3.0)  # Velocity effect must be positive
        if self.m > 1:
            B_q[1] = cp.clip(B_q[1], -1.0, 1.0)  # Steering effect bounded

        return (
            float(self.x[0]),
            float(self.x[1]),
            float(self.x[2]),
            B_q,
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

        # Covariance - HEAVILY REDUCED for faster convergence
        P_diag = [0.01, 0.01, 0.1, 0.1] + [0.1] * (2 * m_inputs)
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
        self.dt = 0.02  # 20 Hz
        self.v_max = 1.5
        self.v_min = 0.0  # CRITICAL: Allow robot to stop! Was 0.5
        self.omega_max = 0.5
        self.omega_min = -0.5
        self.r_max = 5.0  # Max range for LiDAR and obstacle detection (meters)
        self.length_scale = 1.0  # Very tight kernel - less bleed from distant obstacles
        self.sigma_f = 1.0

        # HOCBF parameters (from paper, Section II-C) - RELAXED FOR FEASIBILITY
        self.lambda_0 = 0.25  # Reduced from 1.0 for less aggressive constraints
        self.lambda_1 = 0.25  # Reduced from 1.0 for less aggressive constraints
        self.c_q = 0.08  # Very small confidence for feasibility (was 0.3)

        # State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 1.0

        # Reference command - will be updated by tangent controller
        self.u_ref = [1.0, 0.0]  # [v_ref, ω_ref]
        self.u_prev = [1.0, 0.0]
        self.v_prev = 1.0  # Track previous velocity for acceleration control

        # Goal for tangent controller
        self.goal_x = 10.0  # Target x position (meters ahead)
        self.goal_y = 0.0  # Target y position (stay centered)

        # Initialize EKFs
        self.safety_ekf = SafetyULM_EKF(Ts=self.dt, m_inputs=2)
        self.position_ekf = PositionULM_EKF(Ts=self.dt, m_inputs=2)

        # CBF object
        self.cbf = ModelFreeCBF(
            dt=self.dt,
            u_min=[self.v_min, self.omega_min],
            u_max=[self.v_max, self.omega_max],
            r_max=self.r_max,
            r_min_obstacle=self.r_max,  # Use same value - obstacles within r_max affect barrier
            length_scale=self.length_scale,
            sigma_f=self.sigma_f,
            lambda_0=self.lambda_0,
            lambda_1=self.lambda_1,
            c_q=self.c_q
        )

        # ROS2 subscriptions and publishers
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.cmd_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        self.get_logger().info('Model-Free CBF Node initialized')

    def tangent_controller(self):
        """
        Gap-following controller: finds largest gap in LiDAR and steers toward it.
        Works in robot frame (no global coordinates needed).
        Returns: [v_ref, omega_ref]
        """
        # Default: drive straight forward
        v_ref = 1.0
        omega_ref = 0.0

        if not hasattr(self, 'last_ranges'):
            return [v_ref, omega_ref]

        ranges = self.last_ranges
        angles = self.last_angles

        # Only consider front sector (-90° to +90°)
        front_mask = cp.abs(angles) < cp.pi/2
        front_ranges = ranges[front_mask]
        front_angles = angles[front_mask]

        if len(front_ranges) == 0:
            return [v_ref, omega_ref]

        # Find gaps (continuous sectors with range > threshold)
        gap_threshold = 0.5  # Minimum distance to be considered "free"
        is_free = front_ranges > gap_threshold

        # Find largest gap
        max_gap_size = 0
        max_gap_center_angle = 0.0
        current_gap_size = 0
        current_gap_start_idx = 0

        for i in range(len(is_free)):
            if is_free[i]:
                if current_gap_size == 0:
                    current_gap_start_idx = i
                current_gap_size += 1
            else:
                if current_gap_size > max_gap_size:
                    max_gap_size = current_gap_size
                    # Gap center angle
                    gap_center_idx = current_gap_start_idx + current_gap_size // 2
                    max_gap_center_angle = float(front_angles[gap_center_idx])
                current_gap_size = 0

        # Check last gap
        if current_gap_size > max_gap_size:
            max_gap_size = current_gap_size
            gap_center_idx = current_gap_start_idx + current_gap_size // 2
            max_gap_center_angle = float(front_angles[gap_center_idx])

        # If no gap found, find direction with maximum range
        if max_gap_size == 0:
            max_range_idx = int(cp.argmax(front_ranges))
            max_gap_center_angle = float(front_angles[max_range_idx])

        # Steer toward gap center with proportional control
        # Prefer forward-facing gaps (weight by cos)
        K_p = 2.0
        omega_ref = K_p * max_gap_center_angle
        omega_ref = max(-0.5, min(0.5, omega_ref))  # Clip using Python built-ins

        return [v_ref, omega_ref]

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

        # Store for tangent controller
        self.last_ranges = ranges
        self.last_angles = angles

        # Update CBF with obstacle points
        self.cbf.set_obstacles(ranges, angles)

        # TANGENT CONTROLLER: Update reference command to steer around obstacles
        self.u_ref = self.tangent_controller()

        # Debug: log obstacle count
        n_obstacles = self.cbf.N
        valid_ranges = ranges[(ranges > 0.1) & (ranges < self.r_max)]
        if len(valid_ranges) > 0:
            min_range = float(cp.min(valid_ranges))
            self.get_logger().info(f'LiDAR: {n_obstacles} obstacles (<{self.r_max}m), closest scan at {min_range:.2f}m')
        else:
            self.get_logger().info(f'LiDAR: No valid scans')

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
        sigma_pdot = 0.01  # Position velocity uncertainty (tune this)
        epsilon = 1e-6
        R_qdot = grad_norm**2 * sigma_pdot**2 + sigma_gp_sq / (self.length_scale**2 * (grad_norm**2 + epsilon))

        # Step 4: Update safety EKF with measurements
        # Add sanity checks to prevent bad measurements from causing divergence
        if not cp.isnan(q_meas) and not cp.isinf(q_meas):
            self.safety_ekf.update_q(float(q_meas), R_q=max(float(sigma_gp_sq), 1e-4))
        else:
            self.get_logger().warn(f'Invalid q_meas={q_meas}, skipping update')

        if not cp.isnan(qdot_meas) and not cp.isinf(qdot_meas) and abs(qdot_meas) < 10.0:
            self.safety_ekf.update_qdot(qdot_meas, R_qdot=max(float(R_qdot), 1e-4))
        else:
            self.get_logger().warn(f'Invalid qdot_meas={qdot_meas}, skipping update')

        # Check if EKF has diverged and reset if needed
        ekf_just_reset = False
        if self.safety_ekf.reset_if_diverged():
            ekf_just_reset = True
            self.get_logger().warn(
                f'EKF diverged! Resetting to initial conditions. '
                f'Recent: q={q_meas:.3f}, qdot={qdot_meas:.3f}, n_obs={n_obstacles}'
            )

        # Step 5: Get estimates for control (with B_q clamped to valid range)
        q_hat, qdot_hat, F_q_hat, B_q_hat, P_safety = self.safety_ekf.get_estimates()

        # If EKF just reset, be extra conservative - don't bypass CBF
        if ekf_just_reset:
            q_hat = min(q_hat, 0.5)  # Force conservative estimate

        # SANITY CHECK: Only activate CBF if actually in danger
        # If barrier is high (q > 0.95) and no close obstacles, bypass CBF
        if q_hat > 0.95 and n_obstacles == 0:
            u_safe = self.u_ref
        else:
            # Step 6: Compute safe control (B_q is already clamped in get_estimates)
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
                # Emergency stop
                u_safe = [0.0, 0.0]

        # Step 7: Send command
        self.send_command(u_safe[0], u_safe[1])
        self.u_prev = u_safe

        total_time = time.time() - start_time

        # Determine what action CBF took
        dv = u_safe[0] - self.u_ref[0]
        dw = u_safe[1] - self.u_ref[1]
        action = "SAFE"
        if abs(dv) > 0.1 or abs(dw) > 0.1:
            if abs(dw) > abs(dv) * 0.5:  # Steering dominates
                action = "STEER"
            else:
                action = "BRAKE"

        # Calculate steering angle for logging
        L = 0.33
        if abs(u_safe[0]) > 0.1:
            steer_angle = float(cp.arctan(L * u_safe[1] / u_safe[0]))
        else:
            steer_angle = float(u_safe[1]) * 0.33
        steer_angle = max(-0.4, min(0.4, steer_angle))  # Clip using Python min/max

        self.get_logger().info(
            f'[{action}] t={total_time:.3f}s | q={q_hat:.3f} | B_q=[{B_q_hat[0]:.2f},{B_q_hat[1]:.2f}] | '
            f'v: {self.u_ref[0]:.2f}→{u_safe[0]:.2f} | ω: {self.u_ref[1]:.2f}→{u_safe[1]:.2f} | '
            f'δ: {steer_angle:.3f}rad'
        )

    def send_command(self, v, omega):
        """
        Publish Ackermann drive command.
        Converts angular velocity (omega) to steering angle.
        """
        msg = AckermannDriveStamped()
        msg.drive.speed = float(v)

        # CRITICAL: F1Tenth VESC needs acceleration field for braking!
        # If commanding lower speed than previous, set negative acceleration
        if v < self.v_prev - 0.1:
            # Braking - need aggressive deceleration
            msg.drive.acceleration = -5.0
            self.get_logger().debug(f'BRAKE: {self.v_prev:.2f}→{v:.2f} m/s, accel=-5.0')
        else:
            # Normal driving - moderate acceleration
            msg.drive.acceleration = 3.0

        # Store previous velocity for next iteration
        self.v_prev = v

        # Convert angular velocity to steering angle using Ackermann geometry
        # steering_angle = arctan(L * omega / v)
        # where L is wheelbase (F1Tenth ~0.33m)
        L = 0.33  # wheelbase in meters
        if abs(v) > 0.1:  # Avoid division by zero
            steering_angle = float(cp.arctan(L * omega / v))
        else:
            # At very low speeds, use direct proportional mapping
            steering_angle = float(omega) * 0.33  # Scale omega to reasonable steering

        # Clip to reasonable steering limits (F1Tenth: ±0.4 radians ≈ ±23°)
        steering_angle = max(-0.4, min(0.4, steering_angle))  # Use Python min/max

        msg.drive.steering_angle = steering_angle
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
