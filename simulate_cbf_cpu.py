#!/usr/bin/env python3
"""
Fast CPU-only simulation for debugging Model-Free CBF
Uses NumPy instead of CuPy for debugging on any machine
"""
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend
import matplotlib.pyplot as plt
import sys


class ModelFreeCBF_CPU:
    """CPU version of ModelFreeCBF using NumPy"""
    def __init__(self, dt, u_min, u_max, r_max, r_min_obstacle, length_scale, sigma_f, lambda_0, lambda_1, c_q):
        self.dt = dt
        self.u_min = np.array(u_min)
        self.u_max = np.array(u_max)
        self.r_max = r_max
        self.r_min_obstacle = r_min_obstacle
        self.length_scale = length_scale
        self.sigma_f = sigma_f
        self.lambda_0 = lambda_0
        self.lambda_1 = lambda_1
        self.c_q = c_q
        self.obstacle_points = None
        self.N = 0

    def set_obstacles(self, ranges, angles):
        ranges = np.asarray(ranges)
        angles = np.asarray(angles)
        mask = (ranges < self.r_min_obstacle) & (ranges > 0.1)
        filtered_ranges = ranges[mask]
        filtered_angles = angles[mask]
        x = filtered_ranges * np.cos(filtered_angles)
        y = filtered_ranges * np.sin(filtered_angles)
        x = x[::5]
        y = y[::5]
        self.obstacle_points = np.column_stack((x, -y))
        self.N = len(self.obstacle_points)

    def rbf_kernel(self, X1, X2):
        sqdist = np.sum(X1**2, axis=1, keepdims=True) + np.sum(X2**2, axis=1) - 2 * (X1 @ X2.T)
        return self.sigma_f**2 * np.exp(-0.5 * sqdist / self.length_scale**2)

    def get_barrier_and_variance(self, p):
        if self.N == 0:
            return 1.0, 0.0
        p = np.array(p).reshape(1, 2)
        Y = -np.ones((self.N, 1))
        K = self.rbf_kernel(self.obstacle_points, self.obstacle_points)
        K_inv = np.linalg.inv(K + 1e-6 * np.eye(self.N))
        k_star = self.rbf_kernel(p, self.obstacle_points)
        h = 1.0 + float(k_star @ K_inv @ (Y - 1.0))
        k_ss = self.rbf_kernel(p, p)[0, 0]
        sigma_sq = float(k_ss - k_star @ K_inv @ k_star.T)
        return h, sigma_sq

    def get_gradient(self, p):
        if self.N == 0:
            return np.zeros(2)
        p = np.array(p).reshape(1, 2)
        Y = -np.ones((self.N, 1))
        K = self.rbf_kernel(self.obstacle_points, self.obstacle_points)
        K_inv = np.linalg.inv(K + 1e-6 * np.eye(self.N))
        alpha = K_inv @ (Y - 1.0)
        k_star = self.rbf_kernel(p, self.obstacle_points)
        diff = self.obstacle_points - p
        grad_h = (k_star.T * alpha).T @ diff / self.length_scale**2
        return grad_h.flatten()

    def compute_safety_margin(self, P, u_max):
        u_wc = np.asarray(self.u_max)
        P = np.asarray(P)
        ell = np.array([
            self.lambda_0 * self.lambda_1,
            self.lambda_0 + self.lambda_1,
            1.0,
            float(u_wc[0]),
            float(u_wc[1])
        ])
        sigma_sq = float(ell @ P @ ell)
        sigma_bar = float(np.sqrt(np.maximum(sigma_sq, 0.0)))
        sigma = float(self.c_q * sigma_bar)
        sigma_max = 0.05
        return min(sigma, sigma_max)

    def compute_safe_control(self, u_ref, q_hat, qdot_hat, F_q_hat, B_q_hat, P):
        """Simple analytical QP solver (no dependencies needed!)"""
        u_ref = np.asarray(u_ref)
        B_q_hat = np.asarray(B_q_hat)

        sigma_k = self.compute_safety_margin(P, self.u_max)
        r_k = -F_q_hat - (self.lambda_0 + self.lambda_1) * qdot_hat - \
              self.lambda_0 * self.lambda_1 * q_hat + sigma_k

        u_ref_np = np.array(u_ref, dtype=np.float64)
        B_q_np = np.array(B_q_hat, dtype=np.float64)
        u_max_np = np.array(self.u_max, dtype=np.float64)
        u_min_np = np.array(self.u_min, dtype=np.float64)

        # Start with unconstrained solution (just clip to bounds)
        u_safe = np.clip(u_ref_np, u_min_np, u_max_np)

        # Check CBF constraint: B_q @ u >= r_k
        cbf_value = B_q_np @ u_safe

        if cbf_value >= r_k:
            # Already safe!
            return [float(u_safe[0]), float(u_safe[1])]

        # Need to project onto CBF constraint
        # Find closest point on line B_q @ u = r_k to u_ref

        # Weighted distance for QP cost
        w_v = 10.0
        w_omega = 0.1
        W = np.array([w_v, w_omega])

        # Project: u = u_ref + lambda * (B_q / ||B_q||_W^2)
        # Such that B_q @ u = r_k
        B_q_norm_sq = np.sum(B_q_np**2 / W)
        if B_q_norm_sq < 1e-6:
            # Degenerate case
            return [float(u_min_np[0]), 0.0]

        # Lambda for projection
        deficit = r_k - (B_q_np @ u_ref_np)
        lam = deficit / B_q_norm_sq

        # Project
        u_proj = u_ref_np + lam * (B_q_np / W)

        # Clip to bounds
        u_safe = np.clip(u_proj, u_min_np, u_max_np)

        # Final check - if still infeasible after clipping, find feasible point
        if B_q_np @ u_safe < r_k:
            # Find feasible point on boundary
            # Try corners and edges
            candidates = [
                np.array([u_max_np[0], u_max_np[1]]),
                np.array([u_max_np[0], u_min_np[1]]),
                np.array([u_min_np[0], u_max_np[1]]),
                np.array([u_min_np[0], u_min_np[1]]),
            ]

            best_u = candidates[0]
            best_cost = float('inf')

            for cand in candidates:
                if B_q_np @ cand >= r_k:
                    cost = w_v * (cand[0] - u_ref_np[0])**2 + w_omega * (cand[1] - u_ref_np[1])**2
                    if cost < best_cost:
                        best_cost = cost
                        best_u = cand

            if best_cost == float('inf'):
                # Truly infeasible
                print(f"QP INFEASIBLE: r_k={r_k:.3f}, B_q={B_q_np}, max={B_q_np@u_max_np:.3f}")
                return [float(u_min_np[0]), 0.0]

            u_safe = best_u

        return [float(u_safe[0]), float(u_safe[1])]


class SafetyULM_EKF_CPU:
    def __init__(self, Ts, m_inputs=2):
        self.Ts = Ts
        self.m = m_inputs
        state_dim = 3 + m_inputs
        self.x = np.zeros(state_dim)
        self.x[2] = 0.0
        self.x[3] = 0.5
        if m_inputs > 1:
            self.x[4] = 0.1
        P_diag = [0.01, 0.01, 0.1] + [0.1] * m_inputs
        self.P = np.diag(np.array(P_diag))
        Q_diag = [1e-6, 1e-5, 1e-2] + [1e-2] * m_inputs
        self.Q = np.diag(np.array(Q_diag))
        self.R_q = 1e-3
        self.R_qdot = 1e-2
        self.H_q = np.zeros((1, state_dim))
        self.H_q[0, 0] = 1.0
        self.H_qdot = np.zeros((1, state_dim))
        self.H_qdot[0, 1] = 1.0

    def predict(self, u):
        Ts = self.Ts
        u = np.array(u)
        q = self.x[0]
        qdot = self.x[1]
        F_q = self.x[2]
        B_q = self.x[3:3+self.m]
        qddot = F_q + np.dot(B_q, u)
        q_new = q + Ts * qdot + (Ts**2 / 2) * qddot
        qdot_new = qdot + Ts * qddot
        self.x[0] = q_new
        self.x[1] = qdot_new
        A = np.eye(3 + self.m)
        A[0, 1] = Ts
        A[0, 2] = Ts**2 / 2
        A[0, 3:3+self.m] = (Ts**2 / 2) * u
        A[1, 2] = Ts
        A[1, 3:3+self.m] = Ts * u
        self.P = A @ self.P @ A.T + self.Q

    def update_q(self, q_meas, R_q=None):
        if R_q is not None:
            self.R_q = R_q
        R = np.array([[self.R_q]])
        y = q_meas - self.H_q @ self.x
        S = self.H_q @ self.P @ self.H_q.T + R
        K = self.P @ self.H_q.T / S[0, 0]
        self.x = self.x + K.flatten() * y
        self.P = (np.eye(len(self.x)) - np.outer(K, self.H_q)) @ self.P

    def update_qdot(self, qdot_meas, R_qdot=None):
        if R_qdot is not None:
            self.R_qdot = R_qdot
        R = np.array([[self.R_qdot]])
        y = qdot_meas - self.H_qdot @ self.x
        S = self.H_qdot @ self.P @ self.H_qdot.T + R
        K = self.P @ self.H_qdot.T / S[0, 0]
        self.x = self.x + K.flatten() * y
        self.P = (np.eye(len(self.x)) - np.outer(K, self.H_qdot)) @ self.P

    def reset_if_diverged(self):
        B_q = self.x[3:3+self.m]
        if float(B_q[0]) < 0.05 or float(B_q[0]) > 5.0:
            self.x[2] = 0.0
            self.x[3] = 0.5
            if self.m > 1:
                self.x[4] = 0.1
            P_diag = [0.01, 0.01, 0.1] + [0.1] * self.m
            self.P = np.diag(np.array(P_diag))
            return True
        return False

    def get_estimates(self):
        B_q = self.x[3:3+self.m].copy()
        B_q[0] = float(np.clip(B_q[0], 0.05, 3.0))
        if self.m > 1:
            B_q[1] = float(np.clip(B_q[1], -1.0, 1.0))
        return (float(self.x[0]), float(self.x[1]), float(self.x[2]), B_q, self.P.copy())


class PositionULM_EKF_CPU:
    def __init__(self, Ts, m_inputs=2):
        self.Ts = Ts
        self.m = m_inputs
        state_dim = 2 + 2 + 2 * m_inputs
        self.x = np.zeros(state_dim)
        self.x[4] = 1.0
        self.x[6] = 0.0
        self.x[5] = 0.0
        self.x[7] = 0.1
        P_diag = [0.01, 0.01, 0.1, 0.1] + [0.1] * (2 * m_inputs)
        self.P = np.diag(np.array(P_diag))
        Q_diag = [1e-6, 1e-6, 1e-3, 1e-3] + [1e-3] * (2 * m_inputs)
        self.Q = np.diag(np.array(Q_diag))
        self.R = np.diag([1e-3, 1e-3])
        self.H = np.zeros((2, state_dim))
        self.H[0, 0] = 1.0
        self.H[1, 1] = 1.0

    def predict(self, u):
        Ts = self.Ts
        u = np.array(u)
        p = self.x[0:2]
        F_p = self.x[2:4]
        B_p = self.x[4:4+2*self.m].reshape(2, self.m)
        pdot = F_p + B_p @ u
        p_new = p + Ts * pdot
        self.x[0:2] = p_new
        A = np.eye(len(self.x))
        A[0, 2] = Ts
        A[0, 4:4+self.m] = Ts * u
        A[1, 3] = Ts
        A[1, 4+self.m:4+2*self.m] = Ts * u
        self.P = A @ self.P @ A.T + self.Q

    def update(self, p_meas):
        p_meas = np.array(p_meas).reshape(2, 1)
        y = p_meas - (self.H @ self.x).reshape(2, 1)
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + (K @ y).flatten()
        self.P = (np.eye(len(self.x)) - K @ self.H) @ self.P

    def get_estimates(self):
        return (self.x[0:2].copy(), self.x[2:4].copy(), self.x[4:4+2*self.m].reshape(2, self.m).copy())


class F1TenthSim:
    def __init__(self, x=0.0, y=0.0, theta=0.0, L=0.33):
        self.x = x
        self.y = y
        self.theta = theta
        self.L = L
        self.v = 0.0

    def step(self, v_cmd, delta_cmd, dt):
        self.v = v_cmd
        self.x += v_cmd * np.cos(self.theta) * dt
        self.y += v_cmd * np.sin(self.theta) * dt
        self.theta += (v_cmd / self.L) * np.tan(delta_cmd) * dt
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))

    def get_state(self):
        return self.x, self.y, self.theta, self.v


class ObstacleField:
    def __init__(self, track_type='oval'):
        """
        Generate racetrack walls

        Args:
            track_type: 'oval' or 'random' obstacles
        """
        self.obstacles = []

        if track_type == 'oval':
            # Oval track parameters
            # Track center at origin, major axis along x, minor axis along y
            major_radius = 4.0  # Half-length along x
            minor_radius = 2.5  # Half-width along y
            track_width = 1.2   # Width of drivable track
            wall_thickness = 0.15  # Wall thickness for LiDAR detection

            # Generate walls as small circular obstacles along track perimeter
            n_points = 80  # Number of wall segments

            # Outer wall
            for i in range(n_points):
                theta = 2 * np.pi * i / n_points
                x = (major_radius + track_width/2) * np.cos(theta)
                y = (minor_radius + track_width/2) * np.sin(theta)
                self.obstacles.append([x, y, wall_thickness])

            # Inner wall
            for i in range(n_points):
                theta = 2 * np.pi * i / n_points
                x = (major_radius - track_width/2) * np.cos(theta)
                y = (minor_radius - track_width/2) * np.sin(theta)
                self.obstacles.append([x, y, wall_thickness])

            print(f"Generated oval racetrack:")
            print(f"  Major radius: {major_radius}m")
            print(f"  Minor radius: {minor_radius}m")
            print(f"  Track width: {track_width}m")
            print(f"  Wall segments: {2*n_points}")
        else:
            # Random obstacles (old behavior)
            n_obstacles = 5
            for i in range(n_obstacles):
                x = np.random.uniform(0.5, 3.5)
                y = np.random.uniform(-0.8, 0.8)
                r = np.random.uniform(0.2, 0.4)
                self.obstacles.append([x, y, r])
            print(f"Generated {n_obstacles} random obstacles")

    def get_lidar_scan(self, robot_x, robot_y, robot_theta, n_rays=360, max_range=5.0):
        angles = np.linspace(-np.pi, np.pi, n_rays)
        ranges = np.ones(n_rays) * max_range
        for i, angle in enumerate(angles):
            ray_angle = robot_theta + angle
            for obs_x, obs_y, obs_r in self.obstacles:
                dx = obs_x - robot_x
                dy = obs_y - robot_y
                dist_center = np.sqrt(dx**2 + dy**2)
                angle_to_obs = np.arctan2(dy, dx) - robot_theta
                angle_to_obs = np.arctan2(np.sin(angle_to_obs), np.cos(angle_to_obs))
                angle_diff = abs(angle - angle_to_obs)
                if angle_diff > np.pi:
                    angle_diff = 2*np.pi - angle_diff
                if angle_diff < np.arcsin(obs_r / max(dist_center, obs_r)):
                    dist_surface = dist_center - obs_r
                    ranges[i] = min(ranges[i], max(0.1, dist_surface))
        return ranges, angles


class CBFSimulator:
    def __init__(self, track_type='oval'):
        self.dt = 0.05
        # Start robot on the track (in the middle of the drivable surface)
        if track_type == 'oval':
            # Track: major_radius=4.0, minor_radius=2.5, track_width=1.2
            # Start on right side, facing tangent to track (90° = north)
            self.robot = F1TenthSim(x=4.0, y=0.0, theta=np.pi/2)
        else:
            self.robot = F1TenthSim(x=0.0, y=0.0, theta=0.0)
        self.obstacles = ObstacleField(track_type=track_type)
        self.v_max = 2.0
        self.v_min = 0.0
        self.omega_max = 10.0
        self.omega_min = -10.0
        self.r_max = 5.0
        self.r_min_obstacle = 0.25
        self.length_scale = 0.1
        self.sigma_f = 1.0
        self.lambda_0 = 0.5
        self.lambda_1 = 0.5
        self.c_q = 0.05
        self.safety_ekf = SafetyULM_EKF_CPU(Ts=self.dt, m_inputs=2)
        self.position_ekf = PositionULM_EKF_CPU(Ts=self.dt, m_inputs=2)
        self.cbf = ModelFreeCBF_CPU(
            dt=self.dt, u_min=[self.v_min, self.omega_min],
            u_max=[self.v_max, self.omega_max], r_max=self.r_max,
            r_min_obstacle=self.r_min_obstacle, length_scale=self.length_scale,
            sigma_f=self.sigma_f, lambda_0=self.lambda_0,
            lambda_1=self.lambda_1, c_q=self.c_q
        )
        self.u_ref = [1.0, 0.0]
        self.u_prev = [0.0, 0.0]
        self.history = {
            'x': [], 'y': [], 'theta': [], 'v': [],
            'q': [], 'B_q_v': [], 'B_q_omega': [],
            'u_v': [], 'u_omega': [], 'n_obstacles': []
        }

    def tangent_controller(self, x, y, theta, goal_x=3.0, goal_y=0.0):
        """
        Gap-following controller: finds largest gap in LiDAR and steers toward it.
        Works in robot frame (no global goal tracking).

        Note: Still takes x,y,theta for simulation purposes (get lidar scan),
        but controller logic is robot-frame only.

        Returns: [v_ref, omega_ref]
        """
        # Default: drive straight forward
        v_ref = 1.0
        omega_ref = 0.0

        # Get simulated LiDAR scan in robot frame
        ranges, angles = self.obstacles.get_lidar_scan(x, y, theta)

        # Only consider front sector (-90° to +90°)
        front_mask = np.abs(angles) < np.pi/2
        front_ranges = ranges[front_mask]
        front_angles = angles[front_mask]

        if len(front_ranges) == 0:
            return [v_ref, omega_ref]

        # Find gaps (continuous sectors with range > threshold)
        gap_threshold = 1.5  # Minimum distance to be considered "free"
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
            max_range_idx = int(np.argmax(front_ranges))
            max_gap_center_angle = float(front_angles[max_range_idx])

        # Steer toward gap center with proportional control
        K_p = 2.0
        omega_ref = float(K_p * max_gap_center_angle)
        omega_ref = np.clip(omega_ref, -1.0, 1.0)

        return [v_ref, omega_ref]

    def step(self):
        x, y, theta, v = self.robot.get_state()
        ranges, angles = self.obstacles.get_lidar_scan(x, y, theta)
        self.cbf.set_obstacles(ranges, angles)
        n_obstacles = self.cbf.N

        # TANGENT CONTROLLER: generates steering to go around obstacles
        self.u_ref = self.tangent_controller(x, y, theta)

        self.safety_ekf.predict(self.u_prev)
        self.position_ekf.predict(self.u_prev)
        self.position_ekf.update(np.array([x, y]))
        q_meas, sigma_gp_sq = self.cbf.get_barrier_and_variance([0.0, 0.0])
        grad_h = self.cbf.get_gradient([0.0, 0.0])
        p_est, F_p, B_p = self.position_ekf.get_estimates()
        p_dot = F_p + B_p @ np.array(self.u_prev)
        qdot_meas = float(grad_h @ p_dot)
        grad_norm = float(np.linalg.norm(grad_h))
        sigma_pdot = 0.1
        epsilon = 1e-6
        R_qdot = grad_norm**2 * sigma_pdot**2 + sigma_gp_sq / (self.length_scale**2 * (grad_norm**2 + epsilon))
        self.safety_ekf.update_q(float(q_meas), R_q=float(sigma_gp_sq))
        self.safety_ekf.update_qdot(qdot_meas, R_qdot=float(R_qdot))
        if self.safety_ekf.reset_if_diverged():
            print(f"[t={len(self.history['x'])*self.dt:.2f}s] EKF RESET!")
        q_hat, qdot_hat, F_q_hat, B_q_hat, P_safety = self.safety_ekf.get_estimates()
        try:
            if q_hat > 0.95 and n_obstacles == 0:
                u_safe = self.u_ref
                status = "NOMINAL"
            else:
                # CBF filters the nominal controller
                u_safe = self.cbf.compute_safe_control(
                    u_ref=self.u_ref, q_hat=q_hat, qdot_hat=qdot_hat,
                    F_q_hat=F_q_hat, B_q_hat=B_q_hat, P=P_safety
                )
                dv = abs(u_safe[0] - self.u_ref[0])
                dw = abs(u_safe[1] - self.u_ref[1])
                if dv < 0.1 and dw < 0.1:
                    status = "TANGENT"  # Following tangent controller
                elif dw > dv * 0.5:
                    status = "CBF_STEER"  # CBF modifying steering
                else:
                    status = "CBF_BRAKE"  # CBF braking
        except Exception as e:
            print(f"CBF QP failed: {e}")
            u_safe = [0.0, 0.0]
            status = "STOP"
        v_cmd = u_safe[0]
        delta_cmd = np.arctan(self.robot.L * u_safe[1] / max(v_cmd, 0.1))
        delta_cmd = np.clip(delta_cmd, -0.5, 0.5)
        self.robot.step(v_cmd, delta_cmd, self.dt)
        self.u_prev = u_safe
        self.history['x'].append(x)
        self.history['y'].append(y)
        self.history['theta'].append(theta)
        self.history['v'].append(v)
        self.history['q'].append(q_hat)
        self.history['B_q_v'].append(B_q_hat[0])
        self.history['B_q_omega'].append(B_q_hat[1])
        self.history['u_v'].append(u_safe[0])
        self.history['u_omega'].append(u_safe[1])
        self.history['n_obstacles'].append(n_obstacles)
        if len(self.history['x']) % 10 == 0:
            print(f"[{status}] t={len(self.history['x'])*self.dt:.2f}s | "
                  f"x={x:.2f}m y={y:.2f}m | q={q_hat:.3f} | "
                  f"B_q=[{B_q_hat[0]:.2f},{B_q_hat[1]:.2f}] | "
                  f"v={v_cmd:.2f} w={u_safe[1]:.2f}")
        # Check for collision with any obstacle/wall
        for obs_x, obs_y, obs_r in self.obstacles.obstacles:
            dist = np.sqrt((x - obs_x)**2 + (y - obs_y)**2)
            if dist < obs_r + 0.15:  # Robot radius ~0.15m
                print(f"\n*** COLLISION at t={len(self.history['x'])*self.dt:.2f}s ***")
                print(f"Hit wall at ({obs_x:.2f}, {obs_y:.2f})")
                return False

        # For oval track, no specific goal - just keep driving
        return True

    def run(self, max_steps=200):
        print("Starting CBF simulation...")
        print(f"Robot will drive around the track using gap-following controller")
        for i in range(max_steps):
            if not self.step():
                break
        print(f"\nFinal position: ({self.history['x'][-1]:.2f}, {self.history['y'][-1]:.2f})")
        print(f"Final B_q: [{self.history['B_q_v'][-1]:.2f}, {self.history['B_q_omega'][-1]:.2f}]")
        self.plot_results()

    def plot_results(self):
        fig, axes = plt.subplots(2, 3, figsize=(15, 8))
        t = np.arange(len(self.history['x'])) * self.dt
        ax = axes[0, 0]

        # Draw track walls/obstacles
        for obs_x, obs_y, obs_r in self.obstacles.obstacles:
            circle = plt.Circle((obs_x, obs_y), obs_r, color='gray', alpha=0.3)
            ax.add_patch(circle)

        # Plot robot trajectory
        ax.plot(self.history['x'], self.history['y'], 'b-', linewidth=2, label='Robot path')

        # Mark start position
        start_x = self.history['x'][0]
        start_y = self.history['y'][0]
        ax.plot(start_x, start_y, 'go', markersize=10, label='Start')

        # Mark end position
        end_x = self.history['x'][-1]
        end_y = self.history['y'][-1]
        ax.plot(end_x, end_y, 'ro', markersize=10, label='End')

        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title('Racetrack Trajectory')
        ax.legend()
        ax.grid(True)
        ax.axis('equal')
        ax = axes[0, 1]
        ax.plot(t, self.history['q'], 'b-', linewidth=2)
        ax.axhline(y=0, color='r', linestyle='--', label='Unsafe (q<0)')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Barrier q')
        ax.set_title('Safety Barrier Value')
        ax.legend()
        ax.grid(True)
        ax = axes[0, 2]
        ax.plot(t, self.history['B_q_v'], 'b-', linewidth=2, label='B_q,v')
        ax.plot(t, self.history['B_q_omega'], 'r-', linewidth=2, label='B_q,ω')
        ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('B_q')
        ax.set_title('ULM Parameter Estimates (EKF)')
        ax.legend()
        ax.grid(True)
        ax = axes[1, 0]
        ax.plot(t, self.history['u_v'], 'b-', linewidth=2, label='Velocity')
        ax.axhline(y=self.u_ref[0], color='b', linestyle='--', alpha=0.3, label='v_ref')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Velocity (m/s)')
        ax.set_title('Control: Velocity')
        ax.legend()
        ax.grid(True)
        ax = axes[1, 1]
        ax.plot(t, self.history['u_omega'], 'r-', linewidth=2, label='Steering rate')
        ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('ω (rad/s)')
        ax.set_title('Control: Steering Rate')
        ax.legend()
        ax.grid(True)
        ax = axes[1, 2]
        ax.plot(t, self.history['n_obstacles'], 'g-', linewidth=2)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Count')
        ax.set_title('Number of Close Obstacles')
        ax.grid(True)
        plt.tight_layout()
        plt.savefig('cbf_simulation_results.png', dpi=150)
        print(f"\nResults saved to cbf_simulation_results.png")


if __name__ == '__main__':
    # Create simulator with oval racetrack
    # track_type: 'oval' for racetrack, 'random' for random obstacles
    sim = CBFSimulator(track_type='oval')
    sim.run(max_steps=600)  # Run for ~30 seconds (about 2-3 laps)
