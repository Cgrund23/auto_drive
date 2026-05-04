#!/usr/bin/env python3
"""
Fast simulation for debugging Model-Free CBF
Runs the exact same CBF code but in a simulated environment
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import sys
sys.path.append('auto_drive')
# from CBF_refactored import ModelFreeCBF
from cbf_Node_refactored import SafetyULM_EKF, PositionULM_EKF


class F1TenthSim:
    """Simple kinematic bicycle model"""
    def __init__(self, x=0.0, y=0.0, theta=0.0, L=0.33):
        self.x = x
        self.y = y
        self.theta = theta
        self.L = L  # Wheelbase
        self.v = 0.0

    def step(self, v_cmd, delta_cmd, dt):
        """
        Update state with bicycle kinematics
        v_cmd: velocity (m/s)
        delta_cmd: steering angle (rad)
        """
        self.v = v_cmd

        # Bicycle model
        self.x += v_cmd * np.cos(self.theta) * dt
        self.y += v_cmd * np.sin(self.theta) * dt
        self.theta += (v_cmd / self.L) * np.tan(delta_cmd) * dt

        # Normalize theta
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))

    def get_state(self):
        return self.x, self.y, self.theta, self.v


class ObstacleField:
    """Simple obstacle environment"""
    def __init__(self):
        # Define static obstacles [x, y, radius]
        # Single obstacle in center of path
        self.obstacles = [
            [1.5, 0.0, 0.3],   # Center obstacle at x=1.5m
        ]

    def get_lidar_scan(self, robot_x, robot_y, robot_theta,
                       n_rays=360, max_range=5.0):
        """
        Simulate LiDAR scan from robot pose
        Returns: ranges (array), angles (array)
        """
        angles = np.linspace(-np.pi, np.pi, n_rays)
        ranges = np.ones(n_rays) * max_range

        for i, angle in enumerate(angles):
            # Ray in world frame
            ray_angle = robot_theta + angle

            # Check intersection with each obstacle
            for obs_x, obs_y, obs_r in self.obstacles:
                # Vector from robot to obstacle center
                dx = obs_x - robot_x
                dy = obs_y - robot_y

                # Distance to obstacle center
                dist_center = np.sqrt(dx**2 + dy**2)

                # Angle to obstacle center
                angle_to_obs = np.arctan2(dy, dx) - robot_theta
                angle_to_obs = np.arctan2(np.sin(angle_to_obs), np.cos(angle_to_obs))

                # Check if ray hits obstacle (simple circle collision)
                angle_diff = abs(angle - angle_to_obs)
                if angle_diff > np.pi:
                    angle_diff = 2*np.pi - angle_diff

                if angle_diff < np.arcsin(obs_r / max(dist_center, obs_r)):
                    # Hit! Compute distance
                    dist_surface = dist_center - obs_r
                    ranges[i] = min(ranges[i], max(0.1, dist_surface))

        return ranges, angles


class CBFSimulator:
    def __init__(self):
        # Simulation parameters
        self.dt = 0.05  # 20 Hz
        self.robot = F1TenthSim(x=0.0, y=0.0, theta=0.0)
        self.obstacles = ObstacleField()

        # CBF parameters (same as hardware)
        self.v_max = 2.0
        self.v_min = 0.0
        self.omega_max = 10.0
        self.omega_min = -10.0
        self.r_max = 5.0
        self.r_min_obstacle = 0.25
        self.length_scale = 0.5
        self.sigma_f = 1.0
        self.lambda_0 = 0.5
        self.lambda_1 = 0.5
        self.c_q = 0.05

        # Initialize EKFs
        self.safety_ekf = SafetyULM_EKF(Ts=self.dt, m_inputs=2)
        self.position_ekf = PositionULM_EKF(Ts=self.dt, m_inputs=2)

        # CBF object
        self.cbf = ModelFreeCBF(
            dt=self.dt,
            u_min=[self.v_min, self.omega_min],
            u_max=[self.v_max, self.omega_max],
            r_max=self.r_max,
            r_min_obstacle=self.r_min_obstacle,
            length_scale=self.length_scale,
            sigma_f=self.sigma_f,
            lambda_0=self.lambda_0,
            lambda_1=self.lambda_1,
            c_q=self.c_q
        )

        # Reference command
        self.u_ref = [1.0, 0.0]
        self.u_prev = [0.0, 0.0]

        # Data logging
        self.history = {
            'x': [], 'y': [], 'theta': [], 'v': [],
            'q': [], 'B_q_v': [], 'B_q_omega': [],
            'u_v': [], 'u_omega': [],
            'n_obstacles': []
        }

    def step(self):
        """Run one simulation step"""
        # Get robot state
        x, y, theta, v = self.robot.get_state()

        # Get LiDAR scan
        ranges, angles = self.obstacles.get_lidar_scan(x, y, theta)

        # Convert to CuPy
        ranges_cp = cp.array(ranges, dtype=cp.float32)
        angles_cp = cp.array(angles, dtype=cp.float32)

        # Update CBF with obstacles
        self.cbf.set_obstacles(ranges_cp, angles_cp)
        n_obstacles = self.cbf.N

        # Predict EKFs
        self.safety_ekf.predict(self.u_prev)
        self.position_ekf.predict(self.u_prev)

        # Update position EKF
        self.position_ekf.update(cp.array([x, y]))

        # Get GP measurements
        q_meas, sigma_gp_sq = self.cbf.get_barrier_and_variance([0.0, 0.0])
        grad_h = self.cbf.get_gradient([0.0, 0.0])

        # Get velocity estimate
        p_est, F_p, B_p = self.position_ekf.get_estimates()
        p_dot = F_p + B_p @ cp.array(self.u_prev)

        # Compute q̇
        qdot_meas = float(grad_h @ p_dot)

        # Compute measurement noise
        grad_norm = float(cp.linalg.norm(grad_h))
        sigma_pdot = 0.1
        epsilon = 1e-6
        R_qdot = grad_norm**2 * sigma_pdot**2 + sigma_gp_sq / (self.length_scale**2 * (grad_norm**2 + epsilon))

        # Update safety EKF
        self.safety_ekf.update_q(float(q_meas), R_q=float(sigma_gp_sq))
        self.safety_ekf.update_qdot(qdot_meas, R_qdot=float(R_qdot))

        # Check for divergence
        if self.safety_ekf.reset_if_diverged():
            print(f"[t={len(self.history['x'])*self.dt:.2f}s] EKF RESET!")

        # Get estimates
        q_hat, qdot_hat, F_q_hat, B_q_hat, P_safety = self.safety_ekf.get_estimates()

        # Compute safe control
        try:
            if q_hat > 0.95 and n_obstacles == 0:
                u_safe = self.u_ref
                status = "SAFE"
            else:
                u_safe = self.cbf.compute_safe_control(
                    u_ref=self.u_ref,
                    q_hat=q_hat,
                    qdot_hat=qdot_hat,
                    F_q_hat=F_q_hat,
                    B_q_hat=B_q_hat,
                    P=P_safety
                )
                dv = abs(u_safe[0] - self.u_ref[0])
                dw = abs(u_safe[1] - self.u_ref[1])
                if dw > dv * 0.5:
                    status = "STEER"
                else:
                    status = "BRAKE"
        except Exception as e:
            print(f"CBF QP failed: {e}")
            u_safe = [0.0, 0.0]
            status = "STOP"

        # Apply control (note: omega is steering angle in robot, delta for bicycle model)
        v_cmd = u_safe[0]
        delta_cmd = np.arctan(self.robot.L * u_safe[1] / max(v_cmd, 0.1))  # Convert omega to steering
        delta_cmd = np.clip(delta_cmd, -0.5, 0.5)  # Limit steering angle

        self.robot.step(v_cmd, delta_cmd, self.dt)
        self.u_prev = u_safe

        # Log data
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

        # Print status
        if len(self.history['x']) % 10 == 0:
            print(f"[{status}] t={len(self.history['x'])*self.dt:.2f}s | "
                  f"x={x:.2f}m y={y:.2f}m | q={q_hat:.3f} | "
                  f"B_q=[{B_q_hat[0]:.2f},{B_q_hat[1]:.2f}] | "
                  f"v={v_cmd:.2f} ω={u_safe[1]:.2f}")

        # Check collision
        for obs_x, obs_y, obs_r in self.obstacles.obstacles:
            dist = np.sqrt((x - obs_x)**2 + (y - obs_y)**2)
            if dist < obs_r + 0.15:  # Robot radius ~0.15m
                print(f"\n*** COLLISION at t={len(self.history['x'])*self.dt:.2f}s ***")
                return False

        # Check success
        if x > 3.0:
            print(f"\n*** SUCCESS! Reached goal at t={len(self.history['x'])*self.dt:.2f}s ***")
            return False

        return True

    def run(self, max_steps=500):
        """Run simulation"""
        print("Starting CBF simulation...")
        print(f"Goal: Drive from (0,0) to (3,0) avoiding obstacles")
        print(f"Parameters: λ0={self.lambda_0}, λ1={self.lambda_1}, cq={self.c_q}")
        print(f"            r_min={self.r_min_obstacle}m, length_scale={self.length_scale}\n")

        for i in range(max_steps):
            if not self.step():
                break

        self.plot_results()

    def plot_results(self):
        """Plot simulation results"""
        fig, axes = plt.subplots(2, 3, figsize=(15, 8))
        t = np.arange(len(self.history['x'])) * self.dt

        # Trajectory
        ax = axes[0, 0]
        ax.plot(self.history['x'], self.history['y'], 'b-', linewidth=2, label='Robot path')
        for obs_x, obs_y, obs_r in self.obstacles.obstacles:
            circle = plt.Circle((obs_x, obs_y), obs_r, color='r', alpha=0.5)
            ax.add_patch(circle)
        ax.plot(0, 0, 'go', markersize=10, label='Start')
        ax.plot(3, 0, 'r*', markersize=15, label='Goal')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title('Trajectory')
        ax.legend()
        ax.grid(True)
        ax.axis('equal')

        # Barrier value
        ax = axes[0, 1]
        ax.plot(t, self.history['q'], 'b-', linewidth=2)
        ax.axhline(y=0, color='r', linestyle='--', label='Unsafe (q<0)')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Barrier q')
        ax.set_title('Safety Barrier Value')
        ax.legend()
        ax.grid(True)

        # B_q estimates
        ax = axes[0, 2]
        ax.plot(t, self.history['B_q_v'], 'b-', linewidth=2, label='B_q,v')
        ax.plot(t, self.history['B_q_omega'], 'r-', linewidth=2, label='B_q,ω')
        ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('B_q')
        ax.set_title('ULM Parameter Estimates (EKF)')
        ax.legend()
        ax.grid(True)

        # Control commands
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

        # Obstacle count
        ax = axes[1, 2]
        ax.plot(t, self.history['n_obstacles'], 'g-', linewidth=2)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Count')
        ax.set_title('Number of Close Obstacles')
        ax.grid(True)

        plt.tight_layout()
        plt.savefig('cbf_simulation_results.png', dpi=150)
        print(f"\nResults saved to cbf_simulation_results.png")
        plt.show()


if __name__ == '__main__':
    sim = CBFSimulator()
    sim.run(max_steps=500)
