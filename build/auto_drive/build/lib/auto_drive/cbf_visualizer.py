#!/usr/bin/env python3
"""
Real-time visualization of CBF barrier field and LiDAR data
Shows what the robot "sees" in terms of obstacles and safety barriers
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import cupy as cp


class CBFVisualizer(Node):
    def __init__(self):
        super().__init__('cbf_visualizer')

        # Subscribe to LiDAR and odometry
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Data storage
        self.ranges = None
        self.angles = None
        self.obstacle_points = None
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # GP parameters (match hardware)
        self.r_max = 5.0
        self.length_scale = 0.25
        self.sigma_f = 1.0

        # Setup matplotlib figure
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(14, 6))

        self.get_logger().info('CBF Visualizer started. Close window to exit.')

        # Start animation
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=100, blit=False)
        plt.show()

    def odom_callback(self, msg):
        """Store robot position"""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        self.theta = np.arctan2(siny_cosp, cosy_cosp)

    def lidar_callback(self, msg):
        """Process LiDAR data"""
        ranges = np.array(msg.ranges, dtype=np.float32)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges), dtype=np.float32)

        # Downsample for speed
        ranges = ranges[::10]
        angles = angles[::10]

        self.ranges = ranges
        self.angles = angles

        # Extract obstacle points
        mask = (ranges < self.r_max) & (ranges > 0.1)
        filtered_ranges = ranges[mask]
        filtered_angles = angles[mask]

        x = filtered_ranges * np.cos(filtered_angles)
        y = filtered_ranges * np.sin(filtered_angles)

        # Downsample
        x = x[::5]
        y = y[::5]

        self.obstacle_points = np.column_stack((x, -y))

    def rbf_kernel(self, X1, X2):
        """RBF kernel for GP"""
        sqdist = np.sum(X1**2, axis=1, keepdims=True) + np.sum(X2**2, axis=1) - 2 * (X1 @ X2.T)
        return self.sigma_f**2 * np.exp(-0.5 * sqdist / self.length_scale**2)

    def compute_barrier_field(self, grid_x, grid_y):
        """Compute barrier function on a grid"""
        if self.obstacle_points is None or len(self.obstacle_points) == 0:
            return np.ones_like(grid_x)

        N = len(self.obstacle_points)
        Y = -np.ones((N, 1))

        # Compute kernel and inverse
        K = self.rbf_kernel(self.obstacle_points, self.obstacle_points)
        K_inv = np.linalg.inv(K + 1e-6 * np.eye(N))
        alpha = K_inv @ (Y - 1.0)

        # Evaluate on grid
        barrier_values = np.zeros_like(grid_x)
        for i in range(grid_x.shape[0]):
            for j in range(grid_x.shape[1]):
                p = np.array([[grid_x[i, j], grid_y[i, j]]])
                k_star = self.rbf_kernel(p, self.obstacle_points)
                h = 1.0 + float(k_star @ alpha)
                barrier_values[i, j] = h

        return barrier_values

    def update_plot(self, frame):
        """Update visualization"""
        if self.ranges is None:
            return

        # Clear axes
        self.ax1.clear()
        self.ax2.clear()

        # === LEFT PLOT: LiDAR Points ===
        self.ax1.set_title('LiDAR Scan (Robot Frame)', fontsize=14, fontweight='bold')
        self.ax1.set_xlabel('X (forward, m)')
        self.ax1.set_ylabel('Y (left, m)')
        self.ax1.set_xlim(-3, 3)
        self.ax1.set_ylim(-3, 3)
        self.ax1.grid(True, alpha=0.3)
        self.ax1.set_aspect('equal')

        # Plot all LiDAR points
        if self.ranges is not None:
            x_all = self.ranges * np.cos(self.angles)
            y_all = -self.ranges * np.sin(self.angles)
            valid = (self.ranges > 0.1) & (self.ranges < self.r_max)
            self.ax1.scatter(x_all[valid], y_all[valid], c='lightgray', s=5, alpha=0.5, label='All LiDAR')

        # Plot obstacle points used by GP
        if self.obstacle_points is not None and len(self.obstacle_points) > 0:
            self.ax1.scatter(self.obstacle_points[:, 0], self.obstacle_points[:, 1],
                           c='red', s=50, marker='x', linewidths=2, label=f'{len(self.obstacle_points)} GP points', zorder=10)

        # Draw robot
        robot_circle = plt.Circle((0, 0), 0.15, color='blue', fill=True, alpha=0.7, label='Robot')
        self.ax1.add_patch(robot_circle)
        self.ax1.arrow(0, 0, 0.3, 0, head_width=0.1, head_length=0.1, fc='blue', ec='blue', zorder=10)

        self.ax1.legend(loc='upper right', fontsize=9)

        # === RIGHT PLOT: Barrier Field ===
        self.ax2.set_title('GP Barrier Field h(x)', fontsize=14, fontweight='bold')
        self.ax2.set_xlabel('X (forward, m)')
        self.ax2.set_ylabel('Y (left, m)')
        self.ax2.set_xlim(-2, 2)
        self.ax2.set_ylim(-2, 2)
        self.ax2.set_aspect('equal')

        # Compute barrier on grid
        grid_range = 2.0
        grid_res = 0.05
        x_grid = np.arange(-grid_range, grid_range, grid_res)
        y_grid = np.arange(-grid_range, grid_range, grid_res)
        X_grid, Y_grid = np.meshgrid(x_grid, y_grid)

        Z_grid = self.compute_barrier_field(X_grid, Y_grid)

        # Plot barrier field
        contour = self.ax2.contourf(X_grid, Y_grid, Z_grid, levels=20, cmap='RdYlGn', alpha=0.8, vmin=-1, vmax=1)

        # Add colorbar if it doesn't exist
        if not hasattr(self, 'cbar'):
            self.cbar = plt.colorbar(contour, ax=self.ax2, label='Barrier h(x)')

        # Draw h=0 contour (unsafe boundary)
        self.ax2.contour(X_grid, Y_grid, Z_grid, levels=[0], colors='red', linewidths=3, linestyles='--')

        # Plot obstacle points
        if self.obstacle_points is not None and len(self.obstacle_points) > 0:
            self.ax2.scatter(self.obstacle_points[:, 0], self.obstacle_points[:, 1],
                           c='black', s=30, marker='x', linewidths=2, zorder=10)

        # Draw robot
        robot_circle2 = plt.Circle((0, 0), 0.15, color='blue', fill=True, alpha=0.7)
        self.ax2.add_patch(robot_circle2)
        self.ax2.arrow(0, 0, 0.3, 0, head_width=0.1, head_length=0.1, fc='blue', ec='blue', zorder=10)

        # Add text with info
        info_text = f"Length scale: {self.length_scale:.2f}m\n"
        info_text += f"Obstacles: {len(self.obstacle_points) if self.obstacle_points is not None else 0}\n"
        info_text += f"Position: ({self.x:.2f}, {self.y:.2f})"
        self.ax2.text(0.02, 0.98, info_text, transform=self.ax2.transAxes,
                     verticalalignment='top', fontsize=9,
                     bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

        self.ax2.grid(True, alpha=0.3)

        plt.tight_layout()


def main(args=None):
    rclpy.init(args=args)
    visualizer = CBFVisualizer()

    try:
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
