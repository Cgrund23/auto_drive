#!/usr/bin/env python3
"""
Offline visualization tool for CBF barrier fields
Useful for debugging and understanding how the GP barrier works
"""
import numpy as np
import matplotlib.pyplot as plt


def rbf_kernel(X1, X2, length_scale, sigma_f):
    """RBF kernel"""
    sqdist = np.sum(X1**2, axis=1, keepdims=True) + np.sum(X2**2, axis=1) - 2 * (X1 @ X2.T)
    return sigma_f**2 * np.exp(-0.5 * sqdist / length_scale**2)


def compute_barrier_field(obstacle_points, grid_x, grid_y, length_scale=0.25, sigma_f=1.0):
    """Compute GP barrier on a grid"""
    if len(obstacle_points) == 0:
        return np.ones_like(grid_x)

    N = len(obstacle_points)
    Y = -np.ones((N, 1))

    # Compute kernel and inverse
    K = rbf_kernel(obstacle_points, obstacle_points, length_scale, sigma_f)
    K_inv = np.linalg.inv(K + 1e-6 * np.eye(N))
    alpha = K_inv @ (Y - 1.0)

    # Evaluate on grid
    barrier_values = np.zeros_like(grid_x)
    for i in range(grid_x.shape[0]):
        for j in range(grid_x.shape[1]):
            p = np.array([[grid_x[i, j], grid_y[i, j]]])
            k_star = rbf_kernel(p, obstacle_points, length_scale, sigma_f)
            h = 1.0 + float(k_star @ alpha)
            barrier_values[i, j] = h

    return barrier_values


def visualize_scenario(obstacle_points, length_scale=0.25, title="CBF Barrier Field"):
    """Visualize a specific scenario"""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

    # === LEFT: Top-down view ===
    ax1.set_title('Obstacle Configuration', fontsize=14, fontweight='bold')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_xlim(-3, 3)
    ax1.set_ylim(-3, 3)
    ax1.grid(True, alpha=0.3)
    ax1.set_aspect('equal')

    # Plot obstacles
    if len(obstacle_points) > 0:
        ax1.scatter(obstacle_points[:, 0], obstacle_points[:, 1],
                   c='red', s=100, marker='x', linewidths=3, label=f'{len(obstacle_points)} obstacles', zorder=10)

    # Draw robot
    robot_circle = plt.Circle((0, 0), 0.15, color='blue', fill=True, alpha=0.7, label='Robot')
    ax1.add_patch(robot_circle)
    ax1.arrow(0, 0, 0.3, 0, head_width=0.1, head_length=0.1, fc='blue', ec='blue', zorder=10)

    ax1.legend(loc='upper right', fontsize=10)

    # === RIGHT: Barrier field ===
    ax2.set_title(f'{title}\nLength scale = {length_scale:.2f}m', fontsize=14, fontweight='bold')
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_xlim(-3, 3)
    ax2.set_ylim(-3, 3)
    ax2.set_aspect('equal')

    # Compute barrier field
    grid_range = 3.0
    grid_res = 0.05
    x_grid = np.arange(-grid_range, grid_range, grid_res)
    y_grid = np.arange(-grid_range, grid_range, grid_res)
    X_grid, Y_grid = np.meshgrid(x_grid, y_grid)

    Z_grid = compute_barrier_field(obstacle_points, X_grid, Y_grid, length_scale)

    # Plot barrier field
    contour = ax2.contourf(X_grid, Y_grid, Z_grid, levels=30, cmap='RdYlGn', alpha=0.9)
    plt.colorbar(contour, ax=ax2, label='Barrier h(x)')

    # Draw h=0 contour (unsafe boundary)
    ax2.contour(X_grid, Y_grid, Z_grid, levels=[0], colors='red', linewidths=3, linestyles='--', label='h=0 (unsafe)')

    # Plot obstacles
    if len(obstacle_points) > 0:
        ax2.scatter(obstacle_points[:, 0], obstacle_points[:, 1],
                   c='black', s=80, marker='x', linewidths=2, zorder=10)

    # Draw robot
    robot_circle2 = plt.Circle((0, 0), 0.15, color='blue', fill=True, alpha=0.7)
    ax2.add_patch(robot_circle2)
    ax2.arrow(0, 0, 0.3, 0, head_width=0.1, head_length=0.1, fc='blue', ec='blue', zorder=10)

    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='upper right', fontsize=10)

    plt.tight_layout()
    plt.show()


def compare_length_scales():
    """Compare different length scales"""
    # Create a simple scenario
    obstacles = np.array([
        [1.5, 0.0],
        [0.8, 1.2],
        [0.8, -1.2]
    ])

    fig, axes = plt.subplots(2, 2, figsize=(14, 12))
    length_scales = [0.1, 0.25, 0.5, 1.0]

    for idx, ls in enumerate(length_scales):
        ax = axes[idx // 2, idx % 2]
        ax.set_title(f'Length Scale = {ls:.2f}m', fontsize=12, fontweight='bold')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_xlim(-2, 2)
        ax.set_ylim(-2, 2)
        ax.set_aspect('equal')

        # Compute barrier field
        x_grid = np.arange(-2, 2, 0.05)
        y_grid = np.arange(-2, 2, 0.05)
        X_grid, Y_grid = np.meshgrid(x_grid, y_grid)
        Z_grid = compute_barrier_field(obstacles, X_grid, Y_grid, length_scale=ls)

        # Plot
        contour = ax.contourf(X_grid, Y_grid, Z_grid, levels=20, cmap='RdYlGn', alpha=0.9)
        ax.contour(X_grid, Y_grid, Z_grid, levels=[0], colors='red', linewidths=2, linestyles='--')
        plt.colorbar(contour, ax=ax)

        # Obstacles
        ax.scatter(obstacles[:, 0], obstacles[:, 1], c='black', s=80, marker='x', linewidths=2, zorder=10)

        # Robot
        robot_circle = plt.Circle((0, 0), 0.15, color='blue', fill=True, alpha=0.7)
        ax.add_patch(robot_circle)
        ax.arrow(0, 0, 0.3, 0, head_width=0.1, head_length=0.1, fc='blue', ec='blue', zorder=10)
        ax.grid(True, alpha=0.3)

    plt.suptitle('Effect of Length Scale on Barrier Field', fontsize=16, fontweight='bold')
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    print("CBF Barrier Visualization Tool")
    print("=" * 50)
    print("\n1. Single obstacle ahead")
    obstacles1 = np.array([[1.5, 0.0]])
    visualize_scenario(obstacles1, length_scale=0.25, title="Single Obstacle")

    print("\n2. Narrow corridor")
    obstacles2 = np.array([
        [1.0, 0.8],
        [1.0, -0.8],
        [1.5, 0.8],
        [1.5, -0.8],
        [2.0, 0.8],
        [2.0, -0.8]
    ])
    visualize_scenario(obstacles2, length_scale=0.25, title="Narrow Corridor")

    print("\n3. Compare length scales")
    compare_length_scales()
