#!/usr/bin/env python3
"""
Real-time barrier visualization for RViz
Publishes visualization markers without blocking control loop
"""
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import cupy as cp
import numpy as np


class BarrierVisualizer(Node):
    """
    Publishes barrier visualization to RViz at lower rate than control loop.
    Runs in separate thread to avoid blocking.
    """

    def __init__(self, cbf_object, controller_node):
        super().__init__('barrier_visualizer')

        self.cbf = cbf_object
        self.controller = controller_node

        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, '/barrier_viz', 10)

        # Visualization parameters
        self.grid_size = 30  # 30x30 grid (lower = faster)
        self.x_range = (-2.0, 3.0)  # Meters
        self.y_range = (-2.0, 2.0)
        self.viz_rate = 5.0  # Hz (update visualization at 5Hz, not 100Hz!)

        # Timer for visualization updates (runs in separate thread)
        self.create_timer(1.0 / self.viz_rate, self.publish_visualization)

        self.get_logger().info(f'Barrier visualizer started at {self.viz_rate}Hz')

    def publish_visualization(self):
        """Publish barrier visualization to RViz"""
        try:
            markers = MarkerArray()

            # Marker 1: Barrier surface as colored mesh
            barrier_marker = self.create_barrier_surface()
            if barrier_marker:
                markers.markers.append(barrier_marker)

            # Marker 2: Zero-level set (safety boundary)
            boundary_marker = self.create_safety_boundary()
            if boundary_marker:
                markers.markers.append(boundary_marker)

            # Marker 3: Robot position and velocity
            robot_marker = self.create_robot_marker()
            if robot_marker:
                markers.markers.append(robot_marker)

            # Marker 4: Obstacle points
            obstacle_marker = self.create_obstacle_marker()
            if obstacle_marker:
                markers.markers.append(obstacle_marker)

            # Marker 5: Goal marker
            goal_marker = self.create_goal_marker()
            if goal_marker:
                markers.markers.append(goal_marker)

            # Publish all markers
            self.marker_pub.publish(markers)

        except Exception as e:
            self.get_logger().warn(f'Visualization error: {e}')

    def create_barrier_surface(self):
        """Create colored surface showing barrier values"""
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "barrier_surface"
        marker.id = 0
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.pose.orientation.w = 1.0

        # Generate grid
        x = np.linspace(self.x_range[0], self.x_range[1], self.grid_size)
        y = np.linspace(self.y_range[0], self.y_range[1], self.grid_size)

        # Compute barrier at each grid point (on CPU for RViz compatibility)
        barrier_values = np.zeros((self.grid_size, self.grid_size))

        for i in range(self.grid_size):
            for j in range(self.grid_size):
                pos = [x[i], y[j]]
                h, _ = self.cbf.get_barrier_and_variance(pos)
                barrier_values[i, j] = h

        # Create triangles for surface visualization
        z_scale = 0.1  # Scale barrier values for visibility

        for i in range(self.grid_size - 1):
            for j in range(self.grid_size - 1):
                # Two triangles per grid cell
                # Triangle 1: (i,j), (i+1,j), (i,j+1)
                p1 = Point(x=float(x[i]), y=float(y[j]), z=float(barrier_values[i, j] * z_scale))
                p2 = Point(x=float(x[i+1]), y=float(y[j]), z=float(barrier_values[i+1, j] * z_scale))
                p3 = Point(x=float(x[i]), y=float(y[j+1]), z=float(barrier_values[i, j+1] * z_scale))

                marker.points.extend([p1, p2, p3])

                # Color based on barrier value (red=unsafe, green=safe)
                for val in [barrier_values[i, j], barrier_values[i+1, j], barrier_values[i, j+1]]:
                    color = self.value_to_color(val)
                    marker.colors.append(color)

                # Triangle 2: (i+1,j), (i+1,j+1), (i,j+1)
                p4 = Point(x=float(x[i+1]), y=float(y[j]), z=float(barrier_values[i+1, j] * z_scale))
                p5 = Point(x=float(x[i+1]), y=float(y[j+1]), z=float(barrier_values[i+1, j+1] * z_scale))
                p6 = Point(x=float(x[i]), y=float(y[j+1]), z=float(barrier_values[i, j+1] * z_scale))

                marker.points.extend([p4, p5, p6])

                for val in [barrier_values[i+1, j], barrier_values[i+1, j+1], barrier_values[i, j+1]]:
                    color = self.value_to_color(val)
                    marker.colors.append(color)

        return marker

    def create_safety_boundary(self):
        """Create line showing h(x,y) = 0 contour (safety boundary)"""
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "safety_boundary"
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05  # Line width
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Red
        marker.pose.orientation.w = 1.0

        # Find zero-level contour using marching squares approximation
        x = np.linspace(self.x_range[0], self.x_range[1], self.grid_size)
        y = np.linspace(self.y_range[0], self.y_range[1], self.grid_size)

        for i in range(self.grid_size - 1):
            for j in range(self.grid_size - 1):
                # Check if zero-crossing in this cell
                h00, _ = self.cbf.get_barrier_and_variance([x[i], y[j]])
                h10, _ = self.cbf.get_barrier_and_variance([x[i+1], y[j]])
                h01, _ = self.cbf.get_barrier_and_variance([x[i], y[j+1]])
                h11, _ = self.cbf.get_barrier_and_variance([x[i+1], y[j+1]])

                values = [h00, h10, h01, h11]
                if (min(values) <= 0 <= max(values)):
                    # Zero-crossing detected, add point
                    marker.points.append(Point(x=float(x[i]), y=float(y[j]), z=0.05))

        return marker if len(marker.points) > 0 else None

    def create_robot_marker(self):
        """Create arrow showing robot position and heading"""
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "robot"
        marker.id = 2
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Arrow from robot position in direction of heading
        start = Point(x=float(self.controller.x), y=float(self.controller.y), z=0.0)
        end = Point(
            x=float(self.controller.x + 0.5 * np.cos(self.controller.theta)),
            y=float(self.controller.y + 0.5 * np.sin(self.controller.theta)),
            z=0.0
        )
        marker.points = [start, end]

        marker.scale.x = 0.1  # Shaft diameter
        marker.scale.y = 0.15  # Head diameter
        marker.scale.z = 0.15  # Head length
        marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)  # Blue

        return marker

    def create_obstacle_marker(self):
        """Create spheres showing detected obstacles"""
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "obstacles"
        marker.id = 3
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.8)  # Black
        marker.pose.orientation.w = 1.0

        # Get obstacle points from CBF (already in robot frame)
        if self.cbf.N > 0:
            obstacles = self.cbf.obstacle_points.get()  # Convert CuPy to NumPy

            # Transform from robot frame to odom frame
            cos_theta = np.cos(self.controller.theta)
            sin_theta = np.sin(self.controller.theta)

            for obs in obstacles:
                # Rotation + translation
                x_odom = self.controller.x + obs[0] * cos_theta - obs[1] * sin_theta
                y_odom = self.controller.y + obs[0] * sin_theta + obs[1] * cos_theta

                marker.points.append(Point(x=float(x_odom), y=float(y_odom), z=0.0))

        return marker if len(marker.points) > 0 else None

    def create_goal_marker(self):
        """Create marker showing goal position"""
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "goal"
        marker.id = 4
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        marker.pose.position.x = float(self.controller.goal_x)
        marker.pose.position.y = float(self.controller.goal_y)
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.5  # Diameter
        marker.scale.y = 0.5
        marker.scale.z = 0.05  # Height
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.5)  # Green, semi-transparent

        return marker

    def value_to_color(self, value):
        """Convert barrier value to color (red=unsafe, yellow=marginal, green=safe)"""
        # Map barrier values: <0=red, 0-0.5=yellow, >0.5=green
        color = ColorRGBA()
        color.a = 0.7  # Transparency

        if value < 0:
            # Red (unsafe)
            color.r = 1.0
            color.g = 0.0
            color.b = 0.0
        elif value < 0.5:
            # Gradient red -> yellow
            t = value / 0.5
            color.r = 1.0
            color.g = t
            color.b = 0.0
        else:
            # Gradient yellow -> green
            t = min((value - 0.5) / 0.5, 1.0)
            color.r = 1.0 - t
            color.g = 1.0
            color.b = 0.0

        return color


def add_visualization_to_controller(controller_node):
    """
    Add visualization to existing controller node.
    Call this from main() after creating ControllerNode.

    Args:
        controller_node: Instance of ControllerNode

    Returns:
        visualizer: BarrierVisualizer node
    """
    visualizer = BarrierVisualizer(controller_node.cbf, controller_node)
    return visualizer
