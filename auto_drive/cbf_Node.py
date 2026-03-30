#!/usr/bin/env python3
import rclpy
import os
#import numpy as cp
import cupy as cp
import sys
#sys.path.append("/home/jetson/f1tenth_ws/src/auto_drive/auto_drive")
from dataclasses import dataclass
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray 
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from auto_drive.CBF import CBF
import time

# After your other imports
from dataclasses import dataclass

# Add this below your imports
class SecondOrderULM_KF:
    def __init__(self, Ts, beta0, Q=None, R=None, y0=0.0):
        self.Ts = Ts
        self.x = cp.array([y0, 0.0, 0.0, beta0], dtype=float)
        self.P = cp.diag([1.0, 1.0, 10.0, 10.0])
        self.Q = cp.diag([1e-5,1e-4,1e-2,1e-2]) if Q is None else Q
        self.R = cp.array([[1e-4]]) if R is None else R
        self.H = cp.array([[1.0,0.0,0.0,0.0]])
    def predict(self, u):
        Ts = self.Ts
        A = cp.array([
            [1.0, Ts, 0.0, Ts*u],
            [0.0, 1.0, Ts, Ts*u],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ])
        self.x = A @ self.x
        self.P = A @ self.P @ A.T + self.Q
    def update(self, y_meas):
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T / S
        self.x = self.x + (K.flatten() * (y_meas - self.H @ self.x))
        self.P = (cp.eye(4) - K @ self.H) @ self.P
    def get_estimates(self):
        return tuple(self.x)  # returns (y_hat, ydot_hat, F_hat, beta_hat)

class Controller_Node(Node):
    def __init__(self):
        super().__init__('Controller_Node')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.pose_callback,
            10)
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_pose_callback,
            10)
        

        class params():
            dt: float = 1#0.05#1/10 # 10ms

            # Car info

            v: float = 1.0 # velocity
            u_max: float = [1.0,0.85] # max speed,angle
            u_min: float = [-0.0,-0.85] # min speed,angle

            # Starting pose
            beta: float = 0.0
            gamma: float = 0.0
            theta: float = 0.0    
            
            x0: float = 0.0  # Start x
            y0: float = 0.0   # Start y
            
            # TODO get from model
            xdim: float = 4
            udim: float = 2
            lf: float = 0.23
            lr: float = 0.2

            # Obstacle position
            #TODO will be from lidar and continually updated figured out
            r_max: float = 5.0
            cbf_gamma: float = 1.0
            # Desired target point 
            #TODO this will still exist need to find a way to relate global to local
            weightslack:float = 1.0
            cbfrate:float = 1.0
            #self.length_scale = 0.08    # found from  loop demo

        self.params = params
        self.CBFobj = CBF(params)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0
        #self.u_ref = [self.params.v,0.0]
        self.u_ref = [0.5,0.0]

                # Initialize ULMs for ρ and α (you can tune beta0)
        self.ulm_rho = SecondOrderULM_KF(Ts=self.params.dt, beta0=1.0, y0=1.0)
        self.ulm_alpha = SecondOrderULM_KF(Ts=self.params.dt, beta0=1.0, y0=5.0)  # alpha initial guess

        # Publisher and Subscriber
        self.my_vel_command = self.create_publisher(AckermannDriveStamped, "/drive", 10)
        #self.state_publisher = self.create_publisher(Float32MultiArray, "/state", 10) 

    def pose_callback(self,msg):
        #print('pose')
        #start = time.time()
        self.x = 0.0 #msg.pose.pose.position.x
        self.y = 0.0 #msg.pose.pose.position.y
        self.theta = 0.0 #msg.pose.pose.orientation.x
        angle_rate = msg.twist.twist.angular.z
        #print('V')
        #print(self.v)
        #self.v = msg.twist.twist.linear.x
        self.v = 1.0
        #self.CBFobj.updateState(self.x,self.y,self.theta,self.v)
        #self.CBFobj.updateState(0.0,0.0,self.theta,self.v)
        #total_time = time.time() - start
        #self.get_logger().info(f"pose callback time: {total_time:.3f}")


    def lidar_pose_callback(self, msg):
        """
        Process LiDAR data, update ULM estimates, compute CBF-constrained control,
        and send velocity commands.
        """
        start_time = time.time()

        # --- Step 1: Extract LiDAR ranges and compute angles ---
        ranges = cp.array(msg.ranges)
        angles = cp.linspace(msg.angle_min, msg.angle_max, len(ranges))  # ensure same length

        # --- Step 2: Update CBF object with obstacles ---
        self.CBFobj.setObjects(ranges, angles)

        # --- Step 3: Predict/update ULM estimates for rho and alpha ---
        rho_meas = 1.0    # placeholder measurement
        alpha_meas = 5.0  # placeholder measurement

        self.ulm_rho.predict(self.u_ref[0])
        self.ulm_alpha.predict(self.u_ref[1])

        self.ulm_rho.update(rho_meas)
        self.ulm_alpha.update(alpha_meas)

        rho_hat, _, _, _ = self.ulm_rho.get_estimates()
        alpha_hat, _, _, _ = self.ulm_alpha.get_estimates()
        self.get_logger().info(f"ULM estimates: rho={rho_hat:.3f}, alpha={alpha_hat:.3f}")

        # --- Step 4: Compute CBF-constrained control ---
        try:
            u, state = self.CBFobj.constraints_cost(
                u_ref=self.u_ref,
                x=self.x,
                y=self.y,
                theta=self.theta,
                v=self.v,
                alpha=alpha_hat
            )
        except Exception as e:
            self.get_logger().error(f"CBF constraint cost failed: {e}")
            u = [0.0, 0.0]

        # --- Step 5: Send velocity command once ---
        self.send_vel(u[0], u[1])

        total_time = time.time() - start_time
        self.get_logger().info(f"LiDAR callback total time: {total_time:.3f}s")
        
    def send_vel(self,x,z):
        msg = AckermannDriveStamped()
        msg.drive.speed = float(x)  # Set desired velocity in m/s
        msg.drive.steering_angle = -float(z) # Set steering angle in radians
        #self.get_logger().info('msg =: "%s"' % msg)
        self.my_vel_command.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = Controller_Node()
    controller.get_logger().info("Hello friend!")
    print(os.cpu_count())
    # Use a multi-threaded executor (for example, with 4 threads)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(controller)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()
    # rclpy.spin(controller)
    # controller.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()
