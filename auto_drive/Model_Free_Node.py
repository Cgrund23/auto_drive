#!/usr/bin/env python3
import rclpy
import numpy as np
import sys
sys.path.append("/home/jetson/f1tenth_ws/src/auto_drive/auto_drive")
from dataclasses import dataclass
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray 
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

from PP import PP
from IP_ackermann import IP


class Controller_Node(Node):
    def __init__(self):
        super().__init__('Controller_Node')

        self.subscription = self.create_subscription(Odometry,'/odom',self.pose_callback,10)
        self.subscription = self.create_subscription(LaserScan,'/scan',self.lidar_pose_callback,10)

        class params():

            xdim: float = 4
            udim: float = 2
            lf: float = 1
            lr: float = 1
 
            u_max: float = [1,1.54] # max accel, angle
            u_min: float = [0.25,-1.54] # min accel, angle

            # Initial state
            
            x0: float = 0   # Start x
            y0: float = 0   # Start y
            theta0: float = 0 # start theta
            v0: float = 0.0 # initial velocity
            state: float = [x0,y0,theta0,v0] # starting state vector

            # Gains
            Kvi: float = .01
            Kvp: float = 5

            Kthetap: float = .5
            Kthetai: float = .1

            # waypoints
            wx: float = [2, 10, 20, 30.0]
            wy: float = [2, 7, 1, 3.0]

        self.params = params # store structure
        self.PP = PP(self.params) # pass structure to car
        self.PP.get_trajectory(self.params.wx,self.params.wy)
        self.IP_vel = IP(alpha = 8, kp = 100,dt = 0.001)
        self.IP_theta = IP(alpha = 0.25, kp = 100.0,dt = 0.001)

        # Publisher
        self.my_vel_command = self.create_publisher(AckermannDriveStamped, "/drive", 10)       # Send velocity and steer angle
        self.visual = self.create_publisher(Float64MultiArray, "visual", 10)    # send data to visulise will be changing

        

    def pose_callback(self,msg):
        #print("pose call")
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Quarternon to euler
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        t3 = +2.0 * (w * z)
        t4 = +1.0 - 2.0 * (z * z)
        theta = np.arctan2(t3, t4)
        
        
        # speed
        v = msg.twist.twist.linear.x
        print("velocity of car")
        print(v)
        #vdes,thetades = self.PP.control(x,y,v,theta)
        vdes = -1
        v = self.IP_vel.control(v,vdes)
        #theta = self.IP_theta.control(theta,x_ref=thetades)
        print(v,theta)
        theta = 0
        self.send_vel(v,theta)

    def lidar_pose_callback(self, msg):
        #print("lidar call")
        r = np.array(msg.ranges)  # DistanceS
        numpoints = len(r)
        self.angle = (msg.angle_max - msg.angle_min)/numpoints
        angle = np.arange(msg.angle_min, msg.angle_max, self.angle)
        print("min distance")
        print(r.min())
        if r.min() < .1:
            #self.send_vel(0,0)
            return
        try:
            #print(msg)
            pass
        except Exception as e:
            print(f"An error occurred: {e}")

    def send_vel(self,x,z):
        z = 0.0
        msg = AckermannDriveStamped()
        msg.drive.speed = float(x)  # Set desired velocity in m/s
        msg.drive.steering_angle = float(z)  # Set steering angle in radians
        self.my_vel_command.publish(msg)
        self.get_logger().info(f'Publishing Velocity:{msg.drive.speed} m/s')

def main(args=None):
    rclpy.init(args=args)
    controller = Controller_Node()
    controller.get_logger().info("Hello friend!")
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
