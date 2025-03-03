#!/usr/bin/env python3
import rclpy
import math
import numpy as np
import sys
sys.path.append("/home/parallels/ackerman/src/bridge2ros/bridge2ros")
from dataclasses import dataclass
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray 
from sensor_msgs.msg import LaserScan
from PP import PP
from Visulise_kernals import MyFig



class Controller_Node(Node):
    def __init__(self):
        super().__init__('Controller_Node')

        self.subscription = self.create_subscription(Odometry,'/model/Car/odometry',self.pose_callback,10)
        # self.subscription = self.create_subscription(LaserScan,'lidar',self.lidar_pose_callback,10)
        

        class params():
            # for qp
            dt: float = 1/100 # 100ms

            # Car info

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

            beta: float = 0 
            gamma: float = 0
            theta: float = 0 

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
        # Publisher and Subscriber
        self.my_vel_command = self.create_publisher(Twist, "cmd_vel", 10)       # Send velocity and steer angle
        self.visual = self.create_publisher(Float64MultiArray, "visual", 10)    # send data to visulise will be changing
        

    def pose_callback(self,msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Quarternon to euler
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        t3 = +2.0 * (w * z)
        t4 = +1.0 - 2.0 * (z * z)
        theta = math.atan2(t3, t4)
        
        # speed
        v = msg.twist.twist.linear.x
        v,theta = self.PP.control(x,y,v,theta)
        # print('x: y: v: theta:')
        # print(x,y,v,theta)
        self.send_vel(v,theta)


    def lidar_pose_callback(self, msg):
        r = np.array(msg.ranges)  # DistanceSS
        numpoints = len(r)
        self.angle = (msg.angle_max - msg.angle_min)/numpoints
        angle = np.arange(msg.angle_min, msg.angle_max, self.angle)
        
        try:
            pass
        except Exception as e:
            print(f"An error occurred: {e}")

    def send_vel(self,x,z):
        my_msg = Twist()
        my_msg.linear.x = float(x)
        my_msg.angular.z = float(z)
        # print(x,z)
        self.my_vel_command.publish(my_msg)

def main(args=None):
    rclpy.init(args=args)
    controller = Controller_Node()
    controller.get_logger().info("Hello friend!")
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
