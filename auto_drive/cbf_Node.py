#!/usr/bin/env python3
import rclpy
#import numpy as cp
import cupy as cp
import sys
#sys.path.append("/home/jetson/f1tenth_ws/src/auto_drive/auto_drive")
from dataclasses import dataclass
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray 
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from auto_drive.CBF import CBF



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
            dt: float = 1/10 # 10ms

            # Car info

            v: float = 1.0 # velocity
            u_max: float = [1,1.54] # max speed,angle
            u_min: float = [0.25,-1.54] # min speed,angle

            # Starting pose
            beta: float = 0
            gamma: float = 0
            theta: float = 0    
            
            x0: float = 0  # Start x
            y0: float = 0   # Start y
            
            # TODO get from model
            xdim: float = 4.0
            udim: float = 2.0
            lf: float = 1.0
            lr: float = 1.0

            # Obstacle position
            #TODO will be from lidar and continually updated figured out
            r_max: float = 2
            cbf_gamma: float = 1

            # Desired target point 
            #TODO this will still exist need to find a way to relate global to local

            weightslack:float = 10

            cbfrate:float = 1

        self.params = params
        self.CBFobj = CBF(params)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0
        self.u_ref = [1.0,0.0]

        # Publisher and Subscriber
        self.my_vel_command = self.create_publisher(AckermannDriveStamped, "/drive", 10) 

    def pose_callback(self,msg):
        print('pose')
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.theta = msg.pose.pose.orientation.x
        angle_rate = msg.twist.twist.angular.z
        print('theta')
        print(self.theta)
        self.v = msg.twist.twist.linear.x
        self.CBFobj.updateState(self.x,self.y,self.theta,self.v)


    def lidar_pose_callback(self, msg):
        #numpoints = len(r) # hard code instead
        self.params.ranges = cp.array(msg.ranges)
        angle = cp.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        self.CBFobj.setObjects(self.params.ranges,angle)
        
        try:
            u = (self.CBFobj.constraints_cost(u_ref=self.u_ref,x=self.params.x,y=self.params.y,theta=self.theta,v=self.v))
            print("success")
            self.send_vel(1.0,u[1])
        except Exception as e:
            print('failed lidar')
            print(f"An error occurred: {e}")
            pass
        
    def send_vel(self,x,z):
        msg = AckermannDriveStamped()
        msg.drive.speed = float(1.0)  # Set desired velocity in m/s
        msg.drive.steering_angle = float(z)  # Set steering angle in radians
        # self.get_logger().info('msg =: "%s"' % my_msg)
        self.my_vel_command.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = Controller_Node()
    controller.get_logger().info("Hello friend!")
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
