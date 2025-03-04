
import numpy as np
from scipy.interpolate import CubicSpline 
import matplotlib.pyplot as ptl
from dataclasses import dataclass

class PP:
    # Initiate Car
    def __init__(self,param):
        """
        Set up the car with parameters and dynamics
        """
        @dataclass
        class params:
            pass
        self.params = param

        # Local cordinate system
        self.params.x = np.array((self.params.x0))
        self.params.y = np.array((self.params.x0))
        self.params.theta = self.params.theta0
        self.params.goal = [20,5]

        # CBF inits
        self.params.Y = {}
        self.params.sigma_f = 1

        #states 
        self.vistate = 0
        self.thetaistate = 0
        self.velocity_actuator_error = 0
        self.theta_actuator_error = 0

        pass
    def get_trajectory(self,x_waypoints,y_waypoints):
        index = np.linspace(0,10,len(x_waypoints)) 
        path_x = CubicSpline(index,x_waypoints)
        path_y = CubicSpline(index,y_waypoints) 
        index = np.linspace(0,10,20)   
        #fig, ax = ptl.subplots(figsize=(6.5, 4))    
        self.path_x = path_x(index)
        self.path_xx = self.path_x
        self.path_y = path_y(index) 
        self.path_yy = self.path_y

    def get_pdes(self,x,y,lookahead):
        try:
            dist = np.sqrt((self.path_x[0]-x)**2+(self.path_y[0]-y)**2)
            print("location x,y:")
            print(x,y)
            self.params.x = np.hstack((self.params.x,x))
            self.params.y = np.hstack((self.params.y,y))
            if (dist>lookahead):
                return self.path_x[0],self.path_y[0]
            else:
                if (dist<0.1):
                    self.plot()
                    return x,y
                if (self.path_x.size == 1):
                    return self.path_x[0],self.path_y[0]
                
                # remove first element track second
                self.path_x = np.delete(self.path_x , 0)
                self.path_y = np.delete(self.path_y , 0)
                return self.path_x[0],self.path_y[0]
        except:
            return x,y
        
        

    def get_velocity_des(self,x_des,y_des,x,y):
        v = np.sqrt((x_des-x)**2+(y_des-y)**2)
        # Clip
        return np.clip(v,0,1)
    
    def get_theta_des(self,x_des,y_des,x,y):
        theta = np.arctan2(y_des-y,x_des-x)
        return np.clip(theta,-np.pi/2,np.pi/2)

    def control(self,x,y,v,theta):
        # test as GTG add index here after
        lookahead = 4
        xdes , ydes = self.get_pdes(x ,y, lookahead)
        xdes = 20
        ydes = 20
        # Get desired velocity and theta
        vdes = self.get_velocity_des(xdes,ydes,x,y)
        thetades = self.get_theta_des(xdes,ydes,x,y)

        return vdes,thetades
