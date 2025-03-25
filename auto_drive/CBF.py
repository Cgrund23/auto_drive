import autograd.numpy as np
from sympy import symbols
from dataclasses import dataclass
import numdifftools as nd
from autograd import jacobian
import cvxpy as cp
from qpsolvers import solve_qp
import time
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

class CBF:
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
        self.params.x = 0
        self.params.y = 0
        self.params.Od = {}
        self.params.Y = {}
        self.params.sigma_f = 1
        
        pass
        self.length_scale = 1.25    # found from  loop demo

    # Dynamics #

    def f(self):
        """
        The forced dynamics of the car bike model
        """
        x = np.array([1, 0, -self.params.v * np.sin(self.params.theta + self.params.beta)*self.params.dt])
        y = np.array([0, 1, self.params.v * np.cos(self.params.theta + self.params.beta)*self.params.dt])
        t = np.array([0, 0 , 1])
        return np.vstack((x,y,t))
    
    def g(self):
        """
        The natrual dynamics of the ackerman steering car bike
        """
        return np.array([
            [np.cos(self.params.theta + self.params.beta) * self.params.dt, 0],
            [np.cos(self.params.theta + self.params.beta) * self.params.dt, 0],
            [np.cos(self.params.beta) / (self.params.lf + self.params.lr) * np.tan(self.params.gamma) * self.params.dt,
            self.params.v * np.cos(self.params.beta) / ((self.params.lf + self.params.lr) * np.cos(self.params.gamma)**2) * self.params.dt]
        ])

    def c(self):
        """
        Path through component
        """
        return np.array([self.params.v * self.params.theta * np.sin(self.params.theta + self.params.beta)*self.params.dt,
            -self.params.v * self.params.theta * np.cos(self.params.theta + self.params.beta)*self.params.dt,
            -self.params.v * self.params.gamma * np.cos(self.params.beta)/((self.params.lf+self.params.lr)*np.cos(self.params.gamma)**2)*self.params.dt]).reshape((1,3))
    
    def x(self):
        """
        Returns the location of the car and angle of tires / Fornow x,y 0 always
        """
        return np.array([self.params.x, self.params.y, self.params.theta],dtype=float).reshape((3,1))
    
    def updateState(self, V, gamma):
        """
        Sets all global variables from "sensor" data
        """
        self.params.v = V
        self.params.gamma = gamma
        self.params.beta = np.arctan2((self.params.lf*np.tan(gamma)),(self.params.lf+self.params.lr))
        self.params.theta = (V*np.cos(self.params.beta)/(self.params.lf+self.params.lr))*np.tan(gamma)
        pass
    
    def setObjects(self,distance,angle):                               
        """
        Take all lidar points and turn them into data
        """
        M = len(distance)   # Total Number of possible lidar data points
        self.N = 0          # Total number of points in range

        # Instantiate matrix
        filtered_distance = []
        filtered_angle = []

        for k in range(M):
        # Keep only points within max lidar field
            if not np.isinf(distance[k]):                               
                self.N += 1
                filtered_distance.append(distance[k])
                filtered_angle.append(angle[k]) 
        
        # Create value and distance to plant of all points
        self.Y = -1*np.ones(self.N)
        self.NY = np.ones(self.N)
        self.Dist = np.array(filtered_distance).reshape((self.N,1))
        
        # Convert to x y cordinates
        x_lidar = np.array(np.array(filtered_distance) * np.cos(np.array(filtered_angle) + self.params.theta)+self.params.x).reshape((self.N, 1))
        y_lidar = np.array(np.array(filtered_distance) * np.sin(np.array(filtered_angle) + self.params.theta)+self.params.y).reshape((self.N, 1))

        self.Poe = np.hstack((x_lidar,y_lidar))


    def rbf_kernel(self, X1, X2, length_scale, sigma_f):
        """
        Computes the RBF (Radial Basis Function) kernel between X1 and X2.
        """
        
        sqdist = np.sum(X1**2, 1).reshape(-1, 1) + np.sum(X2**2, 1) - 2 * X1 @ X2.T  # distance between points in X1 and X2
                                                                                    # note the dimentions in the sums!
                                                                               # all distances between pairs of points
        return sigma_f * np.exp(-0.5 * (sqdist / length_scale**2))                  # Same kernel as in paper


    def cbf_function(self, x_test, X_train, length_scale, sigma_f):
        """
        Computes the CBF
        """
        return 1-2*(self.rbf_kernel(x_test, X_train, length_scale, sigma_f))
        #return  self.rbf_kernel(x_test, X_train, length_scale, sigma_f) @ alpha - safe_dist
      
    def dcbf_function(self, x_test, X_train, length_scale, sigma_f):
        """
        Compute the derivitive of the cbf function
        """
        return  -1 / length_scale**2 * self.rbf_kernel(x_test, X_train, length_scale, sigma_f)*(x_test-X_train)

    def lf_cbf_function(self,dcbf):
        """
        Derivitive of the cbf function by the forced dynamics
        """
        f = self.f()
        return dcbf.T @ f
     
    def lg_cbf_function(self,dcbf):
        """
        Derivitive of the cbf function by the Input dynamics
        """
        g = self.g()
        return dcbf.T @ g

    # Constraints/Cost
    def constraints_cost(self,u_ref,x,y,theta,v):
        self.updateState(v,theta)
        self.params.x,self.params.y = x,y
        # Create variables for optimisation 
        self.u_ref = u_ref
        A = np.empty((0,2), float)
        B = {}
        b = np.empty((0,1),float)
        LfB = {}
        LgB = {}

        X_query = self.f()
          # Note desired "saftey" TUNE
                          
        
    # Create grid of safe and unsafe
        x_width = 12
        y_width = 12
        resolution = .5    # resolution of lidar data
        grid_size = int(x_width*1/resolution)    # Grid resolution matches lidar grid
        x_grid, y_grid = np.meshgrid(np.linspace(-x_width, x_width, grid_size), np.linspace(-y_width, y_width, grid_size))
        safety_matrix = np.column_stack((x_grid.ravel(), y_grid.ravel()))
        
        k_ss = self.rbf_kernel(safety_matrix,safety_matrix,self.length_scale,self.params.sigma_f)
 
        # Fill array with barrier locations
        K = self.rbf_kernel(self.Poe,self.Poe,self.length_scale,self.params.sigma_f)
        k_star = self.rbf_kernel(self.Poe,safety_matrix,self.length_scale,self.params.sigma_f)
        Poe_angles = np.arctan2(self.Poe[:,1],self.Poe[:,0]).reshape((self.Poe.shape[0],1))
        self.PoeA = np.hstack((self.Poe,Poe_angles))
        K_self = self.rbf_kernel(self.PoeA,X_query,self.length_scale,self.params.sigma_f)
        q = X_query[:,:2]
        K_selfish = self.rbf_kernel(X_query[:,:2],self.Poe,self.length_scale,self.params.sigma_f)
        k_inv = np.linalg.pinv(K)
        h_control = 1-2*(K_self.T @ k_inv @ - self.Y)
        h_control[h_control > 1] = 1
        h_control[h_control < -1] = -1

        h_world =  1-2*(k_star.T @ k_inv @ - self.Y)
        
        # h = h.reshape((int(h.size/2),2))
        # Reshape for plotting
        
        # h_grid = h.reshape(grid_size, grid_size)
        # plt.figure()
        # plt.contourf(x_grid, y_grid, h_grid, 20, cmap='twilight')
        # plt.colorbar()
        # plt.title('h')
        # plt.xlabel('X [m]')
        # plt.ylabel('Y [m]')
        # plt.legend()
        # plt.show()

        dkdp = -1/self.length_scale**2*K_selfish
        dcbf = self.Y.T @ k_inv @ dkdp.T
        # dcbf = dcbf.reshape((int(dcbf.size/2),2))
        # dcbf = dkdp @ k_inv @ self.Y.T
        # dcbf[dcbf > 1] = 1
        # dcbf[dcbf < -1] = -1
        
        # dcbf_grid = dcbf.reshape(grid_size, grid_size)
        # plt.figure()
        # plt.contourf(x_grid, y_grid, dcbf_grid, 20, cmap='twilight')
        # plt.colorbar()
        # plt.title('dh/dx')
        # plt.xlabel('X [m]')
        # plt.ylabel('Y [m]')
        # plt.legend()
        # plt.show()

        # h = np.hstack((h,np.ones((h.shape[0],1))))
        # dcbf = np.hstack((dcbf,np.zeros((dcbf.shape[0],1))))

        ##TODO add theta of all points to dcbf function??? 
        b = self.lg_cbf_function(dcbf) 
        b = b @ self.u_ref
       
        b = b.reshape((b.size,1))
         
        A = -self.lf_cbf_function(dcbf) 
        A -= h_control**3
        
        
        # umax constraints
        k = np.hstack(([np.eye(self.params.udim), np.zeros((self.params.udim, 1))]))
        A = np.vstack((A,k))
        k = np.array((self.params.u_max))
        b = np.vstack((b.reshape((b.shape[0],1)),k.reshape((k.size,1))))

        # u_min constraints
        A = np.vstack((A,np.hstack((-np.eye(self.params.udim), np.zeros((self.params.udim, 1))))))
        k = np.array((self.params.u_min))
        b = np.vstack((b,-k.reshape((k.size,1))))
        weight_input = np.eye(2)

        H = np.array(((1,0),(0,1)))
        f_ = (weight_input) @ (-self.u_ref).reshape(2,1)

                  
        #     # Optimal control input
        try:  
            x = solve_qp(H, f_, A, b, solver = "clarabel") 
            print(x)  
            self.u = x[0]
            #TODO update from imu data
            ##self.updateState(x[1],x[0])
            self.params.gamma = float(x[1])
            self.params.v = float(x[0])
            self.params.weightslack = float(x[2])
            x = [1,1]
            return x,h_world,dcbf
        except Exception as e:
            print(f"An error occurred: {e}")
            return [0,0],h_world,dcbf
        

        
