#import autograd.numpy as np
from dataclasses import dataclass
from qpsolvers import solve_qp
import cupy as cp
#import numpy as np
import time

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
        self.params.x = 0.0
        self.params.y = 0.0
        self.params.Od = {}
        self.params.Y = {}
        self.params.sigma_f = 1.0*10**-2
        
        pass
        self.length_scale = 0.05    # found from  loop demo

    # Dynamics #

    def f(self):
        """
        The forced dynamics of the car bike model
        """
        x = cp.array([cp.array(1, dtype=cp.float32),cp.array(0, dtype=cp.float32), -self.params.v * cp.sin(self.params.theta + self.params.beta)*self.params.dt],dtype=cp.float32)
        y = cp.array([cp.array(0, dtype=cp.float32),cp.array(1, dtype=cp.float32), self.params.v * cp.cos(self.params.theta + self.params.beta)*self.params.dt])
        t = cp.array([0.0, 0.0 , 1.0])
        return cp.vstack((x,y,t))
    
    def f_full(self):
        """Returns the next state using CuPy arrays.

        Returns:
            cp.ndarray: State update array of shape (4, 1)
        """
        #print('f_full')
        dx = self.params.v * cp.cos(self.params.theta + self.params.beta) * self.params.dt
        dy = self.params.v * cp.sin(self.params.theta + self.params.beta) * self.params.dt
        return cp.array([dx, dy, cp.array(0.0), cp.array(0.0)]).reshape((4, 1))
    
    def g(self):
        """
        The natrual dynamics of the ackerman steering car bike
        """
        return cp.array([
            [cp.cos(self.params.theta + self.params.beta) * self.params.dt, cp.array(0, dtype=cp.float32)],
            [cp.cos(self.params.theta + self.params.beta) * self.params.dt, cp.array(0, dtype=cp.float32)],
            [cp.cos(self.params.beta) / (self.params.lf + self.params.lr) * cp.tan(self.params.gamma) * self.params.dt,
            self.params.v * cp.cos(self.params.beta) / ((self.params.lf + self.params.lr) * cp.cos(self.params.gamma)**2) * self.params.dt]
        ])
    def g_full(self):
        """
        The natural dynamics of the Ackermann steering bicycle model.
        """
        v = cp.asarray(self.params.v)
        theta = cp.asarray(self.params.theta)
        beta = cp.asarray(self.params.beta)
        dt = cp.asarray(self.params.dt)
        lf = cp.asarray(self.params.lf)

        return cp.array([
            [cp.array(0.0), -v * cp.sin(theta + beta) * dt],
            [cp.array(0.0),  v * cp.cos(theta + beta) * dt],
            [cp.array(1.0),  cp.array(0.0)],
            [cp.array(0.0),  v / lf]
        ]).reshape((4, 2))
    
    def c(self):
        """
        Path through component
        """
        return cp.array([self.params.v * self.params.theta * cp.sin(self.params.theta + self.params.beta)*self.params.dt,
            -self.params.v * self.params.theta * cp.cos(self.params.theta + self.params.beta)*self.params.dt,
            -self.params.v * self.params.gamma * cp.cos(self.params.beta)/((self.params.lf+self.params.lr)*cp.cos(self.params.gamma)**2)*self.params.dt]).reshape((1,3))
    
    def x(self):
        """
        Returns the location of the car and angle of tires / Fornow x,y 0 always
        """
        return cp.array([self.params.x, self.params.y, self.params.theta],dtype=cp.float32).reshape((3,1))
    
    def updateState(self, x, y, V, gamma):
        """
        Sets all global variables from "sensor" data
        """
        self.params.x = x
        self.params.y = y
        self.params.v = V
        self.params.gamma = gamma
        self.params.beta = cp.arctan2((self.params.lf*cp.tan(gamma)),(self.params.lf+self.params.lr))
        self.params.theta = (V*cp.cos(self.params.beta)/(self.params.lf+self.params.lr))*cp.tan(gamma)
        pass
    
    def check_constraints_feasibility(self, A, b, num_iters=1000, lr=1e-3, penalty=1e4, tol=1e-6):
        """
        Check the feasibility of inequality constraints A x <= b using a penalty method entirely in CuPy.
        
        We approximately solve:
            minimize    f(x, xi) = xi + penalty * sum(max(0, A x - b - xi))
            subject to  xi >= 0
        via a simple subgradient descent approach.
        
        Parameters:
        A: cp.array of shape (m, n) -- constraint matrix.
        b: cp.array of shape (m,) or (m, 1) -- constraint vector.
        num_iters: number of gradient descent iterations.
        lr: learning rate.
        penalty: penalty parameter to strongly enforce the constraints.
        tol: tolerance for determining if a constraint is violated.
        
        Returns:
        x: cp.array, approximate solution for x.
        xi: cp.array scalar, approximate optimal slack.
        violated_idx: cp.array, indices of constraints whose residual exceeds tol.
        residuals: cp.array of computed residuals, r = A x - b.
        """
        # Ensure b is a column vector (m, 1)
        b = cp.atleast_2d(b).reshape(-1, 1)
        m, n = A.shape

        # Initialize our decision variable x and slack xi
        x = cp.ones((n, 1))
        xi = cp.array([[1.0]])  # slack variable, shape (1, 1)

        # Perform subgradient descent
        for it in range(num_iters):
            # Compute constraint residuals (including slack): v = A x - b - xi
            v = cp.dot(A, x) - b - xi  # shape: (m, 1)
            
            # Only positive violations contribute to the penalty term.
            violation = cp.maximum(v, 0)  # shape: (m, 1)
            
            # (Optional) Compute the current objective value:
            f_val = xi + penalty * cp.sum(violation)
            # You could print f_val every so often for diagnostics.
            
            # Compute subgradients.
            # For x: subgrad_x = penalty * A^T * indicator(v > 0)
            indicator = (v > 0).astype(cp.float32)  # shape: (m, 1)
            subgrad_x = penalty * cp.dot(A.T, indicator)  # shape: (n, 1)
            
            # For xi: subgrad_xi = 1 - penalty * sum(indicator)
            subgrad_xi = 1 - penalty * cp.sum(indicator)
            
            # Update x and xi using the subgradients.
            x = x - lr * subgrad_x
            xi = xi - lr * subgrad_xi
            # Enforce xi >= 0
            xi = cp.maximum(xi, 0)

        # Compute final residuals for the original constraints.
        residuals = cp.dot(A, x) - b  # shape: (m, 1)
        violated_idx = cp.where(residuals > tol)[0]
        
        if xi.item() > tol:
            print("Constraints may be infeasible. Final slack xi =", xi.item())
            if violated_idx.size > 0:
                print("Violated constraints and their residuals:")
                for idx in violated_idx.get():
                    print(f"Constraint {idx}: residual = {residuals[idx].item():.2e}")
        else:
            print("All constraints appear feasible within tolerance.")
        
        return x, xi, violated_idx, residuals


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
            if distance[k] < self.params.r_max:                                
                self.N += 1
                filtered_distance.append(distance[k])
                filtered_angle.append(angle[k]) 
        
        # Create value and distance to plant of all points
        self.Y = -1*cp.ones(self.N)
        self.NY = cp.ones(self.N)
        self.Dist = cp.array(filtered_distance).reshape((self.N,1))
        
        # Convert to x y cordinates Global frame
        x_lidar = cp.array(cp.array(filtered_distance) * cp.cos(cp.array(filtered_angle) + self.params.theta)+self.params.x).reshape((self.N, 1))
        y_lidar = cp.array(cp.array(filtered_distance) * cp.sin(cp.array(filtered_angle) + self.params.theta)+self.params.y).reshape((self.N, 1))
        
        # Convert to x y cordinates Local frame
        x_lidar = cp.array(cp.array(filtered_distance) * cp.cos(cp.array(filtered_angle))).reshape((self.N, 1))
        y_lidar = cp.array(cp.array(filtered_distance) * cp.sin(cp.array(filtered_angle))).reshape((self.N, 1))
        filtered_angle = cp.array(filtered_angle).reshape((self.N,1))
        self.Poe = cp.hstack((x_lidar,y_lidar)).reshape((self.N,2))


    def rbf_kernel(self, X1, X2, length_scale, sigma_f):
        """
        Computes the RBF (Radial Basis Function) kernel between X1 and X2.
        """
        
        sqdist = (cp.sum(X1**2, 1).reshape(-1, 1) + cp.sum(X2**2, 1)) - 2 * X1 @ X2.T  # distance between points in X1 and X2
        #print(X1.shape,X2.shape)
                                                                              # note the dimentions in the sums!
                                                                                      # all distances between pairs of points
        return sigma_f * cp.exp((-0.5/length_scale**2) * sqdist)                      # Same kernel as in paper

    def rbf_kernel_grad_input(X1, X2, length_scale, sigma_f):
        """
        Gradient of the RBF kernel w.r.t. X1.
        Returns array of shape (N, M, D), where grad[i, j] = ∂k(X1[i], X2[j]) / ∂X1[i]
        """
        # Compute squared distances (N, M)
        sqdist = cp.sum(X1**2, axis=1).reshape(-1, 1) + cp.sum(X2**2, axis=1) - 2 * X1 @ X2.T
        K = sigma_f * cp.exp(-0.5 * sqdist / length_scale**2)

        # (N, M, D): X2 - X1 for each pair
        diff = X2[cp.newaxis, :, :] - X1[:, cp.newaxis, :]  # shape (N, M, D)

        # Apply gradient formula
        grad = (K[:, :, cp.newaxis] / length_scale**2) * diff  # shape (N, M, D)
        return grad
    

    def cbf_function(self, x_test, X_train):
        """
        Computes the CBF
        """
        print(x_test.shape,X_train.shape)
        return (1 - 2 * x_test.T @ X_train @ self.NY)
        #return 1-2*(self.rbf_kernel(x_test, X_train, length_scale, sigma_f))
        #return  self.rbf_kernel(x_test, X_train, length_scale, sigma_f) @ alpha - safe_dist
      
    def dcbf_function(self, x_query, X_train, k_star, k_inv, length_scale):
        """
        Compute the derivitive of the cbf function
        """
        diff = x_query - X_train
        print(k_star.shape,diff.shape)
        grad =  - (1 / (length_scale**2)) * k_star.T * diff.T
        print(grad.shape)
        grad_h = (self.Y.T @ k_inv @ grad.T)
        #print(cp.vstack((grad_h, cp.zeros((2, grad_h.shape[1])))).shape)
        print(grad_h.shape)
        return cp.hstack((grad_h.reshape((2,1)), cp.zeros((2, 1)))).T

    def lf_cbf_function(self,dcbf):
        """
        Derivitive of the cbf function by the forced dynamics
        """
        f = self.f_full()
        return dcbf.T @ f
     
    def lg_cbf_function(self,dcbf):
        """
        Derivitive of the cbf function by the Icput dynamics
        """
        g = self.g_full()
        #print(dcbf.shape,g.shape)
        return dcbf.T @ g

    # Constraints/Cost
    def constraints_cost(self,u_ref,x,y,theta,v):
        #self.updateState(v,theta)
        self.params.x,self.params.y = x,y
        # Create variables for optimisation 
        self.u_ref = cp.array(u_ref)
        A = cp.empty((0,2), float)
        B = {}
        b = cp.empty((0,1),float)
        LfB = {}
        LgB = {}
        X_query = self.f_full()[:2,:].T
        K = self.rbf_kernel(self.Poe,self.Poe,self.length_scale,self.params.sigma_f)

        K_star = self.rbf_kernel(X_query,self.Poe,self.length_scale,self.params.sigma_f)

        k_inv = cp.linalg.inv(K)
        print(X_query.shape)
        #k_test = self.rbf_kernel(self.f_full().T,cp.hstack((self.Poe,cp.zeros((self.Poe.shape[0],2)))).T,self.length_scale,self.params.sigma_f)
        #print(k_test.shape)
        cbf = self.cbf_function(K_star.T,k_inv)
        dcbf = self.dcbf_function(x_query=X_query,k_star=K_star.T,X_train=self.Poe,k_inv=k_inv,length_scale=self.length_scale)

        ##TODO add theta of all points to dcbf function??? 
        b = self.lg_cbf_function(dcbf) 
        b = b @ self.u_ref 
        b = b.reshape((b.size,1)) 
        A = - (self.lf_cbf_function(dcbf) + cbf**3)
        A = cp.hstack((cp.zeros((A.shape[0],1)), A , cp.zeros((A.shape[0],1))))

        # umax constraints
        
        k = cp.hstack(([cp.eye(self.params.udim), cp.zeros((self.params.udim, 1))]))
        #print(k.shape,A.shape)
        A = cp.vstack((A,k))
        k = cp.array((self.params.u_max))
        b = cp.vstack((b.reshape((b.shape[0],1)),k.reshape((k.size,1))))
        
        # u_min constraints
        
        A = cp.vstack((A,cp.hstack((-cp.eye(self.params.udim), cp.zeros((self.params.udim, 1))))))
       
        k = cp.array((self.params.u_min))
       
        b = cp.vstack((b,-k.reshape((k.size,1))))
        weight_input = cp.eye(2)

        #H = cp.array(((1,0),(0,1)))
        H = cp.eye(3)
        H = cp.diag(cp.array([10.0, 1.0, 10.0]))
        
        f = (weight_input) @ (-self.u_ref).reshape(2,1)
        f = cp.vstack((f,self.params.weightslack))
                 
        #     # Optimal control icput
        try:

        #print(H.shape,f.shape,A.shape,b.shape)  
            x_feas, slack = self.check_constraints_feasibility((A), (b))
            x = solve_qp(P=cp.asnumpy(H), q=cp.asnumpy(f), G=cp.asnumpy(A), h=cp.asnumpy(b), solver="clarabel")
        #x = solve_qp(P=H, q=f, G=A, h=b, solver = "clarabel") 
        #print('x')
        #print(x)  
            self.u = x[0]
            #TODO update from imu data
            ##self.updateState(x[1],x[0])
            self.params.gamma = float(x[1])
            self.params.v = float(x[0])
            self.params.weightslack = float(x[2])
            #x = [1,1]
            return x
        except Exception as e:
            print('failed constraints')
            print(f"An error occurred: {e}")
            return [0,0]
        

        
