#import autograd.numpy as np
from dataclasses import dataclass
from qpsolvers import solve_qp
import cupy as cp
import torch
from qpth.qp import QPFunction
import numpy as np
import time
import matplotlib.pyplot as plt
import numpy as np

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
        self.params.sigma_f = 1.0*10**1
        
        pass
        self.length_scale = 0.05    # found from  loop demo
        self.time = 0.0
    # Dynamics #

    def f(self):
        """
        The forced dynamics of the car bike model
        """
        x = cp.array([cp.array(1, dtype=cp.float32),cp.array(0, dtype=cp.float32), -self.params.v * cp.sin(self.params.theta + self.params.beta)*self.params.dt],dtype=cp.float32)
        y = cp.array([cp.array(0, dtype=cp.float32),cp.array(1, dtype=cp.float32), self.params.v * cp.cos(self.params.theta + self.params.beta)*self.params.dt])
        t = cp.array([0.0, 0.0 , 1.0])
        return cp.vstack((x,y,t))
    
    def vis_barrier(
        self, K, K_inv, training_data, Y, length_scale=0.001, sigma_f=10,
        grid_limits=((-2, 2), (-2, 2)), grid_resolution=500
    ):
        (x_min, x_max), (y_min, y_max) = grid_limits
        x_lin = cp.linspace(x_min, x_max, grid_resolution)
        y_lin = cp.linspace(y_min, y_max, grid_resolution)
        x_grid, y_grid = cp.meshgrid(x_lin, y_lin)
        grid_points = cp.column_stack((x_grid.ravel(), y_grid.ravel()))

        #shifted_Y = -1 * np.ones(self.distances.shape)
        Y = Y - 1.0  # Shift labels so that far from obstacles the default is +1
        # Cross-kernel
        K_star = self.rbf_kernel(grid_points, training_data, length_scale, sigma_f)
        # GP prediction
        mean_pred = cp.dot(K_star, cp.dot(K_inv, Y))

        # SHIFT BACK: adding +1 => "safe" defaults to +1, obstacle region near –1
        cbf_values = mean_pred + 1
        #cbf_values = cp.clip(cbf_values, -1, 1)

        # Reshape for plotting
        cbf_grid = cbf_values.reshape((grid_resolution, grid_resolution))

        # Plot
        x_grid_np = cp.asnumpy(x_grid)
        y_grid_np = cp.asnumpy(y_grid)
        cbf_grid_np = cp.asnumpy(cbf_grid)
        training_np = cp.asnumpy(training_data)
        np.save('points.npy', (training_np))

        plt.figure(figsize=(8, 6))
        contour = plt.contourf(x_grid_np, y_grid_np, cbf_grid_np, levels=50, cmap='winter')
        plt.colorbar(contour,label='CBF Value')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Visualized CBF Barriers (Safe ~ +1, Obstacles ~ –1)')
        plt.scatter(training_np[:, 0], training_np[:, 1], color='red', marker='x', label='Obstacle Lidar Pts')
        zero_level = plt.contour(x_grid_np, y_grid_np, cbf_grid_np, levels=[0], colors='black', linewidths=2)
        plt.legend()
        plt.show()

    def vis_barrier_and_dcbf_origin(self, training_data, Y, length_scale=0.001, sigma_f=10,
                                    grid_limits=((-2, 2), (-2, 2)), grid_resolution=200):
        """
        Visualizes the barrier function and overlays the gradient (dCBF) at the origin as an arrow.
        
        Barrier:
          - Computes the GP-based barrier function on a 2D grid.
          - Uses shifted training labels so that far from obstacles the default is +1.
          - Clips values between -1 and +1.
          - Plots the barrier with a filled contour plot along with the training (lidar) points and
            a black contour line at the 0-level set.
        
        dCBF at Origin:
          - Computes the dCBF (gradient) at the origin using the same GP parameters.
          - Overlays an arrow at (0,0) showing the gradient direction and magnitude.
        
        Parameters
        ----------
        training_data : cp.ndarray
            CuPy array of shape (N, 2) containing the training (lidar) points.
        Y : cp.ndarray
            CuPy array of target barrier values (e.g., -1 for obstacles); shape can be (N,) or (N,1).
        length_scale : float, optional
            RBF kernel length scale. Default is 0.001.
        sigma_f : float, optional
            Signal variance of the RBF kernel. Default is 10.
        grid_limits : tuple, optional
            ((x_min, x_max), (y_min, y_max)) limits for the grid. Default is ((-2, 2), (-2, 2)).
        grid_resolution : int, optional
            Number of grid points along each axis for the barrier visualization. Default is 100.
        """
        # -------------------------
        # Barrier Visualization Part:
        # -------------------------
        (x_min, x_max), (y_min, y_max) = grid_limits
        x_lin = cp.linspace(x_min, x_max, grid_resolution)
        y_lin = cp.linspace(y_min, y_max, grid_resolution)
        x_grid, y_grid = cp.meshgrid(x_lin, y_lin)
        grid_points = cp.column_stack((x_grid.ravel(), y_grid.ravel()))
        
        # Compute the kernel matrix for training data and its inverse.
        K = self.rbf_kernel(training_data, training_data, length_scale, sigma_f)
        K_inv = cp.linalg.inv(K)
        
        # Shift training labels: if obstacles are -1 then shifted_Y = Y - 1 gives -2 (forcing default far away to +1).
        shifted_Y = Y - 1.0
        
        # Compute cross-kernel between grid points and training data.
        K_star = self.rbf_kernel(grid_points, training_data, length_scale, sigma_f)
        # Gaussian Process prediction (assuming zero prior mean)
        mean_pred = cp.dot(K_star, cp.dot(K_inv, shifted_Y))
        # Shift back by adding 1 so that away from obstacles we approach +1.
        cbf_values = 1.0 + mean_pred
        # Clip the barrier values between -1 and +1.
        cbf_values = cp.clip(cbf_values, -1, 1)
        # Reshape into grid for plotting.
        cbf_grid = cbf_values.reshape((grid_resolution, grid_resolution))
        
        # -------------------------
        # dCBF at the Origin Part:
        # -------------------------
        # Define the query point as the origin.
        X_query = cp.array([[0.0, 0.0]])  # shape (1, 2)
        # Compute GP weights: w = K_inv * (Y - 1)
        w = cp.dot(K_inv, shifted_Y)  # shape (N, 1)
        w_flat = cp.ravel(w)          # shape (N,)
        
        # Compute differences between the origin and each training point.
        diff = X_query[:, None, :] - training_data[None, :, :]  # shape (1, N, 2)
        # Compute kernel values between the origin and each training point.
        k_mat = self.rbf_kernel(X_query, training_data, length_scale, sigma_f)  # shape (1, N)
        # Calculate the gradient at the origin.
        gradient_origin = - cp.sum(diff * k_mat[..., None] * w_flat[None, :, None], axis=1) / (length_scale**2)
        # Convert gradient to a NumPy array.
        gradient_origin_np = cp.asnumpy(gradient_origin)[0]
        
        # -------------------------
        # Plotting both the barrier and the dCBF arrow
        # -------------------------
        # Convert grid arrays from CuPy to NumPy.
        x_grid_np = cp.asnumpy(x_grid)
        y_grid_np = cp.asnumpy(y_grid)
        cbf_grid_np = cp.asnumpy(cbf_grid)
        training_np = cp.asnumpy(training_data)
        
        plt.figure(figsize=(8, 6))
        # Plot filled contour of the barrier function.
        contour = plt.contourf(x_grid_np, y_grid_np, cbf_grid_np, levels=50, cmap='viridis', vmin=-1, vmax=1)
        plt.colorbar(contour, label='CBF Value')
        # Add a black contour line at the 0 level set.
        plt.contour(x_grid_np, y_grid_np, cbf_grid_np, levels=[0], colors='black', linewidths=2)
        # Plot the training points.
        plt.scatter(training_np[:, 0], training_np[:, 1], color='red', marker='x', label='Obstacle Lidar Pts')
        
        # Overlay the dCBF arrow at the origin.
        # Choose a suitable scaling factor (scale=1 here means no automatic scaling; adjust if needed).
        plt.quiver(0, 0, gradient_origin_np[0], gradient_origin_np[1],
                   color='blue', angles='xy', scale_units='xy', scale=1, width=0.5)
        plt.scatter([0], [0], color='blue', s=50, label='Origin dCBF')
        
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Barrier Function with dCBF at the Origin')
        plt.legend()
        plt.show()


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
        x = cp.zeros((n, 1))
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
        Process all lidar points using vectorized operations.
        Filters out points beyond the maximum range and computes the x-y coordinates.

        Parameters:
        distance: Iterable or array of distances
        angle:    Iterable or array of angles (in radians)
        """
        # Convert inputs to CuPy arrays (if they're not already)
        if not isinstance(distance, cp.ndarray):
            distance = cp.asarray(distance)
        if not isinstance(angle, cp.ndarray):
            angle = cp.asarray(angle)

        # Create a boolean mask for points within the max range
        mask_range = distance < self.params.r_max
        filtered_distance = distance[mask_range]
        filtered_angle    = angle[mask_range]
        mask_angles = (filtered_angle >= -cp.pi) & (filtered_angle <= cp.pi)
        filtered_distance = filtered_distance[mask_angles]
        filtered_angle    = filtered_angle[mask_angles]
        
        
        
        # Apply the mask to filter distances and angles
        # filtered_distance = distance[mask_range]
        # filtered_angle = angle[mask_range]

       

        # Compute local coordinates (you can also compute global if needed)
        # Local coordinates:
        # x_lidar = filtered_distance * cp.cos(filtered_angle)
        # y_lidar = filtered_distance * cp.sin(filtered_angle)

        # Every other
        x_lidar = cp.round(filtered_distance[::2] * cp.cos(filtered_angle[::2]).astype(cp.float32),5)
        y_lidar = cp.round(filtered_distance[::2] * cp.sin(filtered_angle[::2]).astype(cp.float32),5)
        self.distances = filtered_distance[::2]
        # Stack the computed coordinates into a 2-column matrix
        self.Poe = cp.column_stack((-y_lidar, x_lidar))
         # Update the number of points
        self.N = x_lidar.size

        # Create associated arrays directly on the GPU
        self.Y = -1 * cp.ones(self.N)
        self.NY = cp.ones(self.N)


    # def setObjects(self,distance,angle):                               
    #     """
    #     Take all lidar points and turn them into data
    #     """
    #     M = len(distance)   # Total Number of possible lidar data points
    #     self.N = 0          # Total number of points in range

    #     # Instantiate matrix
    #     filtered_distance = []
    #     filtered_angle = []

    #     for k in range(M):
    #     # Keep only points within max lidar field 
    #         if distance[k] < self.params.r_max:                                
    #             self.N += 1
    #             filtered_distance.append(distance[k])
    #             filtered_angle.append(angle[k]) 
        
    #     # Create value and distance to plant of all points
    #     self.Y = -1*cp.ones(self.N)
    #     self.NY = cp.ones(self.N)
    #     #self.Dist = cp.array(filtered_distance).reshape((self.N,1))
        
    #     # Convert to x y cordinates Global frame
    #     #x_lidar = cp.array(cp.array(filtered_distance) * cp.cos(cp.array(filtered_angle) + self.params.theta)+self.params.x).reshape((self.N, 1))
    #     #y_lidar = cp.array(cp.array(filtered_distance) * cp.sin(cp.array(filtered_angle) + self.params.theta)+self.params.y).reshape((self.N, 1))
        
    #     # Convert to x y cordinates Local frame
    #     x_lidar = cp.array(cp.array(filtered_distance) * cp.cos(cp.array(filtered_angle))).reshape((self.N, 1))
    #     y_lidar = cp.array(cp.array(filtered_distance) * cp.sin(cp.array(filtered_angle))).reshape((self.N, 1))
    #     #filtered_angle = cp.array(filtered_angle).reshape((self.N,1))
    #     self.Poe = cp.hstack((x_lidar,y_lidar)).reshape((self.N,2))

   

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
    

    def cbf_function(self, K_star, k_inv):
        """
        Computes the CBF
        """
        #print(x_test.shape,X_train.shape)
        return  1 - 2 * (K_star.T @ k_inv @ (self.Y))
        #return (1 - 2 * x_test.T @ X_train @ self.NY)
        #return 1-2*(self.rbf_kernel(x_test, X_train, length_scale, sigma_f))
        #return  self.rbf_kernel(x_test, X_train, length_scale, sigma_f) @ alpha - safe_dist
      
    def dcbf_function(self, x_query, X_train, k_star, k_inv, length_scale):
        """
        Compute the derivitive of the cbf function
        """
        diff = x_query - X_train
        #print(k_star.shape,diff.shape)
        grad =  - (1 / (length_scale**2)) * k_star.T * diff.T
        #print(grad.shape)
        grad_h = (self.Y.T @ k_inv @ grad.T)
        #print(cp.vstack((grad_h, cp.zeros((2, grad_h.shape[1])))).shape)
        #print(grad_h.shape)
        return cp.vstack((grad_h.reshape((2,1)), cp.zeros((2, 1))))

    def lf_cbf_function(self,dcbf):
        """
        Derivitive of the cbf function by the forced dynamics
        """
        f = self.f_full()
        print('dcbf f')
        print(dcbf.T @ f)
        return dcbf.T @ f
     
    def lg_cbf_function(self,dcbf):
        """
        Derivitive of the cbf function by the Icput dynamics
        """
        g = self.g_full()
        print('dcbf g')
        print(dcbf.T @ g)
        return dcbf.T @ g

    # Constraints/Cost
    def constraints_cost(self,u_ref,x,y,theta,v):
        #self.updateState(v,theta)
        
        #print(self.time - time.time())
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
        #print(self.N)
        K_star = self.rbf_kernel(X_query,self.Poe,self.length_scale,self.params.sigma_f)
        #tim = time.time()
        k_inv = cp.linalg.inv(K)
        #print('inverse time')
        
        #print(X_query.shape)
        #k_test = self.rbf_kernel(self.f_full().T,cp.hstack((self.Poe,cp.zeros((self.Poe.shape[0],2)))).T,self.length_scale,self.params.sigma_f)
        #print(k_test.shape)
        cbf = self.cbf_function(K_star.T,k_inv)
        print(max(abs(cbf)))
        cbf = cp.clip(cbf, -1, 1)
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
        weight_input = cp.diag(cp.array([1.0, 1.0]))
        print(A.shape,b.shape)
        # H = cp.eye(3)
        H = cp.diag(cp.array([1.0, 1.0*10**-6, 1.0]))
        
        f = (weight_input) @ (-self.u_ref).reshape(2,1)
        f = cp.vstack((f,self.params.weightslack))
        self.vis_barrier(K=K,K_inv=k_inv,training_data=self.Poe, Y = self.Y, length_scale=self.length_scale*2, sigma_f=1, 
                            grid_limits=((-2, 2), (-2, 2)), grid_resolution=400)
      
        #self.vis_barrier_and_dcbf_origin(training_data=self.Poe, Y=self.Y, length_scale=self.length_scale,grid_resolution=300, sigma_f=1)
        try:

            x = solve_qp(P=cp.asnumpy(H), q=cp.asnumpy(f), G=cp.asnumpy(A), h=cp.asnumpy(b), solver="clarabel") 
            print(x)
            self.u = x[0]
            self.params.gamma = float(x[1])
            self.params.v = float(x[0])
            self.params.weightslack = float(x[2])
            return x,self.f_full()
        except Exception as e:
            #print('failed constraints')
            print(f"An error occurred: {e}")
            return [0,0],self.f_full()