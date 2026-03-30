#import autograd.numpy as np
from dataclasses import dataclass
from qpsolvers import solve_qp
import cupy as cp
import torch
from qpth.qp import QPFunction
import time
import matplotlib.pyplot as plt
import matplotlib.lines as mlines

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
        self.params.sigma_f = 1.0
        
        pass
        self.length_scale = 2   # found from  loop demo
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
    
    def draw_detailed_turtlebot(self, ax, center=(0, 0), base_radius=1, wheel_width=0.1, wheel_height=0.5,
                                sensor_radius=0.15, caster_wheel_radius=0.1, orientation_deg=0):
        x, y = center  # Unpack the center coordinates
    
        # Draw the base (outer circle) with rotation
        #base_x, base_y = rotate(x, y, orientation_deg)
        #base = plt.Circle((base_x, base_y), base_radius, color='lightblue', fill=True)
        req_width = 0.2
        req_length = 0.35
        base = plt.Rectangle((-req_width/2, -req_length/2),req_width, req_length, color='black',) # type: ignore
        ax.add_patch(base)
    
        # Draw the rear left wheel with rotation
        left_wheel_x = x - req_width/2 - wheel_width
        left_wheel_y = y + req_length/2 * .8 - wheel_height / 2
        
        left_wheel = plt.Rectangle((left_wheel_x, left_wheel_y),
                                wheel_width, wheel_height, color='gray', angle=0)
        ax.add_patch(left_wheel)
    
        # Draw the rear right wheel with rotation
        right_wheel_x = x + req_width/2  #wheel_width / 2
        right_wheel_y = y + req_length/2 * 0.8 - wheel_height / 2
        #right_wheel_x_rot, right_wheel_y_rot = rotate(right_wheel_x - x, right_wheel_y - y, orientation_deg)
        right_wheel = plt.Rectangle((x + right_wheel_x, y + right_wheel_y),
                                    wheel_width, wheel_height, color='gray', angle=0)
        ax.add_patch(right_wheel)

        # Draw the front left wheel with rotation
        left_wheel_x = x - req_width/2 - wheel_width
        left_wheel_y = y - req_length/2 * .8 - wheel_height / 2
        
        front_left_wheel = plt.Rectangle((left_wheel_x, left_wheel_y),
                                wheel_width, wheel_height, color='gray', angle=0)
        ax.add_patch(front_left_wheel)
    
        # Draw the front wheel with rotation
        right_wheel_x = x + req_width/2  #wheel_width / 2
        right_wheel_y = y - req_length/2 * 0.8 - wheel_height / 2
        #right_wheel_x_rot, right_wheel_y_rot = rotate(right_wheel_x - x, right_wheel_y - y, orientation_deg)
        front_right_wheel = plt.Rectangle((x + right_wheel_x, y + right_wheel_y),
                                    wheel_width, wheel_height, color='gray', angle=0)
        ax.add_patch(front_right_wheel)
    
        # Draw a small lidar or camera sensor on top with rotation
        sensor = plt.Circle((x, y), sensor_radius, color='orange', fill=True)
        ax.add_patch(sensor)

    def vis_barrier(
        self, K, K_inv, training_data, Y, length_scale=0.001, sigma_f=10,
        grid_limits=((-2, 2), (-2, 2)), grid_resolution=500
    ):
        (x_min, x_max), (y_min, y_max) = grid_limits
        x_lin = cp.linspace(x_min, x_max, grid_resolution)
        y_lin = cp.linspace(y_min, y_max, grid_resolution)
        x_grid, y_grid = cp.meshgrid(x_lin, y_lin)
        grid_points = cp.column_stack((x_grid.ravel(), y_grid.ravel()))

        Y = -1 * cp.ones(self.distances.shape)
        #Y = -Y  # Shift labels so that far from obstacles the default is +1
        # Cross-kernel
        K = self.rbf_kernel(training_data, training_data, length_scale, sigma_f)
        K_star = self.rbf_kernel(grid_points, training_data, length_scale, sigma_f)
        # GP prediction
        alpha = cp.linalg.pinv(K) @ (Y - 1)  # Compute alpha for GP prediction
        cbf_values = self.cbf_function(K_star, K)
        cbf_values = (K_star @ alpha) + 1  # Mean prediction at grid points
        #cbf_values = cp.clip(cbf_values,-1,1)
        # SHIFT BACK: adding +1 => "safe" defaults to +1, obstacle region near –1

        #cbf_values = cp.clip(cbf_values, -1, 1)

        # Reshape for plotting
        cbf_grid = cbf_values.reshape((grid_resolution, grid_resolution))

        # Plot
        x_grid_np = cp.asnumpy(x_grid)
        y_grid_np = cp.asnumpy(y_grid)
        cbf_grid_np = cp.asnumpy(cbf_grid)
        training_np = cp.asnumpy(training_data)
        #np.save('points.npy', (training_np))
        fig, ax = plt.subplots()
        #plt.figure(figsize=(8, 6))
        contour = plt.contourf(x_grid_np, y_grid_np, cbf_grid_np, levels=50, cmap='winter')
        plt.colorbar(contour,label='CBF Value')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Visualized CBF Barriers (Safe ~ +1, Obstacles ~ –1)')
        plt.scatter(training_np[:, 0], training_np[:, 1], color='red', marker='x', label='Obstacle Lidar Pts')
        zero_level = plt.contour(x_grid_np, y_grid_np, cbf_grid_np, levels=[0], colors='black', linewidths=2)
        self.draw_detailed_turtlebot(ax, center=(0, 0), base_radius=.2, wheel_width=0.04, wheel_height=.1,
                        sensor_radius=0.03, caster_wheel_radius=0.02, orientation_deg=45)
        # === VECTOR FIELD OVERLAY (sparser grid for clarity) ===
        vec_res = 25
        x_vec = cp.linspace(x_min, x_max, vec_res)
        y_vec = cp.linspace(y_min, y_max, vec_res)
        x_vec_grid, y_vec_grid = cp.meshgrid(x_vec, y_vec)
        vec_points = cp.column_stack((x_vec_grid.ravel(), y_vec_grid.ravel()))

        U = cp.zeros(vec_points.shape[0])
        V = cp.zeros(vec_points.shape[0])

        for i, x_query in enumerate(vec_points):
            k_star_vec = self.rbf_kernel(x_query[cp.newaxis, :], training_data, length_scale, sigma_f).flatten()[:, cp.newaxis]
            grad = self.dcbf_function(x_query.T, training_data, k_star_vec, K_inv, length_scale)
            V[i] = grad[0]
            U[i] = grad[1]

        ax.quiver(
            cp.asnumpy(vec_points[:, 0]),
            cp.asnumpy(vec_points[:, 1]),
            cp.asnumpy(U),
            cp.asnumpy(V),
            angles='xy', scale_units='xy', scale=1, color='black', alpha=0.85
        )

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
        dy = cp.sin(self.params.theta + self.params.beta) * self.params.dt
        return cp.array([dx, dy, cp.array(0.0), cp.array(0.0)]).reshape((4, 1))*self.params.v
    
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
        theta = 0.0 #cp.asarray(self.params.theta)
        beta = 0.0 #cp.asarray(self.params.beta)
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
        x_lidar = cp.round(filtered_distance[::5] * cp.cos(filtered_angle[::5]).astype(cp.float32),5)
        y_lidar = cp.round(filtered_distance[::5] * cp.sin(filtered_angle[::5]).astype(cp.float32),5)
        self.distances = filtered_distance[::5]
        # Stack the computed coordinates into a 2-column matrix
        self.Poe = cp.column_stack((y_lidar,x_lidar))
         # Update the number of points
        self.N = x_lidar.size

        # Create associated arrays directly on the GPU
        self.Y = -1 * cp.ones(self.N)
        self.NY = cp.ones(self.N)

    def rbf_kernel(self, X1, X2, length_scale, sigma_f):
        """
        Computes the RBF (Radial Basis Function) kernel between X1 and X2.
        """
        sqdist = cp.sum(X1**2, 1).reshape(-1, 1) + cp.sum(X2**2, 1) - 2 * X1 @ X2.T  # distance between points in X1 and X2
                                                                                    # note the dimentions in the sums!
                                                                                    # This is to create a matrix containing
                                                                                    # all distances between pairs of points
        return sigma_f**2 * cp.exp(-0.5 / length_scale**2 * sqdist)  # Same kernel as in paper
    

    def cbf_function(self, K_star, k_inv):
        """
        Computes the CBF
        """
        return  (K_star @ (k_inv @ (self.Y - 1 ))) + 1.0
      
    def dcbf_function(self, x_query, X_train, k_star, k_inv, length_scale):
        """
        Compute the derivitive of the cbf function
        """
        diff = x_query - X_train
        
        grad =  - (1 / (length_scale**2)) * k_star.T * diff.T
        
        grad_h = (self.Y.T @ k_inv @ grad.T)

        return cp.vstack((grad_h.reshape((2,1)), cp.zeros((2, 1))))

    def lf_cbf_function(self,dcbf):
        """
        Derivitive of the cbf function by the forced dynamics
        """
        f = self.f_full()
        return(f.T @ dcbf)
        return dcbf.T @ f
     
    def lg_cbf_function(self,dcbf):
        """
        Derivitive of the cbf function by the Icput dynamics
        """
        g = self.g_full()
        return(g.T @ dcbf)
        return dcbf.T @ g

    def constraints_cost(self, u_ref, x, y, theta, v, alpha=None):
        if alpha is None:
            alpha = 5.0

        # ----------------------------
        # Update robot state
        # ----------------------------
        self.updateState(x, y, v, 0)

        u_ref = cp.array(u_ref).reshape(2,1)

        # ----------------------------
        # Query robot position
        # ----------------------------
        X_query = cp.array([[0.0,0.0]])

        # ----------------------------
        # Kernel matrices
        # ----------------------------
        K = self.rbf_kernel(self.Poe, self.Poe, self.length_scale, self.params.sigma_f)
        K_inv = cp.linalg.inv(K)

        K_star = self.rbf_kernel(X_query, self.Poe, self.length_scale, self.params.sigma_f)

        # ----------------------------
        # Barrier value
        # ----------------------------
        h = self.cbf_function(K_star, K_inv).reshape(1,1)

        # ----------------------------
        # Barrier gradient
        # ----------------------------
        dcbf = self.dcbf_function(
            x_query=X_query,
            X_train=self.Poe,
            k_star=K_star.T,
            k_inv=K_inv,
            length_scale=self.length_scale
        )

        # ----------------------------
        # Lie derivatives
        # ----------------------------
        Lf_h = self.lf_cbf_function(dcbf).reshape(1,1)
        Lg_h = self.lg_cbf_function(dcbf).reshape(1,2)


        # ----------------------------
        # CBF constraint
        # Lf h + Lg h u + αh ≥ -δ
        # convert to Gx ≤ h
        # ----------------------------

        A_cbf = -Lg_h
        slack_cbf = cp.array([[1.0]])

        G_cbf = cp.hstack([A_cbf, slack_cbf])

        b_cbf = (Lf_h + alpha*h)

        # ----------------------------
        # Input limits
        # ----------------------------

        G_input = cp.vstack([
            cp.hstack([ cp.eye(2), cp.zeros((2,1)) ]),
            cp.hstack([ -cp.eye(2), cp.zeros((2,1)) ])
        ])

        h_input = cp.vstack([
            cp.array(self.params.u_max).reshape(2,1),
            -cp.array(self.params.u_min).reshape(2,1)
        ])

        # ----------------------------
        # Combine constraints
        # ----------------------------

        G = cp.vstack([
            G_cbf,
            G_input
        ])

        h_vec = cp.vstack([
            b_cbf,
            h_input
        ])

        # ----------------------------
        # Cost function
        # ----------------------------

        W = cp.diag(cp.array([50.0,1.0]))

        H = cp.zeros((3,3))
        H[:2,:2] = W
        H[2,2] = 1000.0   # slack penalty

        f = cp.vstack([
            -W @ u_ref,
            cp.array([[0.0]])
        ])

        # ----------------------------
        # Solve QP
        # ----------------------------

        try:

            sol = solve_qp(
                P = cp.asnumpy(H),
                q = cp.asnumpy(f).flatten(),
                G = cp.asnumpy(G),
                h = cp.asnumpy(h_vec).flatten(),
                solver="clarabel"
            )

            if sol is None:
                raise ValueError("QP returned None")

            u = sol[:2]

            u[1] = -u[1]

            self.params.v = float(u[0])
            self.params.gamma = float(u[1])

            return u, self.f_full()

        except Exception as e:

            print("QP failed:", e)

            return [0,0], self.f_full()