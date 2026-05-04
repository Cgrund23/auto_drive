"""
Model-Free Control Barrier Function Implementation
Based on: "Safety via Control Barrier Functions Synthesized from Ultra-Local Models"

Implements GP-based barrier learning and HOCBF constraints using estimated ULM parameters.
"""
import cupy as cp
import numpy as np
from qpsolvers import solve_qp


class ModelFreeCBF:
    """
    Model-Free CBF using GP barriers and MIMO ULM dynamics estimation.
    """

    def __init__(self, dt, u_min, u_max, r_max, r_min_obstacle, length_scale, sigma_f, lambda_0, lambda_1, c_q):
        """
        Args:
            dt: Sampling period
            u_min: Minimum control input [v_min, ω_min]
            u_max: Maximum control input [v_max, ω_max]
            r_max: Maximum LiDAR range
            r_min_obstacle: Only consider obstacles closer than this
            length_scale: GP kernel length scale
            sigma_f: GP signal variance
            lambda_0, lambda_1: HOCBF parameters (Section II-C of paper)
            c_q: Confidence quantile for safety margin (e.g., 2.0 for 2-sigma)
        """
        self.dt = dt
        self.u_min = cp.array(u_min)
        self.u_max = cp.array(u_max)
        self.r_max = r_max
        self.r_min_obstacle = r_min_obstacle
        self.length_scale = length_scale
        self.sigma_f = sigma_f
        self.lambda_0 = lambda_0
        self.lambda_1 = lambda_1
        self.c_q = c_q

        # Obstacle points (set by set_obstacles)
        self.obstacle_points = None
        self.N = 0

        # Cache for GP computations (speeds up repeated queries)
        self._K_inv_cache = None
        self._alpha_cache = None

    def set_obstacles(self, ranges, angles):
        """
        Process LiDAR data to extract obstacle points in robot frame.

        Args:
            ranges: CuPy array of distances
            angles: CuPy array of angles (radians)
        """
        ranges = cp.asarray(ranges)
        angles = cp.asarray(angles)

        # Filter by range - only consider close obstacles!
        mask = (ranges < self.r_min_obstacle) & (ranges > 0.1)
        filtered_ranges = ranges[mask]
        filtered_angles = angles[mask]

        # Convert to Cartesian (robot frame, x forward, y left)
        # Note: Adjust sign based on your coordinate convention
        x = filtered_ranges * cp.cos(filtered_angles)
        y = filtered_ranges * cp.sin(filtered_angles)

        # Downsample for efficiency (every 5th point)
        x = x[::5]
        y = y[::5]

        self.obstacle_points = cp.column_stack((x, -y))  # Adjust y sign if needed
        self.N = len(self.obstacle_points)

        # Precompute and cache K_inv for this obstacle set (major speedup!)
        if self.N > 0:
            Y = -cp.ones((self.N, 1))
            K = self.rbf_kernel(self.obstacle_points, self.obstacle_points)
            self._K_inv_cache = cp.linalg.inv(K + 1e-6 * cp.eye(self.N))
            self._alpha_cache = self._K_inv_cache @ (Y - 1.0)
        else:
            self._K_inv_cache = None
            self._alpha_cache = None

    def rbf_kernel(self, X1, X2):
        """
        RBF kernel: k(x, x') = σ_f² exp(-||x - x'||² / (2 ℓ²))

        Args:
            X1: (N1, d) array
            X2: (N2, d) array

        Returns:
            K: (N1, N2) kernel matrix
        """
        sqdist = cp.sum(X1**2, axis=1, keepdims=True) + \
                 cp.sum(X2**2, axis=1) - 2 * (X1 @ X2.T)
        return self.sigma_f**2 * cp.exp(-0.5 * sqdist / self.length_scale**2)

    def get_barrier_and_variance(self, p):
        """
        Compute GP posterior mean and variance for barrier at point p.

        h(p) = 1 + k(p, P)^T K^{-1} (Y - 1)

        Where Y = -ones (obstacles labeled as -1), and we shift to make
        default value (far from obstacles) equal to +1.

        Args:
            p: Query point [x, y] (list or array)

        Returns:
            h: Barrier value (scalar)
            sigma_sq: GP posterior variance (scalar)
        """
        if self.N == 0:
            return 1.0, 0.0  # No obstacles

        p = cp.array(p).reshape(1, 2)

        # Use cached K_inv and alpha (huge speedup!)
        k_star = self.rbf_kernel(p, self.obstacle_points)  # (1, N)

        # GP posterior mean (shifted) using cached alpha
        h = 1.0 + float(k_star @ self._alpha_cache)

        # GP posterior variance (Eq. 6 in paper)
        k_ss = self.rbf_kernel(p, p)[0, 0]
        sigma_sq = float(k_ss - k_star @ self._K_inv_cache @ k_star.T)

        return h, sigma_sq

    def get_gradient(self, p):
        """
        Compute gradient of GP barrier: ∇h(p)

        From Eq. (5) in paper:
        ∇h(p) = Σ_j α_j k(p, p_j) (p_j - p) / ℓ²

        where α = K^{-1}(Y - 1)

        Args:
            p: Query point [x, y]

        Returns:
            grad_h: Gradient [∂h/∂x, ∂h/∂y] (2D CuPy array)
        """
        if self.N == 0:
            return cp.zeros(2)

        p = cp.array(p).reshape(1, 2)

        # Use cached alpha (huge speedup!)
        # k(p, p_j) for all j
        k_star = self.rbf_kernel(p, self.obstacle_points)  # (1, N)

        # Differences: (p_j - p) for all j
        diff = self.obstacle_points - p  # (N, 2)

        # Gradient: Σ_j α_j k(p, p_j) (p_j - p) / ℓ²
        grad_h = (k_star.T * self._alpha_cache).T @ diff / self.length_scale**2  # (1, 2)

        return grad_h.flatten()

    def compute_safety_margin(self, P, u_max):
        """
        Compute safety margin σ_k from EKF covariance.

        From Eq. (13) in paper:
        σ̄²_{η,k} = max_{u ∈ U} ℓ^T_{η,k}(u) P^q_k ℓ_{η,k}(u)

        where ℓ_{η,k}(u) = [λ_0 λ_1, λ_0+λ_1, 1, u^T]^T

        Args:
            P: EKF covariance matrix (CuPy array)
            u_max: Maximum control input (for worst-case margin)

        Returns:
            sigma: Safety margin c_q * σ̄_{η,k}
        """
        # Evaluate at worst case: u = u_max
        u_wc = cp.asarray(self.u_max)
        P = cp.asarray(P)

        ell = cp.array([
            self.lambda_0 * self.lambda_1,
            self.lambda_0 + self.lambda_1,
            1.0,
            float(u_wc[0]),
            float(u_wc[1])
        ])

        sigma_sq = float(ell @ P @ ell)
        sigma_bar = float(cp.sqrt(cp.maximum(sigma_sq, 0.0)))

        # Clamp safety margin to prevent infeasibility during EKF convergence
        sigma = float(self.c_q * sigma_bar)
        sigma_max = 0.05  # Maximum safety margin - very small to ensure feasibility
        return min(sigma, sigma_max)

    def compute_safe_control(self, u_ref, q_hat, qdot_hat, F_q_hat, B_q_hat, P):
        """
        Solve CLF-CBF QP to compute safe control.

        From Eq. (9) in paper:
        B̂_q,k u ≥ -F̂_q,k - (λ_0 + λ_1)q̇̂_k - λ_0 λ_1 q̂_k + σ_k

        QP formulation:
            minimize    ||u - u_ref||²
            subject to  B̂_q,k u ≥ r_k  (CBF constraint)
                        u_min ≤ u ≤ u_max  (input bounds)

        Args:
            u_ref: Reference control [v_ref, ω_ref]
            q_hat: Estimated barrier value
            qdot_hat: Estimated barrier derivative
            F_q_hat: Estimated lumped disturbance
            B_q_hat: Estimated input sensitivity [B_v, B_ω]
            P: EKF covariance matrix

        Returns:
            u_safe: Safe control input [v, ω]
        """
        # Ensure inputs are CuPy arrays
        u_ref = cp.asarray(u_ref)
        B_q_hat = cp.asarray(B_q_hat)

        # Compute safety margin
        sigma_k = self.compute_safety_margin(P, self.u_max)

        # HOCBF RHS (Eq. 10 in paper)
        r_k = -F_q_hat - (self.lambda_0 + self.lambda_1) * qdot_hat - \
              self.lambda_0 * self.lambda_1 * q_hat + sigma_k

        # QP formulation - explicitly convert CuPy to NumPy for QP solver
        # Cost: minimize (u - u_ref)^T W (u - u_ref)
        # Higher weight = more expensive to change
        # w_v >> w_omega means "prefer steering over braking"
        w_v = 10.0  # High cost for changing velocity (prefer to maintain speed)
        w_omega = 0.1  # Low cost for changing steering (prefer to steer)
        P_qp = np.diag([w_v, w_omega])

        # Convert to numpy arrays explicitly
        u_ref_np = np.array(u_ref.get(), dtype=np.float64)
        B_q_np = np.array(B_q_hat.get(), dtype=np.float64)
        u_max_np = np.array(self.u_max.get(), dtype=np.float64)
        u_min_np = np.array(self.u_min.get(), dtype=np.float64)

        q_qp = -P_qp @ u_ref_np  # Linear term: -W @ u_ref

        G_np = np.vstack([
            -B_q_np.reshape(1, 2),  # CBF constraint
            np.eye(2),              # u ≤ u_max
            -np.eye(2)              # -u ≤ -u_min
        ])

        h_np = np.array([
            float(-r_k),
            float(u_max_np[0]),
            float(u_max_np[1]),
            float(-u_min_np[0]),
            float(-u_min_np[1])
        ], dtype=np.float64)

        # Solve QP
        try:
            sol = solve_qp(
                P=P_qp,
                q=q_qp,
                G=G_np,
                h=h_np,
                solver='clarabel'
            )

            if sol is None:
                print(f'\n=== QP INFEASIBLE ===')
                print(f'CBF constraint: -B_q @ u <= {float(-r_k):.4f}')
                print(f'  B_q = {B_q_np}')
                print(f'  Breakdown: -F_q={-F_q_hat:.3f}, -(λ0+λ1)q̇={-(self.lambda_0+self.lambda_1)*qdot_hat:.3f}, -λ0λ1q={-self.lambda_0*self.lambda_1*q_hat:.3f}, σ={sigma_k:.3f}')
                print(f'State: q={q_hat:.3f}, q̇={qdot_hat:.3f}')
                print(f'ULM: F_q={F_q_hat:.3f}, B_q={B_q_np}')
                print(f'Control bounds: v∈[{u_min_np[0]:.2f}, {u_max_np[0]:.2f}], ω∈[{u_min_np[1]:.2f}, {u_max_np[1]:.2f}]')
                print(f'Max achievable: B_q @ u_max = {float(B_q_np[0]*u_max_np[0] + B_q_np[1]*u_max_np[1]):.3f}')
                print(f'=====================\n')

                # Return safe fallback
                return [float(u_min_np[0]), 0.0]

            return [float(sol[0]), float(sol[1])]

        except Exception as e:
            print(f'QP solve exception: {e}')
            print(f'  q_hat={q_hat:.3f}, qdot_hat={qdot_hat:.3f}')
            print(f'  F_q_hat={F_q_hat:.3f}, B_q_hat={B_q_np}')
            print(f'  r_k={float(r_k):.3f}, sigma_k={sigma_k:.3f}')
            # Return safe fallback
            return [float(u_min_np[0]), 0.0]

    def check_feasibility(self, q_hat, qdot_hat, F_q_hat, B_q_hat, sigma_k):
        """
        Check if CBF QP is feasible (Lemma 1 in paper).

        Feasibility condition:
        M_k ≥ r_k

        where M_k = Σ_{j: [a_k]_j ≥ 0} [a_k]_j [u_max]_j + Σ_{j: [a_k]_j < 0} [a_k]_j [u_min]_j
        and a_k = B̂_q,k

        Args:
            q_hat, qdot_hat, F_q_hat, B_q_hat: EKF estimates
            sigma_k: Safety margin

        Returns:
            feasible: Boolean
        """
        B_q_hat = cp.array(B_q_hat)
        r_k = -F_q_hat - (self.lambda_0 + self.lambda_1) * qdot_hat - \
              self.lambda_0 * self.lambda_1 * q_hat + sigma_k

        M_k = 0.0
        for j in range(len(B_q_hat)):
            if float(B_q_hat[j]) >= 0:
                M_k += float(B_q_hat[j]) * float(self.u_max[j])
            else:
                M_k += float(B_q_hat[j]) * float(self.u_min[j])

        return float(M_k) >= float(r_k)
