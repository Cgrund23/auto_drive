"""
CBF_refactored_ackermann.py

Ackermann-native replacement for CBF_refactored.py.

GROUND TRUTH: L.A. Duffaut Espinosa & C. Grund, "Safety via Control Barrier
Functions Synthesized from Ultra-Local Models" ("the paper"). Every equation
cited below refers to that paper.

WHAT CHANGED FROM THE ORIGINAL CBF_refactored.py AND WHY
----------------------------------------------------------------------------
The original file's ModelFreeCBF operated on u = [v, omega] (differential-
drive-style angular RATE) and the ROS node then converted omega to a
steering angle at the very last step via steering_angle = atan(L*omega/v)
before publishing AckermannDriveStamped. That conversion is:
  (a) singular as v -> 0 (the node special-cased this with an ad hoc
      "steering_angle = omega * 0.33" fallback),
  (b) dependent on the wheelbase L exactly where the whole point of the
      paper's approach is to avoid needing kinematic constants in the
      safety-critical path, and
  (c) lossy -- the ULM/EKF was learning the sensitivity of the barrier to
      omega, a quantity the vehicle cannot actually command; only the
      *derived* steering_angle is ever sent to the VESC.

This file instead builds the barrier/QP machinery directly on
u = [v, phi] where phi IS the physical steering angle
(ackermann_msgs/AckermannDriveStamped.drive.steering_angle), per the paper's
own Ackermann kinematics (Section II-A):
    m = 2, u = [v, phi]^T, wheelbase L,
    p_dot = v [cos(theta), sin(theta)]^T,  theta_dot = (v/L) tan(phi).
Since MFC/ULM estimates F_q, B_q online from data regardless of what u
physically means (that is the entire premise of the paper -- "bypassing all
Lie-derivative computations"), nothing about the estimator math needs to
change; only the semantics of the second control channel do. The result is
that cbf_Node_refactored_ackermann.py publishes B_q,phi's own commanded
value directly, with no L-dependent conversion at the output stage at all.

Validated in simulation against ackermann_cbf_core.py / run_ackermann_sim.py
(same repo) before being ported here -- see that simulation's printed
verification report (Lemma 1 feasibility, Theorem 1/2 barrier maintenance,
Theorem 3 Lyapunov bound) and the explicit left/right avoidance check.
"""
import numpy as np


def rbf_kernel(X1, X2, sigma_f, ell):
    """Squared-exponential kernel, Eq. (4)-(6)."""
    X1 = np.atleast_2d(X1)
    X2 = np.atleast_2d(X2)
    sqdist = (np.sum(X1**2, axis=1, keepdims=True)
              + np.sum(X2**2, axis=1) - 2 * (X1 @ X2.T))
    sqdist = np.clip(sqdist, 0, None)
    return sigma_f**2 * np.exp(-0.5 * sqdist / ell**2)


class ObstacleGP:
    """Per-obstacle GP barrier, Eq. (4)-(6), Section II-E training-target
    convention (m0 prior far from data, y=0 exactly at buffered boundary)."""

    def __init__(self, length_scale=0.30, sigma_f=1.0, noise_var=5e-3,
                 prior_mean=1.0, max_points=30):
        self.ell = length_scale
        self.sigma_f = sigma_f
        self.noise_var = noise_var
        self.m0 = prior_mean
        self.max_points = max_points
        self.P = np.zeros((0, 2))
        self._K_inv = None
        self._alpha = None

    def add_points(self, pts):
        pts = np.atleast_2d(np.asarray(pts, dtype=float))
        if pts.size == 0:
            return
        if self.P.shape[0] == 0:
            merged = pts
        else:
            keep = []
            for p in pts:
                d = np.min(np.linalg.norm(self.P - p, axis=1))
                if d > 0.03:
                    keep.append(p)
            merged = np.vstack([self.P] + [np.array(keep)]) if keep else self.P
        if merged.shape[0] > self.max_points:
            merged = merged[-self.max_points:]
        self.P = merged
        self._refit()

    def _refit(self):
        N = self.P.shape[0]
        if N == 0:
            self._K_inv, self._alpha = None, None
            return
        K = rbf_kernel(self.P, self.P, self.sigma_f, self.ell) + self.noise_var * np.eye(N)
        self._K_inv = np.linalg.inv(K)
        Y = np.zeros((N, 1))
        self._alpha = self._K_inv @ (Y - self.m0)

    @property
    def n_points(self):
        return self.P.shape[0]

    def posterior_mean(self, p):
        if self.n_points == 0:
            return self.m0
        p = np.atleast_2d(p)
        k_star = rbf_kernel(p, self.P, self.sigma_f, self.ell)
        return float(self.m0 + (k_star @ self._alpha).item())

    def posterior_grad(self, p):
        if self.n_points == 0:
            return np.zeros(2)
        p = np.atleast_2d(p)
        k_star = rbf_kernel(p, self.P, self.sigma_f, self.ell)
        diff = self.P - p
        grad = (k_star * self._alpha.T) @ diff / self.ell**2
        return grad.flatten()

    def posterior_var(self, p):
        if self.n_points == 0:
            return self.sigma_f**2
        p = np.atleast_2d(p)
        k_star = rbf_kernel(p, self.P, self.sigma_f, self.ell)
        k_ss = rbf_kernel(p, p, self.sigma_f, self.ell)[0, 0]
        return max(k_ss - float((k_star @ self._K_inv @ k_star.T).item()), 0.0)


class ModelFreeCBF:
    """
    Ackermann-native, drop-in-named replacement for the original
    ModelFreeCBF: same public method names (set_obstacles / set_obstacles_xy
    / get_barrier_and_variance / get_gradient / compute_safety_margin /
    compute_safe_control / check_feasibility) so cbf_Node_refactored_
    ackermann.py reads almost identically to the original node, but
    internally: per-obstacle GPs + soft-min aggregation (Eq. 3-6) instead of
    a single blended GP, and u=[v, phi] (steering angle) everywhere instead
    of u=[v, omega].
    """

    def __init__(self, dt, u_min, u_max, r_max, r_min_obstacle,
                 length_scale, sigma_f, lambda_0, lambda_1, c_q,
                 kappa=12.0, r_buf=0.15, sigma_cap=None):
        self.dt = dt
        self.u_min = np.array(u_min, dtype=float)
        self.u_max = np.array(u_max, dtype=float)
        self.r_max = r_max
        self.r_min_obstacle = r_min_obstacle
        self.length_scale = length_scale
        self.sigma_f = sigma_f
        self.lambda_0 = lambda_0
        self.lambda_1 = lambda_1
        self.c_q = c_q
        self.kappa = kappa
        self.r_buf = r_buf
        # NOTE: sigma_cap intentionally defaults to None (uncapped). Capping
        # sigma_k below what c_q * sigma_bar_eta demands silently lowers the
        # *actual* confidence level below c_q, which would misreport the
        # Theorem 2 violation bound. If the QP is going infeasible too often
        # on hardware, lower c_q, lambda_0/lambda_1, or u_max instead of
        # capping sigma_k -- see run_ackermann_sim.py's tuning notes.
        self.sigma_cap = sigma_cap if sigma_cap is not None else np.inf

        self._P_qp = np.array([8.0, 2.0])  # diagonal QP tracking weights
        self.gps = {}
        self.last_infeasible_info = None

        # Fallback creep speed (see compute_safe_control's fallback comment):
        # for an ACKERMANN vehicle, theta_dot = (v/L)*tan(phi) is IDENTICALLY
        # ZERO whenever v=0, no matter what phi is commanded -- unlike a
        # differential-drive robot, which can still rotate in place at v=0.
        # Braking all the way to v_min=0 during an infeasible/uncertain
        # episode therefore doesn't just slow the car down, it removes its
        # ability to change heading (or gather new EKF information) AT ALL,
        # which was found during validation to cause a permanent deadlock:
        # v locks at 0, theta stops changing, F_q drifts to whatever is
        # consistent with "parked," and the QP never becomes feasible again.
        # A small nonzero creep speed is used for the fallback specifically
        # (not for u_min itself, which may still legitimately be 0.0 for the
        # QP's normal operating range) so the vehicle always retains the
        # ability to keep turning and re-exciting the estimator. It is only
        # engaged after being stuck infeasible-and-nearly-stopped for
        # stuck_limit consecutive calls -- the FIRST response to infeasibility
        # is still a full brake (best short-term stopping power); creep is
        # the escape hatch for the deadlock case, not the default reaction.
        self.fallback_creep_v = max(0.15, 0.25 * self.u_max[0])
        self.stuck_limit = max(1, int(0.15 / self.dt))     # ~0.15 s of being stuck
        self._stuck_counter = 0

    # -- obstacle ingestion (per-obstacle GP, Eq. 3-6) -----------------------
    def set_obstacles(self, ranges, angles, robot_xy=(0.0, 0.0), robot_theta=0.0,
                       cluster_gap=0.3):
        """
        Convert a LiDAR scan into per-obstacle point clusters and route each
        cluster's points to its own GP (Section II-E: "each range return is
        attributed to the obstacle whose boundary generated the reflection").
        Clustering here is a simple angular-gap split -- adequate for sparse,
        already-downsampled scans; swap in a proper clustering front end if
        the scan is dense.
        """
        ranges = np.asarray(ranges, dtype=float)
        angles = np.asarray(angles, dtype=float)
        mask = (ranges < self.r_min_obstacle) & (ranges > 0.05)
        r, th = ranges[mask], angles[mask]
        if len(r) == 0:
            return

        order = np.argsort(th)
        r, th = r[order], th[order]
        cluster_id = np.zeros(len(r), dtype=int)
        cid = 0
        for i in range(1, len(r)):
            if (th[i] - th[i - 1]) > cluster_gap:
                cid += 1
            cluster_id[i] = cid

        world_a = robot_theta + th
        x = robot_xy[0] + r * np.cos(world_a)
        y = robot_xy[1] + r * np.sin(world_a)

        for cid_val in np.unique(cluster_id):
            m = cluster_id == cid_val
            pts = np.column_stack((x[m], y[m]))
            if cid_val not in self.gps:
                self.gps[cid_val] = ObstacleGP(self.length_scale, self.sigma_f)
            self.gps[cid_val].add_points(pts)

    def set_obstacles_xy(self, points_xy, obstacle_id=0):
        """Directly ingest points for a single named obstacle (bypass clustering)."""
        if obstacle_id not in self.gps:
            self.gps[obstacle_id] = ObstacleGP(self.length_scale, self.sigma_f)
        self.gps[obstacle_id].add_points(points_xy)

    @property
    def N(self):
        return sum(1 for gp in self.gps.values() if gp.n_points > 0)

    # -- soft-min aggregation, Eq. (3) ---------------------------------------
    def get_barrier_and_variance(self, p):
        terms = [(gp.posterior_mean(p) - self.r_buf, gp.posterior_var(p))
                 for gp in self.gps.values() if gp.n_points > 0]
        if not terms:
            return 1.0, 0.0
        qs = np.array([t[0] for t in terms])
        vs = np.array([t[1] for t in terms])
        m = np.max(-self.kappa * qs)
        w_un = np.exp(-self.kappa * qs - m)
        Z = np.sum(w_un)
        q = -(np.log(Z) + m) / self.kappa
        w = w_un / Z
        sigma_sq = float(np.sum(w * vs))
        q = max(-2.0, min(5.0, q))
        return q, sigma_sq

    def get_gradient(self, p):
        active = [(gp.posterior_mean(p) - self.r_buf, gp.posterior_grad(p))
                  for gp in self.gps.values() if gp.n_points > 0]
        if not active:
            return np.zeros(2)
        qs = np.array([t[0] for t in active])
        grads = np.array([t[1] for t in active])
        m = np.max(-self.kappa * qs)
        w_un = np.exp(-self.kappa * qs - m)
        w = w_un / np.sum(w_un)
        return (w[:, None] * grads).sum(axis=0)

    # -- tightening margin, Eq. (12)-(14) ------------------------------------
    def compute_safety_margin(self, P, u_max=None):
        """sigma_k = c_q * sigma_bar_eta,k (+ discretization terms, left at 0
        here -- see Eq. 14; discretization margin can be added by the caller
        if desired). sigma_bar_eta,k = max over box vertices, Eq. (13)."""
        best = 0.0
        for v in (self.u_min[0], self.u_max[0]):
            for phi in (self.u_min[1], self.u_max[1]):
                ell = np.array([self.lambda_0 * self.lambda_1,
                                 self.lambda_0 + self.lambda_1, 1.0, v, phi])
                best = max(best, float(ell @ P @ ell))
        sigma_bar = float(np.sqrt(max(best, 0.0)))
        sigma_k = self.c_q * sigma_bar
        return min(sigma_k, self.sigma_cap), sigma_bar

    # -- Lemma 1 feasibility --------------------------------------------------
    def check_feasibility(self, q_hat, qdot_hat, F_q_hat, B_q_hat, sigma_k):
        B_q_hat = np.asarray(B_q_hat, dtype=float)
        r_k = (-F_q_hat - (self.lambda_0 + self.lambda_1) * qdot_hat
               - self.lambda_0 * self.lambda_1 * q_hat + sigma_k)
        M_k = 0.0
        for j in range(len(B_q_hat)):
            M_k += B_q_hat[j] * (self.u_max[j] if B_q_hat[j] >= 0 else self.u_min[j])
        return M_k >= r_k, M_k, r_k

    # -- closed-form CBF-only QP (Eq. 11's CBF row; see module docstring in
    #    ackermann_cbf_core.solve_cbf_qp_closed_form for the general form) ---
    def _solve_qp(self, u_ref, a, b):
        u_ref = np.asarray(u_ref, dtype=float)
        P_diag = self._P_qp
        u_min, u_max = self.u_min, self.u_max

        a_pos, a_neg = np.clip(a, 0, None), np.clip(a, None, 0)
        if (a_pos @ u_min + a_neg @ u_max) > b + 1e-9:
            return None
        u_box = np.clip(u_ref, u_min, u_max)
        if a @ u_box <= b + 1e-9:
            return u_box
        denom = np.sum(a**2 / P_diag)
        if abs(denom) < 1e-12:
            return None
        lam = (a @ u_ref - b) / denom
        u = np.clip(u_ref - lam * a / P_diag, u_min, u_max)
        if a @ u <= b + 1e-6:
            return u
        candidates = []
        for fixed_0 in (u_min[0], u_max[0]):
            if abs(a[1]) > 1e-9:
                u1 = np.clip((b - a[0] * fixed_0) / a[1], u_min[1], u_max[1])
                cand = np.array([fixed_0, u1])
                if a @ cand <= b + 1e-6:
                    candidates.append(cand)
        for fixed_1 in (u_min[1], u_max[1]):
            if abs(a[0]) > 1e-9:
                u0 = np.clip((b - a[1] * fixed_1) / a[0], u_min[0], u_max[0])
                cand = np.array([u0, fixed_1])
                if a @ cand <= b + 1e-6:
                    candidates.append(cand)
        for ux in (u_min[0], u_max[0]):
            for uy in (u_min[1], u_max[1]):
                cand = np.array([ux, uy])
                if a @ cand <= b + 1e-6:
                    candidates.append(cand)
        if not candidates:
            return None
        costs = [0.5 * np.sum(P_diag * (c - u_ref) ** 2) for c in candidates]
        return candidates[int(np.argmin(costs))]

    def compute_safe_control(self, u_ref, q_hat, qdot_hat, F_q_hat, B_q_hat, P,
                              v_current=1.0):
        u_ref = np.asarray(u_ref, dtype=float)
        B_q_hat = np.asarray(B_q_hat, dtype=float)

        sigma_k, sigma_bar = self.compute_safety_margin(P)
        r_k = (-F_q_hat - (self.lambda_0 + self.lambda_1) * qdot_hat
               - self.lambda_0 * self.lambda_1 * q_hat + sigma_k)

        feasible, M_k, _ = self.check_feasibility(q_hat, qdot_hat, F_q_hat, B_q_hat, sigma_k)
        sol = None if not feasible else self._solve_qp(u_ref, -B_q_hat, -r_k)

        if feasible and sol is not None:
            self.last_infeasible_info = None
            self._stuck_counter = 0
            return [float(sol[0]), float(sol[1])], True

        # Infeasible (Lemma 1) or the QP solve degenerated: fall back.
        # Steering authority is preserved (pick the box side of phi that
        # maximizes B_q@u, i.e. the model's best current guess at which way
        # to turn). Velocity's FIRST response is a full brake (v_min) for the
        # strongest immediate stopping power -- but for this Ackermann
        # vehicle, theta_dot=(v/L)tan(phi) is identically zero whenever v=0,
        # no matter what phi is commanded (unlike a differential-drive robot,
        # which can still rotate in place at v=0). Braking all the way to 0
        # and staying there was found during validation to cause a permanent
        # deadlock: v locks at 0, heading stops changing, F_q drifts to
        # whatever is consistent with "parked," and the QP never becomes
        # feasible again. So creep is only engaged after being stuck
        # infeasible-and-nearly-stopped for stuck_limit consecutive calls.
        self.last_infeasible_info = dict(r_k=r_k, B_q=B_q_hat.copy(), sigma_k=sigma_k,
                                          F_q_hat=F_q_hat, qdot_hat=qdot_hat, q_hat=q_hat,
                                          max_achievable=M_k)
        phi_choice = self.u_max[1] if B_q_hat[1] >= 0 else self.u_min[1]
        # Counter increments every consecutive infeasible call (reset only
        # happens above, on the next call that is genuinely feasible again)
        # -- NOT gated on v_current, so that once creep engages it stays
        # engaged instead of dropping back to a full brake (and re-arming the
        # deadlock) the instant the creep speed itself satisfies "v > 0.05".
        self._stuck_counter += 1
        if self._stuck_counter >= self.stuck_limit:
            v_choice = min(self.fallback_creep_v, self.u_max[0])
        else:
            v_choice = self.u_min[0]
        return [float(v_choice), float(phi_choice)], False
