#!/usr/bin/env python3
"""
cbf_Node_refactored.py (Ackermann-native revision, single-file)

Ackermann-native replacement for the previous differential-drive-flavored
version of this node. This file is self-contained: the barrier/QP machinery
(rbf_kernel, ObstacleGP, ModelFreeCBF) that used to live in a separate
CBF_refactored.py is now defined directly below, so the whole safety filter
+ ROS node ships as a single file (there is no measurable per-loop speed
difference from having two files -- Python's import cost is a one-time
startup cost, not a per-cycle one -- this was simply a deployment
simplification).

GROUND TRUTH: L.A. Duffaut Espinosa & C. Grund, "Safety via Control Barrier
Functions Synthesized from Ultra-Local Models" ("the paper"). Every equation
cited in comments below refers to that paper. This node was derived from
(and is validated against) ackermann_cbf_core.py / run_ackermann_sim.py in
this same delivery, which reproduce the paper's Section V numerical-
illustration methodology on true Ackermann kinematics before anything was
ported to ROS.

WHAT CHANGED FROM THE ORIGINAL cbf_Node_refactored.py, AND WHY
----------------------------------------------------------------------------
1. u = [v, phi] (steering angle) everywhere, not u = [v, omega].
   The original node ran the entire ULM/EKF/CBF pipeline on a differential-
   drive-style [v, omega] command, then converted to steering angle only at
   send_command() via steering_angle = atan(L*omega/v) -- singular at v~0,
   dependent on the wheelbase L exactly where the paper's approach is
   designed to need no kinematic constants, and lossy (the EKF was learning
   sensitivity to a quantity -- omega -- the car cannot actually command).
   Here, B_q's second column IS the barrier's sensitivity to the actual
   steering command, and send_command() publishes it directly.

2. Per-obstacle GP barriers aggregated via the soft-min, Eq. (3), instead of
   a single GP fit fresh from every scan. Section II-E: "a separate GP h_i is
   maintained for each obstacle... each range return is attributed to the
   obstacle whose boundary generated the reflection." ModelFreeCBF.
   set_obstacles() below does a simple angular-gap clustering step to route
   points to per-obstacle GPs, which persist (with a bounded point buffer)
   across scans in the WORLD frame using odometry, rather than being
   rebuilt from scratch every callback.

3. B_q is no longer clamped to be strictly positive. For a position-
   dependent barrier, B_q,v is genuinely sign-indefinite (driving TOWARD an
   obstacle makes the barrier's second derivative more negative as speed
   increases; driving away makes it more positive) -- see
   ackermann_cbf_core.SafetyULM_EKF.get_estimates() for the full derivation.
   The original clamp (inherited from earlier differential-drive tuning)
   silently told the QP "more speed always helps," which is false in exactly
   the head-on case the filter exists for, and was found during validation
   to cause chronic QP infeasibility.

4. Persistent excitation is now CONTINUOUS, not just a one-shot burst.
   Section II-B: [F_q, B_q] identifiable iff inputs are persistently
   exciting. A one-shot startup burst leaves B_q,phi unidentified again by
   the time an obstacle is actually encountered if the burst finished before
   contact. During validation this produced a near-zero, noise-dominated
   B_q,phi estimate exactly when the safety filter needed it, which twice
   picked the WRONG avoidance side around an obstacle (confirmed by an
   explicit "does it turn correctly both left and right" test) before this
   fix. A small continuous steering dither (small enough not to visibly
   perturb the path) is now always superimposed on the reference so B_q,phi
   stays identifiable throughout the run, not just after startup.

5. When the CBF-QP is infeasible (Lemma 1), the fallback now ALWAYS drives
   v toward v_min (brakes) while still steering toward the model's best
   current guess -- rather than picking whichever box corner the (possibly
   still-unidentified) sign of B_q,phi happens to favor at full speed. See
   ModelFreeCBF.compute_safe_control()'s comment below for the failure mode
   this fixes.

7. Actuator rate limits (acceleration, deceleration, steering slew rate) are
   now baked directly into the box of inputs the QP searches over each cycle
   (ModelFreeCBF.effective_bounds()), rather than left unconstrained. Without
   this, both the QP and the Lemma-1 fallback could request an instantaneous
   speed/steering jump (a real VESC/servo can't do that), which also showed
   up as a sharp, unrealistic "corner" in the simulated trajectory right when
   the safety filter first went infeasible near an obstacle. This is
   deliberately NOT done by clipping the command after the QP solves --
   clipping afterward could silently undo the CBF constraint the QP just
   satisfied. compute_safe_control() now requires the previously applied
   command (u_prev) for exactly this reason.

8. No CuPy. The linear algebra here is all sub-10x10 dense matrices (EKF
   state/covariance, GP kernel matrices with <=30 points); GPU kernel-launch
   overhead dominates actual compute at this size, and CuPy-on-Jetson has
   been a recurring source of friction in this project (see project memory).
   Plain NumPy is simpler, is what was validated in simulation, and is very
   likely faster here too. If profiling on hardware shows the per-obstacle
   GP kernel matrices are a bottleneck, they are the one part of this file
   that could benefit from batched GPU evaluation -- everything else is too
   small to matter.
"""
import time

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


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
    Ackermann-native safety filter: per-obstacle GPs + soft-min aggregation
    (Eq. 3-6), u=[v, phi] (steering angle) everywhere.
    """

    def __init__(self, dt, u_min, u_max, r_max, r_min_obstacle,
                 length_scale, sigma_f, lambda_0, lambda_1, c_q,
                 kappa=12.0, r_buf=0.15, sigma_cap=None,
                 v_accel_max=2.5, v_decel_max=3.5, phi_rate_max=3.0):
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

        # Actuator rate limits (m/s^2 for v, rad/s for phi). Neither the QP
        # nor the Lemma-1 fallback otherwise know how fast the vehicle can
        # actually change speed/steering, so both could in principle request
        # an instantaneous jump (e.g. v: 1.2 -> 0.0 in one 10ms sample) --
        # which a real VESC/servo can't do anyway, and which was also found
        # during validation to produce a sharp, unrealistic "corner" in the
        # trajectory right when the QP first goes infeasible near an
        # obstacle. effective_bounds() below shrinks the box [u_min, u_max]
        # itself to the set actually reachable from the previous command,
        # and THAT tighter box -- not [u_min, u_max] -- is what gets passed
        # into feasibility checking, the tightening margin, and the QP in
        # compute_safe_control(). This is deliberately NOT done by clipping
        # the QP's output after the fact: post-hoc clipping could silently
        # violate the very CBF constraint (a_cbf@u >= r_k) the QP just
        # solved to satisfy, reopening the safety gap this file exists to
        # close. Baking the limits into the box instead keeps the smoothing
        # guaranteed-safe by construction.
        self.v_accel_max = v_accel_max
        self.v_decel_max = v_decel_max
        self.phi_rate_max = phi_rate_max
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

    # -- actuator rate limiting: shrink the box to what's reachable ----------
    def effective_bounds(self, u_prev):
        """Box of inputs reachable in one dt from u_prev given
        v_accel_max/v_decel_max/phi_rate_max -- see __init__'s comment for
        why this is used INSTEAD OF post-hoc clipping everywhere below."""
        u_prev = np.asarray(u_prev, dtype=float)
        lo = np.array([max(self.u_min[0], u_prev[0] - self.v_decel_max * self.dt),
                       max(self.u_min[1], u_prev[1] - self.phi_rate_max * self.dt)])
        hi = np.array([min(self.u_max[0], u_prev[0] + self.v_accel_max * self.dt),
                       min(self.u_max[1], u_prev[1] + self.phi_rate_max * self.dt)])
        return lo, hi

    # -- tightening margin, Eq. (12)-(14) ------------------------------------
    def compute_safety_margin(self, P, u_min=None, u_max=None):
        """sigma_k = c_q * sigma_bar_eta,k (+ discretization terms, left at 0
        here -- see Eq. 14; discretization margin can be added by the caller
        if desired). sigma_bar_eta,k = max over box vertices, Eq. (13).
        u_min/u_max default to the vehicle's absolute bounds but should be
        the per-step reachable box from effective_bounds() when called from
        compute_safe_control (see there)."""
        u_min = self.u_min if u_min is None else u_min
        u_max = self.u_max if u_max is None else u_max
        best = 0.0
        for v in (u_min[0], u_max[0]):
            for phi in (u_min[1], u_max[1]):
                ell = np.array([self.lambda_0 * self.lambda_1,
                                 self.lambda_0 + self.lambda_1, 1.0, v, phi])
                best = max(best, float(ell @ P @ ell))
        sigma_bar = float(np.sqrt(max(best, 0.0)))
        sigma_k = self.c_q * sigma_bar
        return min(sigma_k, self.sigma_cap), sigma_bar

    # -- Lemma 1 feasibility --------------------------------------------------
    def check_feasibility(self, q_hat, qdot_hat, F_q_hat, B_q_hat, sigma_k,
                           u_min=None, u_max=None):
        u_min = self.u_min if u_min is None else u_min
        u_max = self.u_max if u_max is None else u_max
        B_q_hat = np.asarray(B_q_hat, dtype=float)
        r_k = (-F_q_hat - (self.lambda_0 + self.lambda_1) * qdot_hat
               - self.lambda_0 * self.lambda_1 * q_hat + sigma_k)
        M_k = 0.0
        for j in range(len(B_q_hat)):
            M_k += B_q_hat[j] * (u_max[j] if B_q_hat[j] >= 0 else u_min[j])
        return M_k >= r_k, M_k, r_k

    # -- closed-form CBF-only QP (Eq. 11's CBF row; see module docstring in
    #    ackermann_cbf_core.solve_cbf_qp_closed_form for the general form) ---
    def _solve_qp(self, u_ref, a, b, u_min=None, u_max=None):
        u_ref = np.asarray(u_ref, dtype=float)
        P_diag = self._P_qp
        u_min = self.u_min if u_min is None else u_min
        u_max = self.u_max if u_max is None else u_max

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
                              u_prev, v_current=1.0):
        """
        u_prev: the [v, phi] actually applied last cycle (NOT just the
        odometry speed) -- required to compute the reachable-set box via
        effective_bounds(). Everything below (margin, feasibility, QP,
        fallback) is computed against that box rather than the vehicle's
        absolute [u_min, u_max], so the resulting command is both safe
        (Lemma 1 w.r.t. what's reachable) and automatically smooth (never
        asks for more accel/decel/steering-rate than v_accel_max/
        v_decel_max/phi_rate_max allow) without any post-hoc clipping.
        """
        u_ref = np.asarray(u_ref, dtype=float)
        B_q_hat = np.asarray(B_q_hat, dtype=float)
        u_lo, u_hi = self.effective_bounds(u_prev)

        sigma_k, sigma_bar = self.compute_safety_margin(P, u_lo, u_hi)
        r_k = (-F_q_hat - (self.lambda_0 + self.lambda_1) * qdot_hat
               - self.lambda_0 * self.lambda_1 * q_hat + sigma_k)

        feasible, M_k, _ = self.check_feasibility(q_hat, qdot_hat, F_q_hat, B_q_hat, sigma_k, u_lo, u_hi)
        sol = None if not feasible else self._solve_qp(u_ref, -B_q_hat, -r_k, u_lo, u_hi)

        if feasible and sol is not None:
            self.last_infeasible_info = None
            self._stuck_counter = 0
            return [float(sol[0]), float(sol[1])], True

        # Infeasible (Lemma 1, within the reachable box u_lo/u_hi) or the QP
        # solve degenerated: fall back. Steering authority is preserved (pick
        # the box side of phi that maximizes B_q@u, i.e. the model's best
        # current guess at which way to turn) and the *target* velocity's
        # FIRST response is a full brake for the strongest immediate stopping
        # power -- but for this Ackermann vehicle, theta_dot=(v/L)tan(phi) is
        # identically zero whenever v=0, no matter what phi is commanded
        # (unlike a differential-drive robot, which can still rotate in
        # place at v=0). Braking all the way to 0 and staying there was
        # found during validation to cause a permanent deadlock: v locks at
        # 0, heading stops changing, F_q drifts to whatever is consistent
        # with "parked," and the QP never becomes feasible again. So creep is
        # only targeted after being stuck infeasible-and-nearly-stopped for
        # stuck_limit consecutive calls. Either way the TARGET is clipped
        # into [u_lo, u_hi], so the fallback ramps exactly as smoothly as a
        # normal QP solution would instead of jumping.
        self.last_infeasible_info = dict(r_k=r_k, B_q=B_q_hat.copy(), sigma_k=sigma_k,
                                          F_q_hat=F_q_hat, qdot_hat=qdot_hat, q_hat=q_hat,
                                          max_achievable=M_k)
        phi_target = self.u_max[1] if B_q_hat[1] >= 0 else self.u_min[1]
        phi_choice = float(np.clip(phi_target, u_lo[1], u_hi[1]))
        # Counter increments every consecutive infeasible call (reset only
        # happens above, on the next call that is genuinely feasible again)
        # -- NOT gated on v_current, so that once creep engages it stays
        # engaged instead of dropping back to a full brake (and re-arming the
        # deadlock) the instant the creep speed itself satisfies "v > 0.05".
        self._stuck_counter += 1
        if self._stuck_counter >= self.stuck_limit:
            v_target = min(self.fallback_creep_v, self.u_max[0])
        else:
            v_target = self.u_min[0]
        v_choice = float(np.clip(v_target, u_lo[0], u_hi[0]))
        return [float(v_choice), float(phi_choice)], False


class SafetyULM_EKF:
    """
    Second-order MIMO ULM EKF for the safety output, Eq. (8)-(10):
        q_ddot = F_q + B_q @ u,   u = [v, phi]
    State xi^q_k = [q_k, qdot_k, F_q,k, B_q,v,k, B_q,phi,k]^T.
    """

    def __init__(self, Ts, m_inputs=2):
        self.Ts = Ts
        self.m = m_inputs
        state_dim = 3 + m_inputs
        self.x = np.zeros(state_dim)
        self.x[2] = 0.0     # F_q
        self.x[3] = 1.0     # B_q,v  (prior: more speed away from an obstacle helps)
        if m_inputs > 1:
            self.x[4] = 0.0  # B_q,phi (no prior belief about steering's effect)

        P_diag = [0.01, 0.01, 0.05] + [0.05] * m_inputs
        self.P = np.diag(P_diag)
        Q_diag = [1e-6, 1e-5, 2e-4] + [2e-4] * m_inputs
        self.Q = np.diag(Q_diag)

        self.H_q = np.zeros((1, state_dim)); self.H_q[0, 0] = 1.0
        self.H_qdot = np.zeros((1, state_dim)); self.H_qdot[0, 1] = 1.0

    def predict(self, u):
        Ts = self.Ts
        u = np.asarray(u, dtype=float)
        q, qdot, F_q = self.x[0], self.x[1], self.x[2]
        B_q = self.x[3:3 + self.m]

        qddot = F_q + B_q @ u
        self.x[0] = q + Ts * qdot + (Ts**2 / 2) * qddot
        self.x[1] = qdot + Ts * qddot

        A = np.eye(3 + self.m)
        A[0, 1] = Ts
        A[0, 2] = Ts**2 / 2
        A[0, 3:3 + self.m] = (Ts**2 / 2) * u
        A[1, 2] = Ts
        A[1, 3:3 + self.m] = Ts * u
        self.P = A @ self.P @ A.T + self.Q

    def update_q(self, q_meas, R_q):
        """Measurement 1: q_k from the GP posterior mean, Eq. (4)."""
        R = np.array([[max(R_q, 1e-6)]])
        y = q_meas - self.H_q @ self.x
        S = self.H_q @ self.P @ self.H_q.T + R
        K = self.P @ self.H_q.T / S[0, 0]
        self.x = self.x + K.flatten() * y
        self.P = (np.eye(len(self.x)) - np.outer(K, self.H_q)) @ self.P

    def update_qdot(self, qdot_meas, R_qdot):
        """Measurement 2: qdot_meas = grad(h)^T p_dot_hat, Section III-B."""
        R = np.array([[max(R_qdot, 1e-6)]])
        y = qdot_meas - self.H_qdot @ self.x
        S = self.H_qdot @ self.P @ self.H_qdot.T + R
        K = self.P @ self.H_qdot.T / S[0, 0]
        self.x = self.x + K.flatten() * y
        self.P = (np.eye(len(self.x)) - np.outer(K, self.H_qdot)) @ self.P

    def get_estimates(self):
        B_q = np.clip(self.x[3:3 + self.m].copy(), -6.0, 6.0)
        return float(self.x[0]), float(self.x[1]), float(self.x[2]), B_q, self.P.copy()


class PositionULM_EKF:
    """First-order MIMO ULM for position, p_dot = F_p + B_p @ u, u = [v, phi]."""

    def __init__(self, Ts, m_inputs=2):
        self.Ts = Ts
        self.m = m_inputs
        state_dim = 2 + 2 + 2 * m_inputs
        self.x = np.zeros(state_dim)
        self.x[4] = 1.0   # B_p,x,v
        self.x[6] = 0.0   # B_p,y,v
        self.x[5] = 0.0   # B_p,x,phi
        self.x[7] = 0.0   # B_p,y,phi

        self.P = np.diag([0.01, 0.01, 0.1, 0.1] + [0.1] * (2 * m_inputs))
        self.Q = np.diag([1e-6, 1e-6, 1e-3, 1e-3] + [1e-3] * (2 * m_inputs))
        self.R = np.diag([1e-3, 1e-3])
        self.H = np.zeros((2, state_dim)); self.H[0, 0] = 1.0; self.H[1, 1] = 1.0

    def predict(self, u):
        Ts = self.Ts
        u = np.asarray(u, dtype=float)
        p = self.x[0:2]
        F_p = self.x[2:4]
        B_p = self.x[4:4 + 2 * self.m].reshape(2, self.m)
        pdot = F_p + B_p @ u
        self.x[0:2] = p + Ts * pdot

        A = np.eye(len(self.x))
        A[0, 2] = Ts; A[0, 4:4 + self.m] = Ts * u
        A[1, 3] = Ts; A[1, 4 + self.m:4 + 2 * self.m] = Ts * u
        self.P = A @ self.P @ A.T + self.Q

    def update(self, p_meas):
        p_meas = np.asarray(p_meas, dtype=float).reshape(2, 1)
        y = p_meas - (self.H @ self.x).reshape(2, 1)
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + (K @ y).flatten()
        self.P = (np.eye(len(self.x)) - K @ self.H) @ self.P

    def get_estimates(self):
        return (self.x[0:2].copy(), self.x[2:4].copy(),
                self.x[4:4 + 2 * self.m].reshape(2, self.m).copy())


class ControllerNode(Node):
    def __init__(self):
        super().__init__('ModelFreeCBF_Node_Ackermann')

        # --- parameters ---
        self.dt = 0.01                 # 100 Hz control loop
        self.L = 0.33                  # F1TENTH wheelbase (m) -- used ONLY for
        # the true-plant / dynamic-extension bookkeeping the vehicle firmware
        # already does; the safety filter itself never uses L (that is the
        # entire point of the model-free approach).
        self.v_min, self.v_max = 0.8, 1.2
        self.phi_min, self.phi_max = -0.4, 0.4   # F1TENTH steering limits (rad)
        self.r_max = 3.0
        self.length_scale = 0.30
        self.sigma_f = 1.0
        self.r_buf = 0.15

        # HOCBF parameters, Section II-C / III-C.
        self.lambda_0 = 1.2
        self.lambda_1 = 1.2
        self.c_q = 0.8      # confidence quantile, Eq. (14) -- see
        # run_ackermann_sim.py's tuning note on why this is lower than the
        # paper's own c_q=2/3 examples: this course's B_q,phi identifiability
        # is weaker than the paper's fully-converged illustration, so a
        # looser (but still principled, Eq. 14-consistent) quantile is used
        # to keep the QP feasible. Raise this once field data shows the EKF
        # covariance converges faster/tighter than assumed here.

        # --- state ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0

        self.u_ref = np.array([1.0, 0.0])     # [v_ref, phi_ref]
        self.u_prev = np.array([1.0, 0.0])
        self.v_prev = 1.0

        self.goal_x = 10.0
        self.goal_y = 0.0

        # continuous persistent-excitation dither, module docstring point 4
        self._dither_ampl = 0.05
        self._dither_period_steps = 30
        self._step_count = 0
        self._excite_counter = -1
        self._excite_steps = 24     # 1.2 s at 100 Hz -- paper Sec. V

        self.safety_ekf = SafetyULM_EKF(Ts=self.dt, m_inputs=2)
        self.position_ekf = PositionULM_EKF(Ts=self.dt, m_inputs=2)

        self.cbf = ModelFreeCBF(
            dt=self.dt,
            u_min=[self.v_min, self.phi_min],
            u_max=[self.v_max, self.phi_max],
            r_max=self.r_max,
            r_min_obstacle=self.r_max,
            length_scale=self.length_scale,
            sigma_f=self.sigma_f,
            lambda_0=self.lambda_0,
            lambda_1=self.lambda_1,
            c_q=self.c_q,
            r_buf=self.r_buf,
        )

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.cmd_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        self.get_logger().info('Model-Free CBF Node (Ackermann-native, u=[v,phi]) initialized')

    # -------------------------------------------------------------------
    def gap_following_controller(self):
        """
        Gap-following reference controller: finds the largest free gap in
        the LiDAR scan and steers toward it, emitting phi_ref (steering
        angle) DIRECTLY -- no omega, no conversion. Returns [v_ref, phi_ref].
        """
        v_ref, phi_ref = 1.0, 0.0
        if not hasattr(self, 'last_ranges'):
            return np.array([v_ref, phi_ref])

        ranges, angles = self.last_ranges, self.last_angles
        front = np.abs(angles) < np.pi / 2
        fr, fa = ranges[front], angles[front]
        if len(fr) == 0:
            return np.array([v_ref, phi_ref])

        is_free = fr > 0.5
        best_size, best_angle, cur_size, cur_start = 0, 0.0, 0, 0
        for i in range(len(is_free)):
            if is_free[i]:
                if cur_size == 0:
                    cur_start = i
                cur_size += 1
            else:
                if cur_size > best_size:
                    best_size = cur_size
                    best_angle = float(fa[cur_start + cur_size // 2])
                cur_size = 0
        if cur_size > best_size:
            best_size = cur_size
            best_angle = float(fa[cur_start + cur_size // 2])
        if best_size == 0:
            best_angle = float(fa[int(np.argmax(fr))])

        # Steer toward the gap center directly in steering-angle space.
        K_p = 1.2
        phi_ref = float(np.clip(K_p * best_angle, self.phi_min, self.phi_max))
        return np.array([v_ref, phi_ref])

    def _apply_persistent_excitation(self, u_ref):
        """Module docstring point 4: continuous small dither + a stronger
        one-shot burst triggered on first obstacle contact."""
        if self._excite_counter < 0 and self.cbf.N > 0:
            self._excite_counter = 0
        if 0 <= self._excite_counter < self._excite_steps:
            phase = 2 * np.pi * self._excite_counter / (self._excite_steps / 2)
            u_ref = np.array([0.5, 0.15 * np.sin(phase)])
            self._excite_counter += 1
        else:
            if self._excite_counter >= 0:
                self._excite_counter += 1
            dither = self._dither_ampl * np.sin(
                2 * np.pi * self._step_count / self._dither_period_steps)
            u_ref = u_ref.copy()
            u_ref[1] = float(np.clip(u_ref[1] + dither, self.phi_min, self.phi_max))
        return u_ref

    # -------------------------------------------------------------------
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        self.theta = float(np.arctan2(siny_cosp, cosy_cosp))
        self.v = msg.twist.twist.linear.x
        self.position_ekf.update(np.array([self.x, self.y]))

    def lidar_callback(self, msg):
        start_time = time.time()
        self._step_count += 1

        ranges_raw = msg.ranges[::20]
        angles_raw = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))[::20]
        ranges = np.asarray(ranges_raw, dtype=np.float32)
        angles = angles_raw.astype(np.float32)
        self.last_ranges, self.last_angles = ranges, angles

        # Per-obstacle GP ingestion in the WORLD frame using odometry, so the
        # learned barrier persists across scans (Section II-E), rather than
        # being rebuilt from a single instantaneous scan every callback.
        self.cbf.set_obstacles(ranges, angles, robot_xy=(self.x, self.y),
                                robot_theta=self.theta)
        valid_ranges = ranges[(ranges > 0.1) & (ranges < self.r_max)]
        p_world = np.array([self.x, self.y])

        # EKF predict, Eq. (8)-(10) discretized.
        self.safety_ekf.predict(self.u_prev)
        self.position_ekf.predict(self.u_prev)

        # Nominal reference + persistent excitation.
        if self._step_count % 3 == 0:
            self.u_ref = self.gap_following_controller()
        u_ref = self._apply_persistent_excitation(self.u_ref)

        # Measurements: q from GP posterior mean (Eq. 4), qdot = grad(h)^T p_dot_hat.
        q_meas, sigma_gp_sq = self.cbf.get_barrier_and_variance(p_world)
        grad_h = self.cbf.get_gradient(p_world)
        _, F_p, B_p = self.position_ekf.get_estimates()
        p_dot = F_p + B_p @ self.u_prev
        qdot_meas = float(grad_h @ p_dot)

        grad_norm = float(np.linalg.norm(grad_h))
        sigma_pdot = 0.02
        eps = 1e-6
        R_qdot = grad_norm**2 * sigma_pdot**2 + sigma_gp_sq / (self.length_scale**2 * (grad_norm**2 + eps))
        R_qdot = min(R_qdot, 1.0)

        if np.isfinite(q_meas):
            self.safety_ekf.update_q(float(q_meas), R_q=max(float(sigma_gp_sq), 1e-4))
        else:
            self.get_logger().warn(f'Invalid q_meas={q_meas}, skipping update')
        if np.isfinite(qdot_meas) and abs(qdot_meas) < 10.0:
            self.safety_ekf.update_qdot(qdot_meas, R_qdot=max(float(R_qdot), 1e-4))
        else:
            self.get_logger().warn(f'Invalid qdot_meas={qdot_meas}, skipping update')

        q_hat, qdot_hat, F_q_hat, B_q_hat, P_safety = self.safety_ekf.get_estimates()

        # Reference-level courtesy slow-down near obstacles (does not affect
        # the safety guarantee -- the CBF-QP enforces q>=0 regardless -- it
        # just keeps the requested reference within reach of the vehicle's
        # bounded curvature, reducing how hard the QP has to fight it).
        u_ref = u_ref.copy()
        u_ref[0] *= float(np.clip((q_hat if np.isfinite(q_hat) else 1.0) / 0.6, 0.25, 1.0))

        min_range = float(np.min(valid_ranges)) if len(valid_ranges) > 0 else 999.0
        if min_range < 0.30:
            # Hard emergency stop -- distinct from, and in addition to, the
            # CBF-QP: a last-resort layer for genuinely imminent contact.
            u_safe, feasible = [0.0, self.u_prev[1]], True
        else:
            try:
                u_safe, feasible = self.cbf.compute_safe_control(
                    u_ref=u_ref, q_hat=q_hat, qdot_hat=qdot_hat,
                    F_q_hat=F_q_hat, B_q_hat=B_q_hat, P=P_safety,
                    u_prev=self.u_prev, v_current=self.v)
            except Exception as e:
                self.get_logger().warn(f'CBF QP raised {e!r}; braking with steering held')
                u_safe, feasible = [0.0, float(self.u_prev[1])], False

        self.send_command(u_safe[0], u_safe[1])
        self.u_prev = np.array(u_safe)

        if self._step_count % 50 == 0:
            total_time = time.time() - start_time
            action = 'SAFE' if feasible and abs(u_safe[0] - self.u_ref[0]) < 0.1 else \
                     ('STEER' if abs(u_safe[1] - self.u_ref[1]) > 0.05 else 'BRAKE')
            self.get_logger().info(
                f'[{action}] {total_time*1000:.0f}ms | q={q_hat:.2f} | '
                f'B_q=[{B_q_hat[0]:.2f},{B_q_hat[1]:.2f}] | v={u_safe[0]:.2f} phi={u_safe[1]:.2f}')

    def send_command(self, v, phi):
        """
        Publish the Ackermann drive command DIRECTLY -- phi is already the
        physical steering angle the safety filter reasoned about, so there is
        no conversion step here at all (contrast with the original node's
        steering_angle = atan(L*omega/v)).
        """
        msg = AckermannDriveStamped()
        msg.drive.speed = float(v)
        msg.drive.acceleration = -5.0 if v < self.v_prev - 0.1 else 3.0
        self.v_prev = v
        msg.drive.steering_angle = float(np.clip(phi, self.phi_min, self.phi_max))
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
