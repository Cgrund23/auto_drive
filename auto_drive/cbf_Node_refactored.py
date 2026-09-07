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
import csv
import os
import time
from datetime import datetime

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

        self._P_qp = np.array([16.0, 1.0])  # diagonal QP tracking weights:
        # v deviations are made expensive relative to phi deviations so that
        # when the CBF constraint forces a departure from u_ref, the QP
        # prefers to steer around the obstacle rather than brake.
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

        # Fallback steering dither: a small perturbation superimposed on
        # phi_choice below, distinct from (and in addition to) the node-level
        # dither on u_ref. The node-level dither is USELESS during a
        # sustained infeasible episode because this fallback branch never
        # looks at u_ref -- it drives phi_choice, a value held essentially
        # constant at one box extreme for as long as B_q_hat's sign doesn't
        # flip. A constant input is the worst case for the online
        # identifier: nothing new excites B_q,phi, so a wrong sign picked up
        # early in a long turn can simply never correct itself, and the
        # vehicle just keeps arcing the same way until it hits whatever is
        # on that side. (Confirmed in simulation: B_q,phi stayed negative for
        # 420 consecutive infeasible cycles while phi sat at its lower bound
        # the entire time, carving the vehicle into the opposite wall.)
        # This dither keeps phi_choice moving during the SAME box-extreme
        # commitment, which is enough to keep re-exciting B_q,phi's sign
        # without undoing the "steer toward the model's best guess" intent.
        self.fallback_dither_ampl = 0.08
        self.fallback_dither_period = max(1, int(0.3 / self.dt))  # ~0.3 s
        self._fallback_dither_counter = 0

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
                              u_prev, v_current=1.0, raw_side_bias=0.0):
        """
        u_prev: the [v, phi] actually applied last cycle (NOT just the
        odometry speed) -- required to compute the reachable-set box via
        effective_bounds(). Everything below (margin, feasibility, QP,
        fallback) is computed against that box rather than the vehicle's
        absolute [u_min, u_max], so the resulting command is both safe
        (Lemma 1 w.r.t. what's reachable) and automatically smooth (never
        asks for more accel/decel/steering-rate than v_accel_max/
        v_decel_max/phi_rate_max allow) without any post-hoc clipping.

        raw_side_bias: caller-supplied (min raw range on the LEFT half of the
        current scan) - (min raw range on the RIGHT half), i.e. positive
        means the left is more open. Used ONLY as a tie-breaker in the
        infeasible fallback below, in place of a stale B_q_hat[1] sign -- see
        that branch's comment for why this is necessary.
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
            self._fallback_dither_counter = 0
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
        # B_q_hat[1] is a SINGLE scalar representing sensitivity to the
        # combined soft-min barrier -- it cannot represent "steer right, this
        # obstacle is on the left" and "steer left, that wall is now on the
        # right" as two different facts about two different things at once.
        # Confirmed by simulation: B_q,phi locked negative (commit right) to
        # correctly dodge an obstacle, then STAYED negative for 200+
        # consecutive infeasible cycles even as the vehicle closed to <0.3m
        # of the wall on the opposite side, because nothing forced it to
        # re-identify against a now-completely-different nearby threat --
        # every (lambda_0/1, c_q, EKF process noise, length_scale) combo
        # tried in simulation crashed the same way, which is what rules this
        # out as a tuning problem. Once the model has had a fair chance
        # (stuck_limit cycles) and is still disagreeing with which side is
        # actually open right now, defer to the raw scan instead of the
        # model's belief.
        model_dir = 1.0 if B_q_hat[1] >= 0 else -1.0
        raw_dir = 1.0 if raw_side_bias >= 0 else -1.0
        if self._stuck_counter > self.stuck_limit and raw_side_bias != 0.0 and model_dir != raw_dir:
            chosen_dir = raw_dir
        else:
            chosen_dir = model_dir
        phi_target = self.u_max[1] if chosen_dir > 0 else self.u_min[1]
        dither = self.fallback_dither_ampl * np.sin(
            2 * np.pi * self._fallback_dither_counter / self.fallback_dither_period)
        self._fallback_dither_counter += 1
        phi_choice = float(np.clip(phi_target + dither, u_lo[1], u_hi[1]))
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
        # Initial estimate only -- measured directly from real scan-to-scan
        # timestamps in lidar_callback() below and corrected every cycle.
        # 0.01 (100 Hz) was the ORIGINAL assumption here and is wrong:
        # timestamps in an actual hardware log showed the real LiDAR
        # delivers scans every ~26ms (~38 Hz), a 2.6x error that fed
        # directly into the EKF's Ts and effective_bounds()'s actuator
        # rate-limiting box (u_prev +/- rate*dt) -- with dt wrong by 2.6x,
        # that box was ~2.6x narrower than what the vehicle could actually
        # achieve in the real elapsed time between commands, which looks
        # exactly like "doesn't turn quickly enough" independent of any
        # geometry or tuning. This value is just the fallback used before
        # the first scan arrives.
        self.dt = 0.0264
        self.n_control_substeps = 3    # see lidar_callback()
        # Estimated from the real 16in vehicle length (0.65-0.70x length is
        # a typical wheelbase/length ratio) -- not yet a direct measurement,
        # replace with the actual wheelbase when known. Was 0.33m, a generic
        # F1TENTH placeholder inconsistent with a 16in-long chassis (would
        # leave almost no front/rear overhang).
        self.L = 0.27                  # wheelbase (m) -- used ONLY for
        # the true-plant / dynamic-extension bookkeeping the vehicle firmware
        # already does; the safety filter itself never uses L (that is the
        # entire point of the model-free approach).
        # v_min=0.0 (NOT 0.8): the QP needs full braking authority to buy
        # time for steering to catch up on an oncoming obstacle -- that is
        # the mechanism a CBF uses to avoid contact with bounded steering
        # rate. A nonzero floor here was tried and made things WORSE (it
        # forced the vehicle to keep closing distance at >=0.8 m/s with only
        # phi available to react, which is what drove it into the 0.30 m
        # hard-stop layer in the first place). The original "car locks at
        # v=0 forever" deadlock this floor was meant to prevent is instead
        # handled by the stuck/creep escape in ModelFreeCBF.compute_safe_
        # control() and the mirrored escape in lidar_callback's emergency-
        # stop branch, both of which recover from v=0 without giving up the
        # QP's ability to actually brake when it needs to.
        self.v_min, self.v_max = 0.0, 1.5
        self.phi_min, self.phi_max = -0.4, 0.4   # F1TENTH steering limits (rad)
        self.r_max = 3.0
        # length_scale/lambda_0/lambda_1/c_q below were chosen by staged grid
        # searches (sweep_left_wall_follower.py) against the hallway-with-
        # obstacles simulation (simulate_left_wall_follower.py). Corridor
        # width and vehicle size have both been corrected twice now (2.4m
        # sim -> "1.0m hallway" guess -> actual 2.0m hallway with a real
        # 8in x 16in vehicle) -- length_scale and left_wall_setpoint below
        # are rescaled from the 1.0m-corridor sweep's findings by the width
        # ratio (2.0/1.0 = 2x), not re-verified by a fresh sweep at this
        # exact scale. See sweep_left_wall_follower.py to re-validate.
        self.length_scale = 0.3      # GP "safety factor" (l): unsafe-set radius around each LiDAR point
        self.sigma_f = 1.0
        # r_buf: the CONTROLLER's belief about where the boundary is,
        # deliberately more conservative than bare vehicle geometry (that
        # ground truth lives in simulate_left_wall_follower.py's
        # ROBOT_RADIUS). Derived from the real vehicle -- 8in wide -> 0.1016m
        # half-width -- plus a 0.05m margin for LiDAR/EKF noise. Uses the
        # vehicle's HALF-WIDTH, not half-length or half-diagonal: correct
        # while driving roughly straight, but an underestimate during a
        # large heading excursion, when the vehicle presents closer to its
        # long (16in) dimension to the corridor's width. This session has
        # repeatedly produced 60-90 deg excursions during avoidance, so this
        # is a real, unresolved conservatism gap, not a hypothetical one.
        self.r_buf = 0.1016 + 0.05    # = 0.1516, vehicle half-width + noise margin
        # HOCBF parameters, Section II-C / III-C.
        self.lambda_0 = 2.5
        self.lambda_1 = 2.5
        self.c_q = 1.1      # confidence quantile, Eq. (14)

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

        # center-lock corridor-follower (nominal reference controller):
        # drives to keep the vehicle EQUIDISTANT from whatever is on its left
        # and right (walls or obstacles), rather than hugging the left wall
        # at a fixed offset. No setpoint distance needed -- the target is
        # simply left_distance == right_distance.
        self.wall_beam_angle = np.deg2rad(30.0)        # main wall-sensing beam, from forward
        self.wall_beam_separation = np.deg2rad(15.0)   # 2nd beam this far forward of the 1st
        # Halved again from the left-wall-hugging values (kp=0.15, kd=0.3):
        # center_lock_controller()'s error = Dt_left + Dt_right is a SUM of
        # two distances that each move by ~1x a lateral shift, so it has
        # ~2x the old error's sensitivity to the same physical drift
        # (d(error)/dy ~= -2 now vs ~= -1 for the old single-wall-distance
        # error). Halving kp/kd here targets the same steering response per
        # meter of actual lateral drift as before, not re-independently
        # verified at this exact scale.
        self.wall_follow_kp = 0.075
        self.wall_follow_kd = 0.15
        self._wall_follow_prev_error = 0.0
        # The two-beam measurement below assumes heading is roughly aligned
        # with the corridor (theta~=0, i.e. close to however the vehicle was
        # facing at startup) -- both beams are cast at shallow angles FROM
        # THE VEHICLE'S CURRENT HEADING, so once a CBF avoidance swing
        # rotates that heading far enough, the beams point somewhere that
        # has nothing to do with the actual left/right walls, and the
        # resulting "error" is measurement noise, not signal. Confirmed in
        # simulation twice: (1) trusting it anyway fed a large, confidently-
        # wrong phi_ref that kept commanding MORE rotation once heading
        # passed ~90 deg off-corridor, spinning the vehicle into a wall; (2)
        # an earlier fix that just faded this reference's authority toward a
        # nonzero FLOOR (instead of a real correction) created its own
        # deadlock -- once in the fade band, phi_cmd became symmetric dither
        # noise averaging ~0, so heading simply stopped recovering -- AND,
        # separately, once heading had already flipped past ~90-180 deg
        # entirely, a beam-blind fallback had no opinion at all about which
        # way was actually "aligned with the corridor," so the vehicle would
        # just cruise confidently at full v_ref in whatever direction it was
        # now facing (confirmed driving 25m backward past the start of a
        # 10m corridor before finally hitting something). Below, phi_ref is
        # blended between the (locally accurate near theta=0) beam-based
        # term and an ABSOLUTE heading-correction term that actively drives
        # theta back toward 0 -- using odometry theta directly, which is
        # available and reliable at any heading, unlike the beam geometry.
        # The blend weight is the beam term's fade (1.0 at theta_err=0, 0.0
        # by fade_end) so near-corridor-aligned tracking is still governed
        # by the more precise beam measurement, while a large excursion
        # hands off entirely to the term that can actually undo it.
        self.wall_follow_heading_fade_start = np.deg2rad(30.0)
        self.wall_follow_heading_fade_end = np.deg2rad(70.0)
        # Gain on the absolute heading-correction term (phi = -kp*theta_err).
        # Deliberately strong: phi saturates at phi_max/kp ~= 0.4/0.6 ~= 0.67
        # rad (~38 deg) of heading error, so anything beyond a moderate
        # excursion commands full corrective lock rather than a gentle
        # nudge -- appropriate here since, by construction, this term only
        # gets significant blend weight once the beam measurement has
        # already been judged unreliable (heading well off-corridor).
        self.heading_correction_kp = 0.6

        # continuous persistent-excitation dither, module docstring point 4
        self._dither_ampl = 0.05
        self._dither_period_steps = 30
        self._step_count = 0
        self._excite_counter = -1
        self._excite_steps = 24     # 1.2 s at 100 Hz -- paper Sec. V

        # Consecutive cycles the hard emergency-stop layer (below) has held
        # v=0 -- see its comment for why this needs its own creep escape.
        self._estop_stuck_counter = 0
        self._last_scan_time = None    # for measuring the real scan period

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

        # --- hardware run logging (for offline analysis via
        # analyze_hardware_log.py) -----------------------------------------
        # Every control step (not just the 1-in-50 console summary in
        # _control_step) is recorded to a CSV so a run can be replayed and
        # plotted after the fact: position, reference vs. commanded u, the
        # full EKF barrier estimate (q, qdot, F_q, B_q), the confidence
        # margin, feasibility, and min_range. A second CSV records the
        # downsampled scan (world-frame pose + ranges) at perception rate so
        # the persistent per-obstacle GPs -- and the barrier field they
        # define -- can be exactly reconstructed offline by replaying
        # set_obstacles() calls through a fresh ModelFreeCBF, rather than
        # needing to serialize the GPs' internal state directly.
        log_dir = os.path.expanduser('~/cbf_logs')
        os.makedirs(log_dir, exist_ok=True)
        run_id = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._control_log_path = os.path.join(log_dir, f'{run_id}_control.csv')
        self._scan_log_path = os.path.join(log_dir, f'{run_id}_scans.csv')
        self._control_log_f = open(self._control_log_path, 'w', newline='')
        self._control_log_w = csv.writer(self._control_log_f)
        self._control_log_w.writerow([
            't', 'step', 'dt', 'x', 'y', 'theta', 'v_odom',
            'v_ref', 'phi_ref', 'v_cmd', 'phi_cmd',
            'q_hat', 'qdot_hat', 'F_q_hat', 'Bqv_hat', 'Bqphi_hat',
            'sigma_k', 'min_range', 'feasible', 'n_obstacles', 'action',
        ])
        self._scan_log_f = open(self._scan_log_path, 'w', newline='')
        self._scan_log_w = csv.writer(self._scan_log_f)
        self._scan_log_header_written = False

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.cmd_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        self.get_logger().info('Model-Free CBF Node (Ackermann-native, u=[v,phi]) initialized')
        self.get_logger().info(f'Logging control data to {self._control_log_path}')
        self.get_logger().info(f'Logging scans to {self._scan_log_path}')

    # -------------------------------------------------------------------
    def _side_wall_distance(self, sign, ranges, angles):
        """
        Perpendicular distance (projected `lookahead` m ahead) to whatever
        the two-beam pair on ONE side (sign=+1.0 -> left, sign=-1.0 ->
        right) is looking at -- wall or obstacle, this reference doesn't
        distinguish. Same general point-to-line geometry as the original
        left-wall-follower (kept because it's exact for ANY pair of beam
        angles, not just 90 deg), just parameterized by `sign` so the
        identical formula produces both sides' distances -- see
        center_lock_controller() for why the two share one sign convention
        rather than needing an abs().
        """
        b_angle = sign * self.wall_beam_angle
        a_angle = b_angle - sign * self.wall_beam_separation

        def beam_at(target_angle):
            idx = int(np.argmin(np.abs(angles - target_angle)))
            r = float(ranges[idx])
            return r if np.isfinite(r) and r > 0.02 else self.r_max

        a = beam_at(a_angle)
        b = beam_at(b_angle)

        pa = np.array([a * np.cos(a_angle), a * np.sin(a_angle)])
        pb = np.array([b * np.cos(b_angle), b * np.sin(b_angle)])
        d = pb - pa
        d_norm = float(np.linalg.norm(d))
        if d_norm < 1e-6:
            return None

        Dt = float(pa[0] * d[1] - pa[1] * d[0]) / d_norm
        lookahead = 0.5 + 0.5 * max(self.v, 0.0)
        return Dt - lookahead * d[1] / d_norm

    def center_lock_controller(self):
        """
        Center-lock reference controller: steers to keep the vehicle
        EQUIDISTANT from whatever is on its left and right (corridor walls,
        or an obstacle intruding from either side), emitting phi_ref
        DIRECTLY -- no omega, no conversion. Returns [v_ref, phi_ref]; this
        is only the NOMINAL reference -- the CBF-QP safety filter still has
        final say over the actual command. Replaces the previous fixed-
        offset LEFT-wall-hugging reference (see git history for that
        version) -- same two-beam geometry, just mirrored onto both sides.

        Angle convention (REP-103, matches this file's LiDAR frame): scan
        angle increases counter-clockwise from the forward (+x) axis, so
        positive angles point along +y (left), negative along -y (right).

        _side_wall_distance's point-line formula gives a POSITIVE number for
        the left-side beam pair and a NEGATIVE number of the same magnitude
        for the right-side pair when the vehicle is symmetric between two
        parallel surfaces (verified algebraically: mirroring the beam
        angles about the forward axis negates Dt). That means the two
        distances can be combined with a plain sum instead of needing
        abs() + subtraction: left_dist + right_dist == 0 exactly when
        centered, > 0 when there's more room on the left (steer left to
        recenter), < 0 when there's more room on the right (steer right).
        """
        v_ref = 1.0
        if not hasattr(self, 'last_ranges') or len(self.last_angles) == 0:
            return np.array([v_ref, 0.0])

        ranges, angles = self.last_ranges, self.last_angles

        Dt_left = self._side_wall_distance(1.0, ranges, angles)
        Dt_right = self._side_wall_distance(-1.0, ranges, angles)
        if Dt_left is None or Dt_right is None:
            return np.array([v_ref, 0.0])

        error = Dt_left + Dt_right
        # error > 0: more clearance on the left  -> steer toward it (left,  +phi)
        # error < 0: more clearance on the right -> steer toward it (right, -phi)
        d_error = error - self._wall_follow_prev_error
        self._wall_follow_prev_error = error

        phi_ref = self.wall_follow_kp * error + self.wall_follow_kd * d_error
        phi_ref = float(np.clip(phi_ref, self.phi_min, self.phi_max))

        # Blend the beam-based term against an ABSOLUTE heading-correction
        # term as heading strays from corridor-aligned -- see __init__'s
        # comment on wall_follow_heading_fade_start/end / heading_correction_kp
        # for why the beam measurement becomes unreliable (not just noisy)
        # past a certain heading error, and why a real correction is needed
        # here instead of just fading toward silence.
        theta_err = self.theta   # signed; already wrapped to [-pi, pi]
        fade_start, fade_end = self.wall_follow_heading_fade_start, self.wall_follow_heading_fade_end
        abs_err = abs(theta_err)
        if abs_err <= fade_start:
            beam_weight = 1.0
        elif abs_err >= fade_end:
            beam_weight = 0.0
        else:
            beam_weight = 1.0 - (abs_err - fade_start) / (fade_end - fade_start)

        heading_phi = float(np.clip(-self.heading_correction_kp * theta_err,
                                     self.phi_min, self.phi_max))
        phi_ref = beam_weight * phi_ref + (1.0 - beam_weight) * heading_phi
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
        """
        Perception-rate entry point: parses the scan and updates the
        persistent per-obstacle GPs ONCE per real LiDAR message (this part
        genuinely needs new sensor data). The actual control law then runs
        self.n_control_substeps times against that single scan via
        _control_step() -- see that method's docstring for why re-running it
        without new perception data is legitimate rather than "inventing"
        information, and why this exists at all (the real scan rate is far
        below the ~100 Hz this file was designed around).
        """
        now = self.get_clock().now()
        if self._last_scan_time is not None:
            scan_dt = (now - self._last_scan_time).nanoseconds * 1e-9
            scan_dt = float(np.clip(scan_dt, 0.005, 0.5))
        else:
            scan_dt = self.dt   # first callback ever: no prior timestamp yet
        self._last_scan_time = now

        ranges_raw = msg.ranges[::20]
        angles_raw = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))[::20]
        ranges = np.asarray(ranges_raw, dtype=np.float32)
        angles = angles_raw.astype(np.float32)
        self.last_ranges, self.last_angles = ranges, angles

        # Scan log: angles are ~constant for a given LiDAR/downsample factor,
        # so they're written ONCE as a marker row rather than repeated every
        # scan -- analyze_hardware_log.py knows to treat a row starting with
        # '#angles' specially.
        if not self._scan_log_header_written:
            self._scan_log_w.writerow(['#angles'] + [f'{a:.6f}' for a in angles])
            self._scan_log_w.writerow(['t', 'x', 'y', 'theta'] + [f'r{i}' for i in range(len(angles))])
            self._scan_log_header_written = True
        self._scan_log_w.writerow(
            [f'{now.nanoseconds * 1e-9:.6f}', f'{self.x:.4f}', f'{self.y:.4f}', f'{self.theta:.5f}']
            + [f'{r:.3f}' for r in ranges])

        # Per-obstacle GP ingestion in the WORLD frame using odometry, so the
        # learned barrier persists across scans (Section II-E), rather than
        # being rebuilt from a single instantaneous scan every callback.
        self.cbf.set_obstacles(ranges, angles, robot_xy=(self.x, self.y),
                                robot_theta=self.theta)
        valid_ranges = ranges[(ranges > 0.1) & (ranges < self.r_max)]

        # Raw (model-free-of-the-EKF) left-vs-right clearance, used only as a
        # tie-breaker deep in the fallback paths below when the learned
        # B_q_hat[1] sign has been fighting the visible geometry too long.
        left_mask, right_mask = angles > 0, angles < 0
        left_min = float(np.min(ranges[left_mask])) if np.any(left_mask) else self.r_max
        right_min = float(np.min(ranges[right_mask])) if np.any(right_mask) else self.r_max
        raw_side_bias = left_min - right_min

        dt_sub = scan_dt / self.n_control_substeps
        for _ in range(self.n_control_substeps):
            self._control_step(valid_ranges, raw_side_bias, dt_sub)

    def _control_step(self, valid_ranges, raw_side_bias, dt_sub):
        """
        One control-law update. Called self.n_control_substeps times per
        real scan (from lidar_callback) rather than once, so actuation
        updates near the rate this file was originally designed for (~100
        Hz) even though the real LiDAR only delivers new perception at
        ~38 Hz. This is NOT fabricating sensor data: q_meas/qdot_meas below
        come from evaluating the already-fitted, persistent per-obstacle
        GPs (set_obstacles() in lidar_callback, unchanged since the last
        real scan) at the CURRENT position estimate, which legitimately
        keeps evolving between scans via EKF prediction and any new
        odometry that has arrived (odom_callback runs independently, and
        this node's executor is multi-threaded) -- re-evaluating a static
        map at an updated position is a real measurement, not an invented
        one. Only perception itself (the scan, obstacle GP updates,
        raw_side_bias, valid_ranges) is genuinely tied to a new LiDAR
        message and stays out of this method.
        """
        start_time = time.time()
        self._step_count += 1

        self.dt = dt_sub
        self.cbf.dt = dt_sub
        self.safety_ekf.Ts = dt_sub
        self.position_ekf.Ts = dt_sub
        self.cbf.stuck_limit = max(1, int(0.15 / dt_sub))
        self.cbf.fallback_dither_period = max(1, int(0.3 / dt_sub))

        p_world = np.array([self.x, self.y])

        # EKF predict, Eq. (8)-(10) discretized.
        self.safety_ekf.predict(self.u_prev)
        self.position_ekf.predict(self.u_prev)

        # Nominal reference + persistent excitation.
        if self._step_count % 3 == 0:
            self.u_ref = self.center_lock_controller()
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
        # Widened from /0.6 floor 0.25: at v_ref=1.0 and this vehicle's
        # turning radius (L=0.33, phi_max=0.4 rad -> ~0.78m radius), waiting
        # until q_hat<0.6 to start slowing left too little runway to
        # complete a dodge -- confirmed in simulation (crashed into the
        # obstacle it was still approaching at full speed). Scaling from the
        # moment q_hat drops below its safe ceiling (1.0), and allowing a
        # slower floor (0.15 vs 0.25) right up against an obstacle, buys
        # back some of that distance budget without touching the actual
        # safety constraint, which the CBF-QP enforces regardless.
        # Restored a full-speed zone (q_hat >= 0.6) that an earlier change
        # this session removed -- scaling from q_hat=1.0 instead of 0.6
        # meant ANY steady-state proximity to a wall (even a comfortably
        # safe one) permanently capped cruising speed, confirmed on
        # hardware: min_range held a stable ~0.6m (q~0.78, well clear of
        # any real risk) yet speed was pinned at 0.78x forever instead of
        # ramping to full v_ref once settled. Below 0.6 still scales down
        # to the 0.15 floor for genuinely close encounters.
        u_ref = u_ref.copy()
        u_ref[0] *= float(np.clip((q_hat if np.isfinite(q_hat) else 1.0) / 0.6, 0.15, 1.0))

        min_range = float(np.min(valid_ranges)) if len(valid_ranges) > 0 else 999.0
        if min_range < 0.30:
            # Hard emergency stop -- distinct from, and in addition to, the
            # CBF-QP: a last-resort layer for genuinely imminent contact.
            # Freezing v=0 indefinitely here is a deadlock for this Ackermann
            # vehicle: theta_dot = (v/L)*tan(phi) is identically zero
            # whenever v=0, so a car parked a few inches from a wall can
            # never turn to open distance again and just sits there forever
            # (this is exactly what was observed on hardware -- the vehicle
            # steered toward a wall, tripped this branch, and never left).
            # Mirror the CBF fallback's stuck/creep escape: hold a full stop
            # briefly, then creep while re-aiming steering every cycle from
            # the model's current best-guess direction (B_q_hat's sign)
            # instead of freezing at whatever phi happened to be applied the
            # instant this branch first triggered.
            self._estop_stuck_counter += 1
            # Trust raw_side_bias IMMEDIATELY here, not after stuck_limit
            # steps like the softer CBF fallback does. That delay exists so
            # the model gets "a fair chance" before being overridden, which
            # is reasonable when there's still distance to spare -- but this
            # branch by definition only engages inside min_range<0.30, the
            # single most time-critical moment there is. Confirmed in
            # simulation: waiting here cost exactly one stuck_limit's worth
            # of WRONG-direction steering (phi snapped to +0.4, toward the
            # obstacle, for 15 steps before correcting) right as min_range
            # first crossed the threshold -- burning through the last of the
            # distance budget on a stale model belief instead of the
            # always-available raw measurement.
            raw_dir = 1.0 if raw_side_bias >= 0 else -1.0
            if raw_side_bias != 0.0:
                chosen_dir = raw_dir
            else:
                chosen_dir = 1.0 if B_q_hat[1] >= 0 else -1.0
            phi_dir = self.phi_max if chosen_dir > 0 else self.phi_min
            if self._estop_stuck_counter >= self.cbf.stuck_limit:
                v_estop = min(self.cbf.fallback_creep_v, self.v_max)
            else:
                v_estop = 0.0
            u_safe, feasible = [v_estop, phi_dir], True
        else:
            self._estop_stuck_counter = 0
            try:
                u_safe, feasible = self.cbf.compute_safe_control(
                    u_ref=u_ref, q_hat=q_hat, qdot_hat=qdot_hat,
                    F_q_hat=F_q_hat, B_q_hat=B_q_hat, P=P_safety,
                    u_prev=self.u_prev, v_current=self.v,
                    raw_side_bias=raw_side_bias)
            except Exception as e:
                self.get_logger().warn(f'CBF QP raised {e!r}; braking with steering held')
                u_safe, feasible = [0.0, float(self.u_prev[1])], False

        self.send_command(u_safe[0], u_safe[1])
        self.u_prev = np.array(u_safe)

        # Compare against u_ref (the scaled target actually being pursued
        # this cycle), NOT self.u_ref (the raw, unscaled v_ref=1.0 the
        # wall-follower emits) -- comparing against the unscaled value meant
        # this label reported BRAKE for perfectly normal, feasible tracking
        # of a courtesy-slowed target every time the vehicle held a steady,
        # safe distance from a wall (confirmed on hardware: v matched the
        # scaled target almost exactly, phi was small dither not a pinned
        # fallback value -- neither is what BRAKE implies). Computed every
        # step now (not just the 1-in-50 print) so it lands in the CSV log
        # for every row, not just the sparse console summary.
        action = 'ESTOP' if min_range < 0.30 else \
                 ('SAFE' if feasible and abs(u_safe[0] - u_ref[0]) < 0.1 else
                  ('STEER' if abs(u_safe[1] - u_ref[1]) > 0.05 else 'BRAKE'))

        # sigma_k (Theorem 2 confidence margin, Eq. 12-14) isn't returned by
        # compute_safe_control -- recompute it here against the SAME
        # reachable box (effective_bounds(self.u_prev), now updated to this
        # step's applied command) purely for logging; cheap (a handful of
        # box-vertex evaluations), not on any control-critical path.
        sigma_k, _ = self.cbf.compute_safety_margin(P_safety, *self.cbf.effective_bounds(self.u_prev))

        now_s = self.get_clock().now().nanoseconds * 1e-9
        self._control_log_w.writerow([
            f'{now_s:.6f}', self._step_count, f'{dt_sub:.5f}',
            f'{self.x:.4f}', f'{self.y:.4f}', f'{self.theta:.5f}', f'{self.v:.3f}',
            f'{u_ref[0]:.3f}', f'{u_ref[1]:.4f}', f'{u_safe[0]:.3f}', f'{u_safe[1]:.4f}',
            f'{q_hat:.4f}', f'{qdot_hat:.4f}', f'{F_q_hat:.4f}', f'{B_q_hat[0]:.4f}', f'{B_q_hat[1]:.4f}',
            f'{sigma_k:.4f}', f'{min_range:.3f}', int(feasible), self.cbf.N, action,
        ])

        if self._step_count % 50 == 0:
            self._control_log_f.flush()
            self._scan_log_f.flush()
            total_time = time.time() - start_time
            self.get_logger().info(
                f'[{action}] {total_time*1000:.0f}ms | feas={feasible} | q={q_hat:.2f} | min_range={min_range:.2f} | '
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

    def destroy_node(self):
        """Flush and close the run logs before shutdown -- main()'s finally
        block calls this on both a clean exit and a KeyboardInterrupt, so
        this is the one place guaranteed to run at the end of a run."""
        for f in (getattr(self, '_control_log_f', None), getattr(self, '_scan_log_f', None)):
            if f is not None:
                try:
                    f.close()
                except Exception:
                    pass
        super().destroy_node()


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
