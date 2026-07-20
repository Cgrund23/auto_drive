#!/usr/bin/env python3
"""
cbf_Node_refactored_ackermann.py

Ackermann-native replacement for cbf_Node_refactored.py.

GROUND TRUTH: L.A. Duffaut Espinosa & C. Grund, "Safety via Control Barrier
Functions Synthesized from Ultra-Local Models" ("the paper"). Every equation
cited in comments below refers to that paper. This node, and
CBF_refactored_ackermann.py that it imports, were derived from (and are
validated against) ackermann_cbf_core.py / run_ackermann_sim.py in this same
delivery, which reproduce the paper's Section V numerical-illustration
methodology on true Ackermann kinematics before anything was ported to ROS.

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
   obstacle whose boundary generated the reflection." CBF_refactored_
   ackermann.ModelFreeCBF.set_obstacles() does a simple angular-gap
   clustering step to route points to per-obstacle GPs, which persist (with a
   bounded point buffer) across scans in the WORLD frame using odometry,
   rather than being rebuilt from scratch every callback.

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
   CBF_refactored_ackermann.ModelFreeCBF.compute_safe_control()'s comment for
   the failure mode this fixes.

6. No CuPy. The linear algebra here is all sub-10x10 dense matrices (EKF
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

from auto_drive.CBF_refactored_ackermann import ModelFreeCBF


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

        self._last_reset_time = -1e9
        self.reset_count = 0

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

    def reset_if_diverged(self, t):
        # B_q,v is sign-indefinite by design (see module docstring point 3),
        # so divergence is judged on magnitude / numerical sanity only.
        B_qv = float(self.x[3])
        if (abs(B_qv) > 6.0 or not np.isfinite(B_qv)) and (t - self._last_reset_time) > 1.0:
            self.x[2] = 0.0
            self.x[3] = 1.0
            if self.m > 1:
                self.x[4] = 0.0
            self.P = np.diag([0.01, 0.01, 0.05] + [0.05] * self.m)
            self._last_reset_time = t
            self.reset_count += 1
            return True
        return False

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
        self.v_min, self.v_max = 0.0, 1.2
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

        self.u_ref = np.array([0.8, 0.0])     # [v_ref, phi_ref]
        self.u_prev = np.array([0.8, 0.0])
        self.v_prev = 0.8

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
        v_ref, phi_ref = 0.8, 0.0
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
        n_obstacles = self.cbf.N
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

        ekf_just_reset = self.safety_ekf.reset_if_diverged(time.time())
        if ekf_just_reset:
            self.get_logger().warn(
                f'EKF diverged! Resetting. Recent: q={q_meas:.3f}, qdot={qdot_meas:.3f}, n_obs={n_obstacles}')

        q_hat, qdot_hat, F_q_hat, B_q_hat, P_safety = self.safety_ekf.get_estimates()
        if ekf_just_reset:
            q_hat = min(q_hat, 0.5)

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
                    F_q_hat=F_q_hat, B_q_hat=B_q_hat, P=P_safety, v_current=self.v)
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
