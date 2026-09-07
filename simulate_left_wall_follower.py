#!/usr/bin/env python3
"""
Standalone (no ROS required) simulation of the actual left-wall-follower +
Model-Free CBF pipeline from auto_drive/cbf_Node_refactored.py.

This does NOT reimplement the safety filter -- it imports ModelFreeCBF,
SafetyULM_EKF, and PositionULM_EKF directly from cbf_Node_refactored.py (the
real, currently-tuned production code) via a small rclpy/message stub, since
this machine has no ROS install. SimNode below mirrors ControllerNode's
lidar_callback/center_lock_controller/_apply_persistent_excitation
logic line-for-line, with the same parameter values as the real node, just
swapping ROS pub/sub for a simulated Ackermann bicycle model and a ray-cast
LiDAR in a synthetic hallway-with-obstacles world.

Usage: python3 simulate_left_wall_follower.py
Outputs: left_wall_follower_simulation.png (summary) and
         left_wall_follower_simulation.gif (animation), both in this dir.
"""
import sys
import types

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle

# --- stub out the ROS imports cbf_Node_refactored.py needs at module level
# (rclpy/nav_msgs/sensor_msgs/ackermann_msgs) so we can import the real
# ModelFreeCBF / SafetyULM_EKF / PositionULM_EKF classes on a machine with no
# ROS install. None of these stubs are ever actually called: we don't
# instantiate ControllerNode or run main() below.
def _stub_module(name, **attrs):
    m = types.ModuleType(name)
    for k, v in attrs.items():
        setattr(m, k, v)
    sys.modules[name] = m
    return m


class _StubNode:
    def __init__(self, *a, **k):
        pass

    def create_subscription(self, *a, **k):
        pass

    def create_publisher(self, *a, **k):
        return None

    def get_logger(self):
        class _L:
            def info(self, *a, **k):
                pass

            def warn(self, *a, **k):
                pass
        return _L()


_stub_module('rclpy', init=lambda *a, **k: None, shutdown=lambda *a, **k: None)
_stub_module('rclpy.node', Node=_StubNode)
_stub_module('rclpy.executors', MultiThreadedExecutor=object)
_stub_module('nav_msgs')
_stub_module('nav_msgs.msg', Odometry=object)
_stub_module('sensor_msgs')
_stub_module('sensor_msgs.msg', LaserScan=object)
_stub_module('ackermann_msgs')
_stub_module('ackermann_msgs.msg', AckermannDriveStamped=object)

sys.path.insert(0, 'auto_drive')
from cbf_Node_refactored import ModelFreeCBF, SafetyULM_EKF, PositionULM_EKF  # noqa: E402

np.random.seed(0)


# ============================================================================
# World: a straight hallway with circular obstacles to slalom around
# ============================================================================
HALLWAY_LENGTH = 10.0
# Real hallway is 2.0m wide (was 1.0m, and 2.4m before that -- both earlier
# guesses). This matters a lot for length_scale/left_wall_setpoint in
# cbf_Node_refactored.py -- both are spatial and need to track the actual
# corridor width.
HALLWAY_HALF_WIDTH = 1.0     # walls at y = +/- HALLWAY_HALF_WIDTH

# Real vehicle: 8in x 16in (0.2032m x 0.4064m). ROBOT_RADIUS is the GROUND
# TRUTH used by check_collision() below -- it should be the vehicle's actual
# physical extent, not a safety-margin-padded number (the padding lives in
# cbf_Node_refactored.py's r_buf instead, which is the CONTROLLER's belief
# about where the boundary is, deliberately more conservative than reality).
# Using half-WIDTH (not half-length or half-diagonal) treats the vehicle as
# a circle sized by its narrow dimension -- correct while it's driving
# roughly straight (its usual attitude, since this is a corridor-following
# task) but an UNDERESTIMATE during a large heading excursion, when the
# vehicle presents closer to its long dimension to the corridor's width.
# This session has repeatedly produced 60-90 deg excursions during obstacle
# avoidance, so this is a real, unresolved conservatism gap, not a
# hypothetical one -- flagging rather than silently picking the more
# conservative half-diagonal (0.227m), which would also change collision
# outcomes throughout the rest of this file.
ROBOT_RADIUS = 0.2032 / 2   # = 0.1016m, vehicle half-width

# Obstacle sits ON the nominal center-lock line (y=0) so the car is forced
# to actually deviate from centering to get around it, then return to
# center afterward.
OBSTACLES = [(3.0, 0.0, 0.10)]

N_RAYS = 1080
ANGLE_MIN, ANGLE_MAX = np.deg2rad(-135.0), np.deg2rad(135.0)
MAX_RANGE = 3.0
BODY_ANGLES = np.linspace(ANGLE_MIN, ANGLE_MAX, N_RAYS)


def simulate_lidar(x, y, theta, obstacles=None):
    """Vectorized ray-cast against the two hallway walls + all obstacles.
    Mirrors a real 2-D LiDAR: returns full-resolution (ranges, angles) in the
    robot body frame, capped at MAX_RANGE. `obstacles` defaults to the
    module-level OBSTACLES layout; pass a different list (e.g. from
    randomize_hallway.py) to test other layouts without editing this file."""
    obstacles = OBSTACLES if obstacles is None else obstacles
    world_angles = theta + BODY_ANGLES
    dx, dy = np.cos(world_angles), np.sin(world_angles)
    best = np.full(N_RAYS, MAX_RANGE)

    for wall_y in (HALLWAY_HALF_WIDTH, -HALLWAY_HALF_WIDTH):
        with np.errstate(divide='ignore', invalid='ignore'):
            t = (wall_y - y) / dy
        xhit = x + t * dx
        valid = (np.abs(dy) > 1e-9) & (t > 0) & (t <= best) & \
                (xhit >= -0.5) & (xhit <= HALLWAY_LENGTH + 0.5)
        best = np.where(valid, t, best)

    for (cx, cy, cr) in obstacles:
        fx, fy = x - cx, y - cy
        b = 2 * (fx * dx + fy * dy)
        c = fx * fx + fy * fy - cr * cr
        disc = b * b - 4 * c
        sqrt_disc = np.sqrt(np.clip(disc, 0, None))
        for t in ((-b - sqrt_disc) / 2, (-b + sqrt_disc) / 2):
            valid = (disc >= 0) & (t > 0) & (t <= best)
            best = np.where(valid, t, best)

    return best, BODY_ANGLES


def check_collision(x, y, obstacles=None):
    obstacles = OBSTACLES if obstacles is None else obstacles
    for wall_y in (HALLWAY_HALF_WIDTH, -HALLWAY_HALF_WIDTH):
        if abs(wall_y - y) < ROBOT_RADIUS:
            return True
    for (cx, cy, cr) in obstacles:
        if np.hypot(x - cx, y - cy) < cr + ROBOT_RADIUS:
            return True
    return False


# ============================================================================
# SimNode: same control loop as ControllerNode in cbf_Node_refactored.py,
# with identical tuned parameters, minus the ROS plumbing.
# ============================================================================
class SimNode:
    def __init__(self):
        # Real scan period, not the 0.01 (100 Hz) this file originally
        # assumed -- measured directly from timestamps in a real hardware
        # log: 650 lidar_callback invocations spanned 17.14s, i.e. ~26.4ms
        # (~38 Hz) per scan. See cbf_Node_refactored.py's matching comment.
        self.dt = 0.0264
        self.n_control_substeps = 3    # see step()/_control_step() below
        self.L = 0.27   # estimated wheelbase from 16in vehicle length; see cbf_Node_refactored.py
        self.v_min, self.v_max = 0.0, 1.2
        self.phi_min, self.phi_max = -0.4, 0.4
        self.r_max = 3.0
        self.length_scale = 0.20
        self.sigma_f = 1.0
        self.r_buf = 0.1016 + 0.05    # vehicle half-width (8in/2) + noise margin
        self.lambda_0 = 2.5
        self.lambda_1 = 2.5
        self.c_q = 1.1

        self.x, self.y, self.theta, self.v = 0.3, 0.50, 0.0, 0.0

        self.u_ref = np.array([1.0, 0.0])
        self.u_prev = np.array([1.0, 0.0])
        self.v_prev = 1.0

        self.wall_beam_angle = np.deg2rad(30.0)
        self.wall_beam_separation = np.deg2rad(15.0)
        self.wall_follow_kp = 0.075   # halved for center-lock; see cbf_Node_refactored.py
        self.wall_follow_kd = 0.15
        self._wall_follow_prev_error = 0.0
        self.wall_follow_heading_fade_start = np.deg2rad(30.0)
        self.wall_follow_heading_fade_end = np.deg2rad(70.0)
        self.heading_correction_kp = 0.6   # see matching comment in cbf_Node_refactored.py

        self._dither_ampl = 0.05
        self._dither_period_steps = 30
        self._step_count = 0
        self._excite_counter = -1
        self._excite_steps = 24
        self._estop_stuck_counter = 0

        self.safety_ekf = SafetyULM_EKF(Ts=self.dt, m_inputs=2)
        self.position_ekf = PositionULM_EKF(Ts=self.dt, m_inputs=2)
        self.cbf = ModelFreeCBF(
            dt=self.dt, u_min=[self.v_min, self.phi_min], u_max=[self.v_max, self.phi_max],
            r_max=self.r_max, r_min_obstacle=self.r_max, length_scale=self.length_scale,
            sigma_f=self.sigma_f, lambda_0=self.lambda_0, lambda_1=self.lambda_1,
            c_q=self.c_q, r_buf=self.r_buf,
        )
        self.last_ranges = np.array([])
        self.last_angles = np.array([])

    # -- exact port of ControllerNode._side_wall_distance -------------------
    def _side_wall_distance(self, sign, ranges, angles):
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

    # -- exact port of ControllerNode.center_lock_controller ----------------
    def center_lock_controller(self):
        v_ref = 1.0
        if len(self.last_angles) == 0:
            return np.array([v_ref, 0.0])

        ranges, angles = self.last_ranges, self.last_angles
        Dt_left = self._side_wall_distance(1.0, ranges, angles)
        Dt_right = self._side_wall_distance(-1.0, ranges, angles)
        if Dt_left is None or Dt_right is None:
            return np.array([v_ref, 0.0])

        error = Dt_left + Dt_right
        d_error = error - self._wall_follow_prev_error
        self._wall_follow_prev_error = error

        phi_ref = self.wall_follow_kp * error + self.wall_follow_kd * d_error
        phi_ref = float(np.clip(phi_ref, self.phi_min, self.phi_max))

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

    # -- exact port of ControllerNode._apply_persistent_excitation ---------
    def _apply_persistent_excitation(self, u_ref):
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

    # -- exact port of ControllerNode.lidar_callback ------------------------
    def step(self, ranges, angles):
        """Perception once per (simulated) scan, then the control law runs
        n_control_substeps times against it -- mirrors cbf_Node_refactored.py's
        lidar_callback/_control_step split; see that method's docstring for
        why re-running the control law without new perception is legitimate
        (q_meas/qdot_meas come from the persistent, already-fitted per-
        obstacle GPs evaluated at the evolving position estimate, not from
        the scan directly)."""
        self.last_ranges, self.last_angles = ranges, angles

        self.cbf.set_obstacles(ranges, angles, robot_xy=(self.x, self.y),
                                robot_theta=self.theta)
        valid_ranges = ranges[(ranges > 0.1) & (ranges < self.r_max)]

        left_mask, right_mask = angles > 0, angles < 0
        left_min = float(np.min(ranges[left_mask])) if np.any(left_mask) else self.r_max
        right_min = float(np.min(ranges[right_mask])) if np.any(right_mask) else self.r_max
        raw_side_bias = left_min - right_min

        dt_sub = self.dt / self.n_control_substeps
        result = None
        for _ in range(self.n_control_substeps):
            result = self._control_step(valid_ranges, raw_side_bias, dt_sub)
        return result

    def _control_step(self, valid_ranges, raw_side_bias, dt_sub):
        self._step_count += 1

        self.cbf.dt = dt_sub
        self.safety_ekf.Ts = dt_sub
        self.position_ekf.Ts = dt_sub
        self.cbf.stuck_limit = max(1, int(0.15 / dt_sub))
        self.cbf.fallback_dither_period = max(1, int(0.3 / dt_sub))

        p_world = np.array([self.x, self.y])

        self.safety_ekf.predict(self.u_prev)
        self.position_ekf.predict(self.u_prev)
        self.position_ekf.update(p_world)

        if self._step_count % 3 == 0:
            self.u_ref = self.center_lock_controller()
        u_ref = self._apply_persistent_excitation(self.u_ref)

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
        if np.isfinite(qdot_meas) and abs(qdot_meas) < 10.0:
            self.safety_ekf.update_qdot(qdot_meas, R_qdot=max(float(R_qdot), 1e-4))

        q_hat, qdot_hat, F_q_hat, B_q_hat, P_safety = self.safety_ekf.get_estimates()

        u_ref = u_ref.copy()
        u_ref[0] *= float(np.clip((q_hat if np.isfinite(q_hat) else 1.0) / 0.6, 0.15, 1.0))

        min_range = float(np.min(valid_ranges)) if len(valid_ranges) > 0 else 999.0
        if min_range < 0.30:
            self._estop_stuck_counter += 1
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
            except Exception:
                u_safe, feasible = [0.0, float(self.u_prev[1])], False

        # Ackermann bicycle model in place of publishing /drive.
        self.v = u_safe[0]
        self.x += u_safe[0] * np.cos(self.theta) * dt_sub
        self.y += u_safe[0] * np.sin(self.theta) * dt_sub
        self.theta += (u_safe[0] / self.L) * np.tan(u_safe[1]) * dt_sub
        self.theta = float(np.arctan2(np.sin(self.theta), np.cos(self.theta)))
        self.u_prev = np.array(u_safe)

        return dict(x=self.x, y=self.y, theta=self.theta, v=self.v,
                    q=q_hat, B_q=B_q_hat.copy(), phi=u_safe[1],
                    v_cmd=u_safe[0], min_range=min_range, feasible=feasible,
                    n_obstacles=self.cbf.N)


def barrier_field(cbf, xs, ys):
    """Evaluate the SAME aggregated soft-min barrier the QP uses,
    cbf.get_barrier_and_variance, over a grid -- this is literally 'all the
    barriers the car sees', combined exactly as the safety filter combines
    them (Eq. 3 soft-min)."""
    Q = np.zeros((len(ys), len(xs)))
    for j, yy in enumerate(ys):
        for i, xx in enumerate(xs):
            Q[j, i] = cbf.get_barrier_and_variance(np.array([xx, yy]))[0]
    return Q


def main():
    sim = SimNode()
    dt = sim.dt
    max_steps = 3500

    hist = {k: [] for k in ('t', 'x', 'y', 'theta', 'v', 'v_cmd', 'phi',
                             'q', 'Bqv', 'Bqphi', 'min_range', 'n_obstacles')}
    scan_snapshots = []   # (x, y, theta, world-frame hit points) at a few times

    status = "RUNNING"
    for step in range(max_steps):
        ranges_full, _ = simulate_lidar(sim.x, sim.y, sim.theta)
        ranges = ranges_full[::20].astype(np.float32)
        angles = BODY_ANGLES[::20].astype(np.float32)

        result = sim.step(ranges, angles)

        hist['t'].append(step * dt)
        hist['x'].append(result['x'])
        hist['y'].append(result['y'])
        hist['theta'].append(result['theta'])
        hist['v'].append(result['v'])
        hist['v_cmd'].append(result['v_cmd'])
        hist['phi'].append(result['phi'])
        hist['q'].append(result['q'])
        hist['Bqv'].append(result['B_q'][0])
        hist['Bqphi'].append(result['B_q'][1])
        hist['min_range'].append(result['min_range'])
        hist['n_obstacles'].append(result['n_obstacles'])

        if step % 150 == 0:
            wx = sim.x + ranges * np.cos(sim.theta + angles)
            wy = sim.y + ranges * np.sin(sim.theta + angles)
            hit = ranges < (MAX_RANGE - 1e-3)
            scan_snapshots.append((sim.x, sim.y, sim.theta, wx[hit], wy[hit]))

        if check_collision(result['x'], result['y']):
            status = f"COLLISION at t={step*dt:.2f}s, x={result['x']:.2f}"
            print(status)
            break
        if result['x'] > HALLWAY_LENGTH - 0.3:
            status = f"REACHED END at t={step*dt:.2f}s"
            print(status)
            break
    else:
        status = f"TIMED OUT after {max_steps*dt:.1f}s, x={hist['x'][-1]:.2f}"
        print(status)

    n_steps = len(hist['t'])
    print(f"Ran {n_steps} steps ({n_steps*dt:.2f}s sim time), "
          f"final x={hist['x'][-1]:.2f}m, min q={min(hist['q']):.3f}, "
          f"obstacles tracked={sim.cbf.N}")

    # ------------------------------------------------------------------
    # Static summary figure
    # ------------------------------------------------------------------
    fig = plt.figure(figsize=(16, 9))
    gs = fig.add_gridspec(2, 3, width_ratios=[1.6, 1, 1])
    ax_map = fig.add_subplot(gs[:, 0])

    ax_map.axhline(HALLWAY_HALF_WIDTH, color='k', linewidth=3)
    ax_map.axhline(-HALLWAY_HALF_WIDTH, color='k', linewidth=3)
    ax_map.set_xlim(-0.5, HALLWAY_LENGTH + 0.5)
    ax_map.set_ylim(-HALLWAY_HALF_WIDTH - 0.4, HALLWAY_HALF_WIDTH + 0.4)

    xs = np.linspace(-0.5, HALLWAY_LENGTH + 0.5, 140)
    ys = np.linspace(-HALLWAY_HALF_WIDTH - 0.4, HALLWAY_HALF_WIDTH + 0.4, 50)
    Q = barrier_field(sim.cbf, xs, ys)
    levels = np.linspace(-2.0, 1.0, 25)
    cf = ax_map.contourf(xs, ys, Q, levels=levels, cmap='RdYlGn', alpha=0.65, extend='both')
    ax_map.contour(xs, ys, Q, levels=[0.0], colors='black', linewidths=2.0)
    fig.colorbar(cf, ax=ax_map, label='aggregated barrier q (green=safe, red=unsafe)', shrink=0.7)

    for (cx, cy, cr) in OBSTACLES:
        ax_map.add_patch(Circle((cx, cy), cr, facecolor='dimgray', edgecolor='k', zorder=3))

    colors = plt.cm.tab10(np.linspace(0, 1, max(len(sim.cbf.gps), 1)))
    for (obs_id, gp), c in zip(sim.cbf.gps.items(), colors):
        if gp.n_points > 0:
            ax_map.scatter(gp.P[:, 0], gp.P[:, 1], s=14, color=c, zorder=4,
                            label=f'GP #{obs_id} scan points ({gp.n_points})')

    for (sx, sy, sth, wx, wy) in scan_snapshots:
        ax_map.scatter(wx, wy, s=3, color='dodgerblue', alpha=0.25, zorder=2)

    ax_map.plot(hist['x'], hist['y'], color='navy', linewidth=2, zorder=5, label='robot path')
    ax_map.plot(hist['x'][0], hist['y'][0], 'go', markersize=10, zorder=6, label='start')
    ax_map.plot(hist['x'][-1], hist['y'][-1], 'r^', markersize=10, zorder=6, label='end')

    ax_map.set_xlabel('x (m)')
    ax_map.set_ylabel('y (m)')
    ax_map.set_title(f'Left-wall-follower + Model-Free CBF -- {status}')
    ax_map.legend(loc='lower right', fontsize=7)
    ax_map.set_aspect('equal')

    ax_q = fig.add_subplot(gs[0, 1])
    ax_q.plot(hist['t'], hist['q'], color='green')
    ax_q.axhline(0, color='red', linestyle='--', label='unsafe (q<0)')
    ax_q.set_ylabel('barrier q')
    ax_q.set_title('Safety barrier (EKF estimate)')
    ax_q.legend(fontsize=7)
    ax_q.grid(alpha=0.3)

    ax_b = fig.add_subplot(gs[0, 2])
    ax_b.plot(hist['t'], hist['Bqv'], label='B_q,v')
    ax_b.plot(hist['t'], hist['Bqphi'], label='B_q,phi')
    ax_b.axhline(0, color='k', alpha=0.3)
    ax_b.set_title('ULM sensitivity estimates')
    ax_b.legend(fontsize=7)
    ax_b.grid(alpha=0.3)

    ax_u = fig.add_subplot(gs[1, 1])
    ax_u.plot(hist['t'], hist['v_cmd'], label='v (safe)')
    ax_u.plot(hist['t'], hist['phi'], label='phi (safe)')
    ax_u.set_xlabel('t (s)')
    ax_u.set_title('Commanded v, phi')
    ax_u.legend(fontsize=7)
    ax_u.grid(alpha=0.3)

    ax_r = fig.add_subplot(gs[1, 2])
    ax_r.plot(hist['t'], hist['min_range'], color='purple')
    ax_r.axhline(0.30, color='red', linestyle='--', label='e-stop threshold')
    ax_r.set_xlabel('t (s)')
    ax_r.set_title('Closest LiDAR return')
    ax_r.legend(fontsize=7)
    ax_r.grid(alpha=0.3)

    plt.tight_layout()
    out_png = 'left_wall_follower_simulation.png'
    plt.savefig(out_png, dpi=140)
    print(f"Saved {out_png}")

    # ------------------------------------------------------------------
    # Animation: scan sweeping + robot driving through the hallway
    # ------------------------------------------------------------------
    frame_stride = 8
    frame_idx = list(range(0, n_steps, frame_stride))

    fig2, ax2 = plt.subplots(figsize=(11, 4.5))
    ax2.axhline(HALLWAY_HALF_WIDTH, color='k', linewidth=3)
    ax2.axhline(-HALLWAY_HALF_WIDTH, color='k', linewidth=3)
    for (cx, cy, cr) in OBSTACLES:
        ax2.add_patch(Circle((cx, cy), cr, facecolor='dimgray', edgecolor='k', zorder=3))
    ax2.set_xlim(-0.5, HALLWAY_LENGTH + 0.5)
    ax2.set_ylim(-HALLWAY_HALF_WIDTH - 0.4, HALLWAY_HALF_WIDTH + 0.4)
    ax2.set_aspect('equal')
    ax2.set_xlabel('x (m)')
    ax2.set_ylabel('y (m)')

    path_line, = ax2.plot([], [], color='navy', linewidth=2)
    scan_scatter = ax2.scatter([], [], s=6, color='dodgerblue', zorder=4)
    robot_dot, = ax2.plot([], [], 'o', color='orange', markersize=10, zorder=5)
    heading_line, = ax2.plot([], [], color='orange', linewidth=2, zorder=5)
    title = ax2.set_title('')

    def render(k):
        i = frame_idx[k]
        path_line.set_data(hist['x'][:i + 1], hist['y'][:i + 1])
        x, y, th = hist['x'][i], hist['y'][i], hist['theta'][i]
        ranges_full, _ = simulate_lidar(x, y, th)
        ranges = ranges_full[::20]
        angles = BODY_ANGLES[::20]
        hit = ranges < (MAX_RANGE - 1e-3)
        wx = x + ranges[hit] * np.cos(th + angles[hit])
        wy = y + ranges[hit] * np.sin(th + angles[hit])
        scan_scatter.set_offsets(np.column_stack([wx, wy]))
        robot_dot.set_data([x], [y])
        heading_line.set_data([x, x + 0.3 * np.cos(th)], [y, y + 0.3 * np.sin(th)])
        title.set_text(f't={hist["t"][i]:.2f}s  q={hist["q"][i]:.2f}  '
                        f'v={hist["v_cmd"][i]:.2f}  phi={hist["phi"][i]:.2f}')
        return path_line, scan_scatter, robot_dot, heading_line, title

    ani = animation.FuncAnimation(fig2, render, frames=len(frame_idx),
                                   interval=40, blit=False)
    out_gif = 'left_wall_follower_simulation.gif'
    ani.save(out_gif, writer='pillow', fps=20)
    print(f"Saved {out_gif}")


if __name__ == '__main__':
    main()
