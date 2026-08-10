"""
Part 4 (v2): Top-down animated visualization with REACTIVE steering.

Fixes vs. the first version:
  1. u_ref is no longer a fixed [1.0, 0.25] command that marches into the wall.
     It's now generated every step by a port of the real tangent_controller()
     from cbf_Node_refactored.py: goal-seeking heading control + a LiDAR-based
     repulsive bias that steers away from the closest obstacle. This is what
     "favor steering over stopping" actually requires -- the CBF is a safety
     filter on top of an avoidance-seeking reference, not a substitute for one.
  2. Real collision detection: the robot has a physical radius and we check
     it against the wall segment's geometry every step, independent of what
     the CBF/QP "feasible" flag says. QP-feasible only means the point
     barrier constraint at the robot's origin was satisfied for that instant
     -- it does NOT guarantee the swept trajectory clears the obstacle. The
     animation now shows both signals separately so this distinction is
     visible.
  3. The obstacle is now a finite wall segment (not just an x= line) so
     "going around it" is a meaningful, visualizable goal.
"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from cbf_sim_core import (ModelFreeCBF, SafetyULM_EKF, PositionULM_EKF,
                           solve_cbf_qp, solve_cbf_qp_with_slack)

np.random.seed(0)

dt = 0.01
v_max, v_min = 1.5, 0.0
omega_max, omega_min = 0.5, -0.5
r_max = 3.0
length_scale = 0.25
sigma_f = 1.0
lambda_0, lambda_1 = 0.1, 0.1
c_q = 0.01
ROBOT_RADIUS = 0.15  # for collision checking / drawing

# Wall: a finite vertical segment the robot must go around
WALL_X = 2.2
WALL_Y_MIN, WALL_Y_MAX = -0.6, 0.6

GOAL_X, GOAL_Y = 4.0, 0.0  # goal is past the wall -> robot must go around and continue


def wrap(a):
    """Wrap an angle to [-pi, pi]."""
    return float(np.arctan2(np.sin(a), np.cos(a)))


def simulate_lidar(robot_x, robot_y, robot_theta, n_beams=36, fov=np.pi):
    """Cast rays from the robot and intersect with the wall segment."""
    angles = np.linspace(-fov / 2, fov / 2, n_beams)
    kept_r, kept_a = [], []
    for a in angles:
        world_a = robot_theta + a
        dx, dy = np.cos(world_a), np.sin(world_a)
        # Ray: (robot_x + t*dx, robot_y + t*dy), intersect with x = WALL_X
        if abs(dx) < 1e-6:
            continue
        t = (WALL_X - robot_x) / dx
        if t <= 0 or t > r_max:
            continue
        y_hit = robot_y + t * dy
        if WALL_Y_MIN <= y_hit <= WALL_Y_MAX:
            kept_r.append(t)
            kept_a.append(a)
    return np.array(kept_r), np.array(kept_a)


def check_collision(robot_x, robot_y):
    """Distance from robot center to the wall segment; collision if < ROBOT_RADIUS."""
    # Closest point on the vertical segment x=WALL_X, y in [WALL_Y_MIN, WALL_Y_MAX]
    cy = np.clip(robot_y, WALL_Y_MIN, WALL_Y_MAX)
    cx = WALL_X
    dist = np.hypot(robot_x - cx, robot_y - cy)
    return dist < ROBOT_RADIUS, dist


def reactive_reference(robot_x, robot_y, robot_theta, ranges, angles, commit_state=None):
    """
    Goal-seeking heading control + LiDAR-based gap-following avoidance.
    This is the reference the CBF filters, not a straight line into the wall.

    commit_state: optional dict with key 'side' (+1/-1/None) used to persist
    which side of an obstacle the robot has committed to, so it doesn't
    oscillate between "go left" and "go right" when the front-sector gap
    geometry is momentarily ambiguous (e.g. exactly head-on to a symmetric
    wall). Once committed, the bias decays away once the obstacle is no
    longer close, so the robot returns to pure goal-seeking afterward.
    """
    if commit_state is None:
        commit_state = {'side': None, 'steps_since_seen': 0, 'locked_side': None}

    dx = GOAL_X - robot_x
    dy = GOAL_Y - robot_y
    distance_to_goal = float(np.hypot(dx, dy))

    if distance_to_goal < 0.3:
        return [0.0, 0.0]

    desired_theta = float(np.arctan2(dy, dx))
    heading_error = desired_theta - robot_theta
    heading_error = float(np.arctan2(np.sin(heading_error), np.cos(heading_error)))

    K_heading = 2.0
    omega_ref = K_heading * heading_error

    v_ref = 1.0 * (1.0 - 0.5 * abs(heading_error) / np.pi)
    v_ref = max(0.3, v_ref)

    if len(ranges) > 0:
        front_mask = np.abs(angles) < np.pi / 2
        front_ranges = ranges[front_mask]
        front_angles = angles[front_mask]
        if len(front_ranges) > 0:
            closest = float(np.min(front_ranges))
            if closest < 2.2:
                # WALL-FOLLOWING using absolute (world-frame) headings
                # throughout, rather than body-frame "turn amount" arithmetic.
                # Earlier versions mixed the two representations (treating a
                # body-frame offset as if it were an error term, or vice
                # versa), which produced steering commands with the wrong
                # sign at specific geometries -- verified directly: turning
                # in the "committed" direction was empirically making
                # clearance worse, not better, at several traced positions.
                # Computing everything as an absolute world heading and then
                # taking ONE heading-error at the end (the same pattern
                # already used for goal-seeking above) removes that class of
                # bug entirely.
                closest_idx = int(np.argmin(front_ranges))
                closest_angle_body = float(front_angles[closest_idx])
                closest_dist = float(front_ranges[closest_idx])
                closest_angle_world = wrap(robot_theta + closest_angle_body)

                target_standoff = 1.2
                dist_error = closest_dist - target_standoff  # negative = too close

                # Two candidate tangent headings in world frame.
                tangent_world_a = wrap(closest_angle_world + np.pi / 2)
                tangent_world_b = wrap(closest_angle_world - np.pi / 2)

                # HARD LOCK: decide the side exactly ONCE per obstacle
                # encounter -- on the very first call where closest < 1.2 --
                # and reuse that decision every subsequent call until the
                # obstacle is no longer close (commit_state cleared below).
                # Deciding fresh each call (even with a tie-breaking
                # heuristic) let the choice oscillate step to step whenever
                # the two tangent options were nearly equal cost, since
                # "nearly equal" can flip which one is nominally smaller from
                # one LiDAR sample to the next. A one-time decision, made and
                # then never revisited for the duration of this encounter,
                # cannot oscillate by construction.
                if commit_state.get('locked_side') is None:
                    if abs(wrap(tangent_world_a - desired_theta)) <= abs(wrap(tangent_world_b - desired_theta)):
                        commit_state['locked_side'] = 1.0
                    else:
                        commit_state['locked_side'] = -1.0
                chosen_tangent = tangent_world_a if commit_state['locked_side'] > 0 else tangent_world_b

                # Standoff correction: blend the tangent heading with the
                # heading pointing directly away from the obstacle, weighted
                # by how far off the target standoff we are. This is a
                # simple, unambiguous interpolation between two headings
                # (rather than a signed rotation, which was the source of
                # earlier bugs) -- blend=0 at target standoff (pure tangent),
                # blend->1 as the robot gets closer than target standoff
                # (pulls toward pointing straight away).
                away_from_obstacle = wrap(closest_angle_world + np.pi)
                blend = float(np.clip(-dist_error / target_standoff, 0.0, 1.0))
                # Interpolate headings via their unit vectors to avoid angle-
                # wrapping artifacts from directly averaging angles.
                vx = (1 - blend) * np.cos(chosen_tangent) + blend * np.cos(away_from_obstacle)
                vy = (1 - blend) * np.sin(chosen_tangent) + blend * np.sin(away_from_obstacle)
                desired_world_heading = float(np.arctan2(vy, vx))

                heading_err_to_tangent = wrap(desired_world_heading - robot_theta)
                K_tangent = 1.5
                omega_ref = K_tangent * heading_err_to_tangent
                v_ref = max(0.4, v_ref * min(1.0, closest / 2.2))

                commit_state['side'] = 1.0 if chosen_tangent == tangent_world_a else -1.0
                commit_state['steps_since_seen'] = 0
        else:
            commit_state['steps_since_seen'] += 1
            if commit_state['steps_since_seen'] > 50:
                commit_state['side'] = None
                commit_state['locked_side'] = None

    omega_ref = max(-0.5, min(0.5, omega_ref))
    return [v_ref, omega_ref]


def run_full_trajectory(use_fixes=False, n_steps=1400):
    cbf = ModelFreeCBF(dt=dt, u_min=[v_min, omega_min], u_max=[v_max, omega_max],
                        r_max=r_max, r_min_obstacle=r_max, length_scale=length_scale,
                        sigma_f=sigma_f, lambda_0=lambda_0, lambda_1=lambda_1, c_q=c_q)
    safety_ekf = SafetyULM_EKF(Ts=dt, m_inputs=2)
    position_ekf = PositionULM_EKF(Ts=dt, m_inputs=2)

    robot_x, robot_y, robot_theta = 0.0, 0.0, 0.0
    u_prev = [0.5, 0.0]
    v_prev = 0.5
    infeasible_streak = 0
    collided = False
    collision_step = None
    commit_state = {'side': None, 'steps_since_seen': 0, 'locked_side': None}

    traj = {k: [] for k in ['x', 'y', 'theta', 'q', 'v_cmd', 'omega_cmd',
                             'feasible', 'obstacle_pts_world', 'min_range',
                             'collided', 'wall_dist']}

    for step in range(n_steps):
        t = step * dt
        ranges, angles = simulate_lidar(robot_x, robot_y, robot_theta)
        cbf.set_obstacles(ranges, angles)
        n_obstacles = cbf.N

        safety_ekf.predict(u_prev)
        position_ekf.predict(u_prev)

        q_meas, sigma_gp_sq = cbf.get_barrier_and_variance([0.0, 0.0])
        grad_h = cbf.get_gradient([0.0, 0.0])
        p_est, F_p, B_p = position_ekf.get_estimates()
        p_dot = F_p + B_p @ np.array(u_prev)
        qdot_meas = float(grad_h @ p_dot)

        grad_norm = float(np.linalg.norm(grad_h))
        R_qdot = grad_norm**2 * 0.01**2 + sigma_gp_sq / (length_scale**2 * (grad_norm**2 + 1e-6))
        R_qdot = min(R_qdot, 1.0)

        if not np.isnan(q_meas) and not np.isinf(q_meas):
            safety_ekf.update_q(float(q_meas), R_q=max(float(sigma_gp_sq), 1e-4))
        if not np.isnan(qdot_meas) and not np.isinf(qdot_meas) and abs(qdot_meas) < 10.0:
            safety_ekf.update_qdot(qdot_meas, R_qdot=max(float(R_qdot), 1e-4))

        reset_threshold = 0.35 if use_fixes else 0.2
        B_q_v = float(safety_ekf.x[3])
        ekf_reset = False
        if (B_q_v < reset_threshold or B_q_v > 5.0) and (t - safety_ekf._last_reset_time) >= 1.0:
            safety_ekf.x[2] = 0.0
            safety_ekf.x[3] = 1.0
            safety_ekf.x[4] = 0.1
            safety_ekf.P = np.diag([0.01, 0.01, 0.1, 0.1, 0.1])
            safety_ekf._last_reset_time = t
            ekf_reset = True

        q_hat, qdot_hat, F_q_hat, B_q_hat, P_safety = safety_ekf.get_estimates()
        if ekf_reset:
            q_hat = min(q_hat, 0.5)

        # Safety-gate clearance: use only a narrow forward cone, not the full
        # +-90 deg sensing sector. A wall edge that's abeam of the robot
        # (perpendicular-ish, not in its direction of travel) will always
        # read close during a tangential pass alongside it; gating velocity
        # on that reading stalls the robot exactly when it's successfully
        # skimming past an obstacle rather than approaching it.
        if len(ranges) > 0:
            forward_cone = np.abs(angles) < np.radians(20)
            forward_ranges = ranges[forward_cone]
            min_range = float(np.min(forward_ranges)) if len(forward_ranges) > 0 else 999.0
        else:
            min_range = 999.0
        # Still track true closest-approach (any direction) for collision bookkeeping
        min_range_any = float(np.min(ranges)) if len(ranges) > 0 else 999.0

        # Reference now comes from the reactive controller, not a fixed command
        u_ref_base = reactive_reference(robot_x, robot_y, robot_theta, ranges, angles, commit_state)
        u_ref_eff = list(u_ref_base)
        if use_fixes and infeasible_streak >= 5:
            cap = max(0.2, 1.0 - 0.1 * (infeasible_streak - 4))
            u_ref_eff[0] = min(u_ref_eff[0], cap)

        feasible = True
        # Effective clearance for velocity scaling: the tighter of (a) what's
        # ahead in the direction of travel, and (b) what's nearby in any
        # direction (which catches a wall grazing the robot's flank while it
        # creeps forward along a tangent -- exactly the collision mode seen
        # when only the forward cone was used to gate speed).
        effective_clearance = min(min_range, min_range_any)

        if effective_clearance < 0.6:
            # Genuine close-proximity stop. With the reference controller now
            # starting to steer away at 2.2m and targeting a 1.2m standoff,
            # this branch should essentially never trigger in normal
            # operation -- it exists purely as a backstop, not as the
            # primary avoidance mechanism. No crawl-forward here: the fix for
            # "gets too close" is turning away earlier (see target_standoff
            # and the 2.2m trigger above), not creeping closer at reduced
            # speed. Steering is preserved so the robot can still rotate to
            # open up clearance if this backstop is ever reached.
            u_safe = [0.0, u_ref_eff[1]]
        elif q_hat > 0.3 and n_obstacles == 0:
            u_safe = u_ref_eff
        elif q_hat < -0.5:
            u_safe = [min(u_ref_eff[0], 0.5), u_ref_eff[1]]
        else:
            v_scale = 1.0 + 0.5 * (v_prev / v_max)
            sigma_k = cbf.compute_safety_margin(P_safety, cbf.u_max) * v_scale
            r_k = (-F_q_hat - (lambda_0 + lambda_1) * qdot_hat
                   - lambda_0 * lambda_1 * q_hat + sigma_k)
            a_vec = -np.asarray(B_q_hat)
            b_val = -r_k

            if use_fixes:
                feasible_pre, M_k, _ = cbf.check_feasibility(
                    q_hat, qdot_hat, F_q_hat, B_q_hat, sigma_k)
                if not feasible_pre:
                    shortfall = r_k - M_k
                    u_ref_eff = [max(0.0, u_ref_eff[0] - shortfall * 0.5), u_ref_eff[1]]
                u_safe, slack = solve_cbf_qp_with_slack(
                    cbf._P_qp, u_ref_eff, a_vec, b_val, cbf.u_min, cbf.u_max)
                feasible = slack <= 1e-6
            else:
                sol = solve_cbf_qp(cbf._P_qp, u_ref_eff, a_vec, b_val, cbf.u_min, cbf.u_max)
                if sol is None:
                    u_safe = [float(cbf.u_min[0]), 0.0]
                    feasible = False
                else:
                    u_safe = [float(sol[0]), float(sol[1])]

        infeasible_streak = 0 if feasible else infeasible_streak + 1

        v_cmd, omega_cmd = u_safe
        robot_theta += omega_cmd * dt
        new_x = robot_x + v_cmd * np.cos(robot_theta) * dt
        new_y = robot_y + v_cmd * np.sin(robot_theta) * dt

        is_colliding, wall_dist = check_collision(new_x, new_y)
        if is_colliding and not collided:
            collided = True
            collision_step = step

        robot_x, robot_y = new_x, new_y
        position_ekf.update(np.array([robot_x, robot_y]))
        u_prev = u_safe
        v_prev = v_cmd

        if cbf.N > 0:
            pts_r = cbf.obstacle_points.copy()
            c, s = np.cos(robot_theta), np.sin(robot_theta)
            R = np.array([[c, -s], [s, c]])
            pts_w = (R @ pts_r.T).T + np.array([robot_x, robot_y])
        else:
            pts_w = np.zeros((0, 2))

        traj['x'].append(robot_x); traj['y'].append(robot_y); traj['theta'].append(robot_theta)
        traj['q'].append(q_hat); traj['v_cmd'].append(v_cmd); traj['omega_cmd'].append(omega_cmd)
        traj['feasible'].append(feasible); traj['obstacle_pts_world'].append(pts_w)
        traj['min_range'].append(min_range); traj['collided'].append(collided)
        traj['wall_dist'].append(wall_dist)

        # Stop once we've cleared the wall and are heading to goal, or collided+settled
        if robot_x > GOAL_X + 0.3 or (collided and step > collision_step + 100):
            break

    for k in ['x', 'y', 'theta', 'q', 'v_cmd', 'omega_cmd', 'feasible', 'min_range',
              'collided', 'wall_dist']:
        traj[k] = np.array(traj[k])
    traj['final_collided'] = collided
    traj['collision_step'] = collision_step
    return traj


print("Running baseline trajectory (reactive steering, no fixes)...")
traj_base = run_full_trajectory(use_fixes=False)
n_inf_base = np.sum(~traj_base['feasible'])
print(f"  {len(traj_base['x'])} steps, {n_inf_base} infeasible, "
      f"collided={traj_base['final_collided']}, min_wall_dist={np.min(traj_base['wall_dist']):.3f}")

print("Running fixed trajectory (reactive steering, with fixes)...")
traj_fixed = run_full_trajectory(use_fixes=True)
n_inf_fixed = np.sum(~traj_fixed['feasible'])
print(f"  {len(traj_fixed['x'])} steps, {n_inf_fixed} infeasible, "
      f"collided={traj_fixed['final_collided']}, min_wall_dist={np.min(traj_fixed['wall_dist']):.3f}")

# ---- Animation ----
STRIDE = 4
n_frames = min(len(traj_base['x']), len(traj_fixed['x'])) // STRIDE

fig, (axL, axR) = plt.subplots(1, 2, figsize=(13, 6.5))

def setup_ax(ax, title):
    ax.set_xlim(-0.5, 4.5)
    ax.set_ylim(-1.5, 3.0)
    ax.set_aspect('equal')
    ax.plot([WALL_X, WALL_X], [WALL_Y_MIN, WALL_Y_MAX], color='k', linewidth=5, alpha=0.8, zorder=3)
    ax.plot(GOAL_X, GOAL_Y, marker='*', markersize=18, color='gold', markeredgecolor='k', zorder=3)
    ax.set_title(title)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.grid(alpha=0.25)

setup_ax(axL, "Baseline (as-shipped)")
setup_ax(axR, "With fixes (slack + precheck + early reset)")

def make_artists(ax):
    robot_circle = plt.Circle((0, 0), ROBOT_RADIUS, color='#1f77b4', zorder=5)
    ax.add_patch(robot_circle)
    heading_line, = ax.plot([], [], '-', color='white', linewidth=2, zorder=6)
    path_line, = ax.plot([], [], '-', color='#1f77b4', alpha=0.35, linewidth=1.5)
    obs_scatter = ax.scatter([], [], c='red', s=12, zorder=4, alpha=0.6)
    status_text = ax.text(0.02, 0.97, '', transform=ax.transAxes, va='top', fontsize=9,
                           family='monospace',
                           bbox=dict(boxstyle='round', facecolor='white', alpha=0.9))
    return dict(robot=robot_circle, heading=heading_line, path=path_line,
                obs=obs_scatter, text=status_text)

artL = make_artists(axL)
artR = make_artists(axR)

def update(frame):
    for traj, art in [(traj_base, artL), (traj_fixed, artR)]:
        i = min(frame * STRIDE, len(traj['x']) - 1)
        x, y, th = traj['x'][i], traj['y'][i], traj['theta'][i]
        art['robot'].center = (x, y)
        hx, hy = x + ROBOT_RADIUS * 1.8 * np.cos(th), y + ROBOT_RADIUS * 1.8 * np.sin(th)
        art['heading'].set_data([x, hx], [y, hy])
        art['path'].set_data(traj['x'][:i + 1], traj['y'][:i + 1])
        pts = traj['obstacle_pts_world'][i]
        art['obs'].set_offsets(pts if len(pts) > 0 else np.zeros((0, 2)))

        feas = traj['feasible'][i]
        collided = traj['collided'][i]
        q = traj['q'][i]
        v = traj['v_cmd'][i]
        wall_dist = traj['wall_dist'][i]
        status = (f"t={i*dt:5.2f}s\n"
                  f"q={q:6.2f}  v={v:5.2f}\n"
                  f"QP: {'FEASIBLE' if feas else 'RELAXED' if not feas else ''}\n"
                  f"wall dist={wall_dist:5.2f}m\n"
                  f"{'*** COLLISION ***' if collided else ''}")
        art['text'].set_text(status)
        if collided:
            art['robot'].set_color('darkred')
            art['text'].set_color('darkred')
        elif not feas:
            art['robot'].set_color('orange')
            art['text'].set_color('#b8860b')
        else:
            art['robot'].set_color('#1f77b4')
            art['text'].set_color('black')
    return (list(artL.values()) + list(artR.values()))

anim = animation.FuncAnimation(fig, update, frames=n_frames, interval=60, blit=False)
plt.tight_layout()

try:
    writer = animation.PillowWriter(fps=16)
    anim.save('/home/claude/sim/robot_animation_v2.gif', writer=writer)
    print("Saved animation to robot_animation_v2.gif")
except Exception as e:
    print(f"\nFailed to save GIF: {e}")
    print("This usually means Pillow isn't installed: pip install Pillow")
    raise
plt.close(fig)
