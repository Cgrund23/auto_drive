#!/usr/bin/env python3
"""
Parameter sweep for the left-wall-follower + Model-Free CBF pipeline,
using simulate_left_wall_follower.py's hallway-with-obstacles simulation as
the objective. Headless (no plotting per trial) so it can run many configs.

Stage 1: (lambda_0=lambda_1, c_q) grid.
Stage 2: fix the stage-1 winner, sweep the EKF's process noise on B_q,v/B_q,phi.
Stage 3: fix stages 1-2, sweep length_scale (the GP "safety factor").

Usage: python3 sweep_left_wall_follower.py
"""
import os
import sys
import types
import time

# Must be set before numpy is imported anywhere in this process. Running many
# trials back-to-back in one process already avoids the multiprocessing
# version's real problem (8 worker processes x numpy's own internal BLAS
# threading = massive oversubscription on this machine -- confirmed: the
# same trial that hung for 9+ minutes under that contention finished in 4.2s
# run alone). These caps are just defensive belt-and-suspenders for a single
# process.
for _v in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS',
           'VECLIB_MAXIMUM_THREADS', 'NUMEXPR_NUM_THREADS'):
    os.environ.setdefault(_v, '1')


def _install_ros_stubs():
    def stub(name, **attrs):
        m = types.ModuleType(name)
        for k, v in attrs.items():
            setattr(m, k, v)
        sys.modules[name] = m

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

    stub('rclpy', init=lambda *a, **k: None, shutdown=lambda *a, **k: None)
    stub('rclpy.node', Node=_StubNode)
    stub('rclpy.executors', MultiThreadedExecutor=object)
    stub('nav_msgs')
    stub('nav_msgs.msg', Odometry=object)
    stub('sensor_msgs')
    stub('sensor_msgs.msg', LaserScan=object)
    stub('ackermann_msgs')
    stub('ackermann_msgs.msg', AckermannDriveStamped=object)


_install_ros_stubs()
sys.path.insert(0, 'auto_drive')
sys.path.insert(0, '.')
import simulate_left_wall_follower as S  # noqa: E402

MAX_STEPS = 3500  # 35 s sim time; if it hasn't finished by then, it's stuck


def run_trial(lam, c_q, q_scale=1.0, length_scale=0.40):
    """Headless trial. Returns a metrics dict; no plotting."""
    sim = S.SimNode()
    sim.cbf.lambda_0 = lam
    sim.cbf.lambda_1 = lam
    sim.cbf.c_q = c_q
    sim.cbf.length_scale = length_scale
    sim.safety_ekf.Q[3, 3] = 2e-4 * q_scale
    sim.safety_ekf.Q[4, 4] = 2e-4 * q_scale

    min_q = float('inf')
    max_infeasible_run = 0
    infeasible_run = 0
    reached_end = False
    collided = False
    collision_kind = None

    for step in range(MAX_STEPS):
        ranges_full, _ = S.simulate_lidar(sim.x, sim.y, sim.theta)
        ranges = ranges_full[::20].astype('float32')
        angles = S.BODY_ANGLES[::20].astype('float32')
        r = sim.step(ranges, angles)

        min_q = min(min_q, r['q'])
        if not r['feasible']:
            infeasible_run += 1
            max_infeasible_run = max(max_infeasible_run, infeasible_run)
        else:
            infeasible_run = 0

        if S.check_collision(r['x'], r['y']):
            collided = True
            for wall_y in (S.HALLWAY_HALF_WIDTH, -S.HALLWAY_HALF_WIDTH):
                if abs(wall_y - r['y']) < S.ROBOT_RADIUS:
                    collision_kind = 'wall'
            if collision_kind is None:
                collision_kind = 'obstacle'
            break
        if r['x'] > S.HALLWAY_LENGTH - 0.3:
            reached_end = True
            break

    return dict(lam=lam, c_q=c_q, q_scale=q_scale, length_scale=length_scale,
                reached_end=reached_end, collided=collided, collision_kind=collision_kind,
                final_x=r['x'], min_q=min_q, max_infeasible_run=max_infeasible_run,
                steps=step, sim_time=step * sim.dt)


def run_grid(configs, label):
    print(f"\n=== {label}: {len(configs)} configs ===")
    t0 = time.time()
    results = []
    for i, cfg in enumerate(configs):
        t_trial = time.time()
        results.append(run_trial(**cfg))
        print(f"  [{i+1}/{len(configs)}] {cfg} -> {time.time()-t_trial:.1f}s", flush=True)
    print(f"({time.time()-t0:.1f}s total)")
    results.sort(key=lambda r: (not r['reached_end'], -r['final_x'], -r['min_q']))
    for r in results:
        tag = "END " if r['reached_end'] else ("CRASH(" + r['collision_kind'] + ")" if r['collided'] else "STUCK")
        print(f"  lam={r['lam']:.2f} c_q={r['c_q']:.2f} q_scale={r['q_scale']:.2f} "
              f"ls={r['length_scale']:.2f} | {tag:14s} x={r['final_x']:5.2f} "
              f"min_q={r['min_q']:6.2f} max_infeas={r['max_infeasible_run']:4d} "
              f"t={r['sim_time']:5.1f}s")
    return results


if __name__ == '__main__':
    # ---- Stage 1: (lambda, c_q) ----
    lambdas = [1.0, 1.5, 2.0, 2.5, 3.0]
    c_qs = [0.5, 0.8, 1.1, 1.4]
    configs = [dict(lam=lam, c_q=cq) for lam in lambdas for cq in c_qs]
    r1 = run_grid(configs, "Stage 1: lambda_0=lambda_1 x c_q")
    best1 = r1[0]
    print(f"\nStage 1 winner: lam={best1['lam']}, c_q={best1['c_q']} "
          f"({'reached end' if best1['reached_end'] else 'best of a bad lot'})")

    # ---- Stage 2: EKF process noise on B_q ----
    q_scales = [0.5, 1.0, 2.0, 5.0]
    configs = [dict(lam=best1['lam'], c_q=best1['c_q'], q_scale=qs) for qs in q_scales]
    r2 = run_grid(configs, "Stage 2: EKF B_q process-noise scale")
    best2 = r2[0]
    print(f"\nStage 2 winner: q_scale={best2['q_scale']}")

    # ---- Stage 3: length_scale (safety factor) ----
    length_scales = [0.30, 0.40, 0.50, 0.60]
    configs = [dict(lam=best1['lam'], c_q=best1['c_q'], q_scale=best2['q_scale'], length_scale=ls)
               for ls in length_scales]
    r3 = run_grid(configs, "Stage 3: length_scale")
    best3 = r3[0]
    print(f"\nStage 3 winner: length_scale={best3['length_scale']}")

    print("\n=== FINAL BEST CONFIG ===")
    print(best3)
