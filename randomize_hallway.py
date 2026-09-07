#!/usr/bin/env python3
"""
Randomized-hallway test for the left-wall-follower + Model-Free CBF pipeline.

The fixed 3-obstacle-on-the-setpoint-line hallway in
simulate_left_wall_follower.py was a deliberately adversarial stress test
(obstacles placed exactly on the nominal wall-following line). This script
instead generates many random obstacle layouts and runs the SAME production
code (SimNode, unmodified) against each headless, to see whether the
recurring ~x=3.3-3.5m crash was specific to that one hard-coded layout or a
more general limitation.

For one interesting seed (first success if any, else the best failure), it
also reproduces the full simulate_left_wall_follower.py output (summary PNG
+ animated GIF) via that module's own main(), just with OBSTACLES swapped
for the random layout -- no logic is duplicated.

Usage: python3 randomize_hallway.py [n_seeds]
"""
import sys
import time

import numpy as np

import simulate_left_wall_follower as S  # noqa: E402 (stubs installed inside that module)

MAX_STEPS = 3500


def random_obstacles(rng, n=3, hallway_length=S.HALLWAY_LENGTH, half_width=S.HALLWAY_HALF_WIDTH):
    """n obstacles spread along the corridor with jittered spacing, each at a
    random lateral offset and radius -- unlike the fixed test layout, NOT
    pinned to the wall-follower's setpoint line."""
    obstacles = []
    for i in range(n):
        base_x = hallway_length * (i + 1) / (n + 1)
        x = base_x + rng.uniform(-0.7, 0.7)
        y = rng.uniform(-0.7, 0.7)
        r = rng.uniform(0.20, 0.35)
        obstacles.append((float(x), float(y), float(r)))
    return obstacles


def run_trial(obstacles, max_steps=MAX_STEPS):
    """Headless trial against a given obstacle layout, using SimNode's
    stock (production) parameters -- no per-trial overrides."""
    sim = S.SimNode()
    min_q = float('inf')
    for step in range(max_steps):
        ranges_full, _ = S.simulate_lidar(sim.x, sim.y, sim.theta, obstacles=obstacles)
        ranges = ranges_full[::20].astype('float32')
        angles = S.BODY_ANGLES[::20].astype('float32')
        r = sim.step(ranges, angles)
        min_q = min(min_q, r['q'])
        if S.check_collision(r['x'], r['y'], obstacles=obstacles):
            return dict(reached_end=False, collided=True, final_x=r['x'], min_q=min_q,
                        steps=step, sim_time=step * sim.dt)
        if r['x'] > S.HALLWAY_LENGTH - 0.3:
            return dict(reached_end=True, collided=False, final_x=r['x'], min_q=min_q,
                        steps=step, sim_time=step * sim.dt)
    return dict(reached_end=False, collided=False, final_x=r['x'], min_q=min_q,
                steps=step, sim_time=step * sim.dt)


def main():
    n_seeds = int(sys.argv[1]) if len(sys.argv) > 1 else 15
    results = []
    t0 = time.time()
    for seed in range(n_seeds):
        rng = np.random.default_rng(seed)
        obstacles = random_obstacles(rng)
        res = run_trial(obstacles)
        res['seed'] = seed
        res['obstacles'] = obstacles
        results.append(res)
        tag = 'END ' if res['reached_end'] else ('CRASH' if res['collided'] else 'STUCK')
        obs_str = ', '.join(f'({x:.1f},{y:.1f},r{r:.2f})' for x, y, r in obstacles)
        print(f"seed={seed:2d} [{tag}] x={res['final_x']:5.2f} min_q={res['min_q']:6.2f} "
              f"t={res['sim_time']:5.1f}s | obstacles: {obs_str}")

    n_end = sum(r['reached_end'] for r in results)
    n_crash = sum(r['collided'] for r in results)
    n_stuck = n_seeds - n_end - n_crash
    print(f"\n({time.time()-t0:.1f}s total)")
    print(f"Reached end: {n_end}/{n_seeds}  Crashed: {n_crash}/{n_seeds}  Stuck/timeout: {n_stuck}/{n_seeds}")
    print(f"Mean final_x: {np.mean([r['final_x'] for r in results]):.2f}m "
          f"(hallway length {S.HALLWAY_LENGTH}m)")

    # Reproduce the full plot+animation for one interesting seed: prefer the
    # first success, else the seed that got furthest.
    successes = [r for r in results if r['reached_end']]
    pick = successes[0] if successes else max(results, key=lambda r: r['final_x'])
    print(f"\nGenerating full visualization for seed={pick['seed']} "
          f"({'success' if pick['reached_end'] else 'best failure'})...")
    S.OBSTACLES = pick['obstacles']
    S.main()


if __name__ == '__main__':
    main()
