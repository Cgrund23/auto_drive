#!/usr/bin/env python3
"""
Offline analysis/visualization for a real hardware run of
auto_drive/cbf_Node_refactored.py.

That node now writes two CSV logs per run to ~/cbf_logs/ on whatever
machine it runs on (<run_id>_control.csv every control step, and
<run_id>_scans.csv every LiDAR scan) -- see the "hardware run logging"
block in ControllerNode.__init__/lidar_callback/_control_step. Copy those
two files off the Jetson (e.g. `scp jetson:~/cbf_logs/20260907_*.csv .`)
and point this script at them.

This does NOT reimplement or guess at the safety filter's math: it replays
the logged scans through a FRESH ModelFreeCBF (imported from
cbf_Node_refactored.py via simulate_left_wall_follower.py's existing
ROS-stub trick, so this needs no ROS install) to exactly reconstruct the
persistent per-obstacle GPs -- and therefore the aggregated barrier field --
that the real run actually saw, in the same order the real scans arrived.
Everything else (q, B_q, feasibility, min_range, commanded vs. reference u)
comes directly from what the node itself computed and logged live.

Usage:
    python3 analyze_hardware_log.py                       # most recent run in ~/cbf_logs
    python3 analyze_hardware_log.py 20260907_154212        # a specific run_id
    python3 analyze_hardware_log.py path/to/foo_control.csv
    python3 analyze_hardware_log.py control.csv scans.csv  # explicit pair
"""
import csv
import glob
import os
import sys

import numpy as np
import matplotlib.pyplot as plt

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.join(_HERE, 'auto_drive'))
import simulate_left_wall_follower as S  # noqa: E402 (reuses its ROS-stub + ModelFreeCBF import)

LOG_DIR = os.path.expanduser('~/cbf_logs')
CONTROL_SUFFIX = '_control.csv'
SCAN_SUFFIX = '_scans.csv'


def find_latest_run(log_dir=LOG_DIR):
    controls = sorted(glob.glob(os.path.join(log_dir, f'*{CONTROL_SUFFIX}')))
    if not controls:
        raise FileNotFoundError(
            f'No *{CONTROL_SUFFIX} files found in {log_dir}. Copy a run\'s logs '
            f'off the Jetson first (scp jetson:~/cbf_logs/<run_id>_*.csv {log_dir}/).')
    control_path = controls[-1]
    run_id = os.path.basename(control_path)[:-len(CONTROL_SUFFIX)]
    return control_path, os.path.join(os.path.dirname(control_path), f'{run_id}{SCAN_SUFFIX}')


def resolve_paths(argv):
    if len(argv) == 0:
        return find_latest_run()
    if len(argv) == 1:
        arg = argv[0]
        if arg.endswith(CONTROL_SUFFIX):
            run_dir, base = os.path.split(arg)
            run_id = base[:-len(CONTROL_SUFFIX)]
            return arg, os.path.join(run_dir, f'{run_id}{SCAN_SUFFIX}')
        if arg.endswith('.csv'):
            raise ValueError(f'Pass the *{CONTROL_SUFFIX} path (or just a run_id), not this file directly.')
        return (os.path.join(LOG_DIR, f'{arg}{CONTROL_SUFFIX}'),
                os.path.join(LOG_DIR, f'{arg}{SCAN_SUFFIX}'))
    return argv[0], argv[1]


def load_control_log(path):
    with open(path, newline='') as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError(f'{path} has no data rows')
    cols = {}
    for key in rows[0].keys():
        vals = [r[key] for r in rows]
        if key in ('feasible', 'n_obstacles', 'step'):
            cols[key] = np.array([int(v) for v in vals])
        elif key == 'action':
            cols[key] = np.array(vals)
        else:
            cols[key] = np.array([float(v) for v in vals])
    cols['feasible'] = cols['feasible'].astype(bool)
    cols['t'] = cols['t'] - cols['t'][0]
    return cols


def load_scan_log(path):
    """Returns (angles, scans) where scans is a time-ordered list of
    (t, x, y, theta, ranges). The '#angles' marker row is written ONCE by
    the node (angles are ~constant for a given LiDAR/downsample factor)
    rather than repeated every scan."""
    angles = None
    scans = []
    with open(path, newline='') as f:
        header_seen = False
        for row in csv.reader(f):
            if not row:
                continue
            if row[0] == '#angles':
                angles = np.array([float(v) for v in row[1:]], dtype=np.float32)
                continue
            if not header_seen:
                header_seen = True   # the 't,x,y,theta,r0,...' column-name row
                continue
            t, x, y, theta = (float(v) for v in row[:4])
            ranges = np.array([float(v) for v in row[4:]], dtype=np.float32)
            scans.append((t, x, y, theta, ranges))
    if angles is None or not scans:
        raise ValueError(f'{path} has no scans')
    t0 = scans[0][0]
    scans = [(t - t0, x, y, theta, r) for (t, x, y, theta, r) in scans]
    return angles, scans


def reconstruct_cbf(angles, scans):
    """Replay every logged scan through a FRESH ModelFreeCBF, in the exact
    order they happened live, using SimNode's parameters (which mirror
    cbf_Node_refactored.py's -- see that file's own 'keep these in sync'
    convention). This reproduces the persistent per-obstacle GP state, and
    therefore the barrier field, that the real run actually saw."""
    cbf = S.SimNode().cbf
    for (_, x, y, theta, ranges) in scans:
        cbf.set_obstacles(ranges, angles, robot_xy=(x, y), robot_theta=theta)
    return cbf


def infeasible_spans(t, feasible):
    """[(t_start, t_end), ...] for each maximal run of feasible==False."""
    spans = []
    n = len(feasible)
    i = 0
    while i < n:
        if not feasible[i]:
            j = i
            while j < n and not feasible[j]:
                j += 1
            spans.append((t[i], t[j - 1]))
            i = j
        else:
            i += 1
    return spans


def scan_periods(scans):
    """Real measured scan-to-scan wall-clock gaps -- the node timestamps
    every scan with the ROS clock at arrival, so this is the ACTUAL cadence
    the perception layer saw, independent of the nominal dt_sub used
    internally for the EKF/CBF math. Choppiness/stalls (LiDAR delivery
    hiccups, executor contention, OS preemption) show up here even when
    the control law's own per-call compute time (logged separately) stays
    flat -- see analyze_hardware_log.py's module docstring."""
    ts = np.array([s[0] for s in scans])
    return ts[1:], np.diff(ts)


def summarize(cols, scans):
    t, feas, minr, q = cols['t'], cols['feasible'], cols['min_range'], cols['q_hat']
    duration = t[-1] - t[0] if len(t) > 1 else 0.0
    dist = float(np.sum(np.hypot(np.diff(cols['x']), np.diff(cols['y']))))
    spans = infeasible_spans(np.arange(len(feas)), feas)   # in STEPS, not seconds
    max_run = max((j - i + 1 for i, j in spans), default=0)
    max_run_start_t = t[int(min((i for i, j in spans if j - i + 1 == max_run), default=0))] if spans else 0.0
    frac_infeasible = float(np.mean(~feas))
    frac_estop = float(np.mean(cols['action'] == 'ESTOP'))
    print(f"Run duration:            {duration:.1f} s ({len(t)} control steps)")
    print(f"Distance traveled:       {dist:.2f} m")
    print(f"Min q_hat reached:       {np.min(q):.3f}  (q<0 = estimated barrier violation)")
    print(f"Min LiDAR range reached: {np.min(minr):.3f} m")
    print(f"Fraction infeasible:     {frac_infeasible*100:.1f}%")
    print(f"Fraction in ESTOP:       {frac_estop*100:.1f}%")
    print(f"Longest infeasible run:  {max_run} steps, starting at t={max_run_start_t:.2f}s")

    scan_t, gaps = scan_periods(scans)
    if len(gaps) > 0:
        median = float(np.median(gaps))
        stall_mask = gaps > median * 2.0
        print(f"\nMedian scan period:      {median*1000:.1f} ms ({1/median:.1f} Hz)")
        print(f"Worst scan gap:          {np.max(gaps)*1000:.0f} ms, at t={scan_t[np.argmax(gaps)]:.2f}s")
        n_stalls = int(np.sum(stall_mask))
        print(f"Scan stalls (>2x median): {n_stalls}", end='')
        if n_stalls:
            stall_times = ', '.join(f'{tt:.2f}s' for tt in scan_t[stall_mask][:10])
            print(f" at t = {stall_times}{' ...' if n_stalls > 10 else ''}")
        else:
            print()


def plot(cols, scans, cbf, out_png):
    t = cols['t']
    spans = infeasible_spans(t, cols['feasible'])

    def shade(ax):
        for (s, e) in spans:
            ax.axvspan(s, e, color='red', alpha=0.08, zorder=0)

    fig = plt.figure(figsize=(17, 12))
    gs = fig.add_gridspec(4, 3, width_ratios=[1.6, 1, 1], height_ratios=[1, 1, 0.4, 0.6])

    ax_map = fig.add_subplot(gs[:2, 0])
    xs = np.linspace(cols['x'].min() - 0.5, cols['x'].max() + 0.5, 160)
    ys = np.linspace(cols['y'].min() - 0.7, cols['y'].max() + 0.7, 80)
    Q = S.barrier_field(cbf, xs, ys)
    levels = np.linspace(-2.0, 1.0, 25)
    cf = ax_map.contourf(xs, ys, Q, levels=levels, cmap='RdYlGn', alpha=0.65, extend='both')
    ax_map.contour(xs, ys, Q, levels=[0.0], colors='black', linewidths=2.0)
    fig.colorbar(cf, ax=ax_map, label='aggregated barrier q (reconstructed)', shrink=0.7)

    colors = plt.cm.tab10(np.linspace(0, 1, max(len(cbf.gps), 1)))
    for (obs_id, gp), c in zip(cbf.gps.items(), colors):
        if gp.n_points > 0:
            ax_map.scatter(gp.P[:, 0], gp.P[:, 1], s=14, color=c, zorder=4,
                            label=f'obstacle #{obs_id} ({gp.n_points} pts)')

    ax_map.scatter(cols['x'], cols['y'], c=cols['q_hat'], cmap='RdYlGn',
                   vmin=-1, vmax=1, s=6, zorder=5, label='path (color = q)')
    ax_map.plot(cols['x'][0], cols['y'][0], 'go', markersize=10, zorder=6, label='start')
    ax_map.plot(cols['x'][-1], cols['y'][-1], 'r^', markersize=10, zorder=6, label='end')
    ax_map.set_xlabel('x (m)')
    ax_map.set_ylabel('y (m)')
    ax_map.set_title('Hardware run: trajectory over reconstructed barrier field')
    ax_map.legend(loc='best', fontsize=7)
    ax_map.set_aspect('equal')

    ax_q = fig.add_subplot(gs[0, 1])
    ax_q.plot(t, cols['q_hat'], color='green')
    ax_q.axhline(0, color='red', linestyle='--', label='unsafe (q<0)')
    shade(ax_q)
    ax_q.set_ylabel('barrier q')
    ax_q.set_title('Safety barrier (EKF estimate)')
    ax_q.legend(fontsize=7)
    ax_q.grid(alpha=0.3)

    ax_b = fig.add_subplot(gs[0, 2])
    ax_b.plot(t, cols['Bqv_hat'], label='B_q,v')
    ax_b.plot(t, cols['Bqphi_hat'], label='B_q,phi')
    ax_b.axhline(0, color='k', alpha=0.3)
    shade(ax_b)
    ax_b.set_title('ULM sensitivity estimates')
    ax_b.legend(fontsize=7)
    ax_b.grid(alpha=0.3)

    ax_u = fig.add_subplot(gs[1, 1])
    ax_u.plot(t, cols['v_cmd'], label='v (cmd)', color='C0')
    ax_u.plot(t, cols['v_ref'], label='v_ref', color='C0', linestyle=':', alpha=0.6)
    ax_u.plot(t, cols['phi_cmd'], label='phi (cmd)', color='C1')
    ax_u.plot(t, cols['phi_ref'], label='phi_ref', color='C1', linestyle=':', alpha=0.6)
    shade(ax_u)
    ax_u.set_xlabel('t (s)')
    ax_u.set_title('Commanded vs. reference u')
    ax_u.legend(fontsize=6)
    ax_u.grid(alpha=0.3)

    ax_r = fig.add_subplot(gs[1, 2])
    ax_r.plot(t, cols['min_range'], color='purple')
    ax_r.axhline(0.30, color='red', linestyle='--', label='e-stop threshold')
    shade(ax_r)
    ax_r.set_xlabel('t (s)')
    ax_r.set_title('Closest LiDAR return')
    ax_r.legend(fontsize=7)
    ax_r.grid(alpha=0.3)

    ax_mode = fig.add_subplot(gs[2, :])
    modes = ['SAFE', 'STEER', 'BRAKE', 'ESTOP']
    mode_colors = {'SAFE': 'tab:green', 'STEER': 'tab:blue', 'BRAKE': 'tab:orange', 'ESTOP': 'tab:red'}
    for i, m in enumerate(modes):
        mask = cols['action'] == m
        ax_mode.scatter(t[mask], np.full(mask.sum(), i), s=4, color=mode_colors[m], marker='|')
    ax_mode.set_yticks(range(len(modes)))
    ax_mode.set_yticklabels(modes)
    ax_mode.set_xlabel('t (s)')
    ax_mode.set_title('Control mode over time (red background = QP infeasible)')
    shade(ax_mode)
    ax_mode.set_xlim(t[0], t[-1])

    ax_scan = fig.add_subplot(gs[3, :])
    scan_t, gaps = scan_periods(scans)
    if len(gaps) > 0:
        median = float(np.median(gaps))
        ax_scan.plot(scan_t, gaps * 1000, color='teal', linewidth=0.8)
        ax_scan.axhline(median * 1000, color='k', linestyle=':', alpha=0.5, label=f'median {median*1000:.0f}ms')
        stall_mask = gaps > median * 2.0
        if np.any(stall_mask):
            ax_scan.scatter(scan_t[stall_mask], gaps[stall_mask] * 1000, color='red', s=20,
                             zorder=5, label='stall (>2x median)')
        ax_scan.set_xlim(t[0], t[-1])
        ax_scan.set_xlabel('t (s)')
        ax_scan.set_ylabel('scan period (ms)')
        ax_scan.set_title('Real measured LiDAR scan cadence (choppiness/stalls show up here even '
                           'when per-call compute time stays flat)')
        ax_scan.legend(fontsize=7)
        ax_scan.grid(alpha=0.3)

    plt.tight_layout()
    plt.savefig(out_png, dpi=140)
    print(f"Saved {out_png}")


def main():
    control_path, scan_path = resolve_paths(sys.argv[1:])
    print(f"Control log: {control_path}")
    print(f"Scan log:    {scan_path}")
    cols = load_control_log(control_path)
    angles, scans = load_scan_log(scan_path)
    cbf = reconstruct_cbf(angles, scans)

    print()
    summarize(cols, scans)
    print()

    if control_path.endswith(CONTROL_SUFFIX):
        out_png = control_path[:-len(CONTROL_SUFFIX)] + '_analysis.png'
    else:
        out_png = os.path.splitext(control_path)[0] + '_analysis.png'
    plot(cols, scans, cbf, out_png)


if __name__ == '__main__':
    main()
