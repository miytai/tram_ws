#!/usr/bin/env python3
"""Produce the figures of docs/RESULTS.md from a scoring CSV + one example run."""
import csv
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, '/home/ros/tram_ws/src/tram_backup_odometry')
from tram_backup_odometry.params import Config, default_path
from tram_backup_odometry import replay as R

BAG = '/home/ros/Downloads/dataset/data/30618_01f73500'
OUT = '/home/ros/tram_ws/src/tram_backup_odometry/docs/figures'


def hist(csv_path, key, title, fname, log=False, xlim=None):
    vals = []
    with open(csv_path) as fh:
        for row in csv.DictReader(fh):
            try:
                v = float(row[key])
            except (ValueError, KeyError):
                continue
            if np.isfinite(v):
                vals.append(v)
    if not vals:
        return
    fig, ax = plt.subplots(figsize=(7, 4))
    if log:
        vals = [v for v in vals if v > 0]
        ax.hist(np.log10(vals), bins=40, color='#2b6cb0', edgecolor='white')
        ax.set_xlabel('log10 ' + key)
    else:
        ax.hist(vals, bins=40, color='#2b6cb0', edgecolor='white')
        ax.set_xlabel(key)
    ax.set_ylabel('runs')
    ax.set_title(f'{title}\nn={len(vals)}  median {np.median(vals):.3f}  '
                 f'p90 {np.percentile(vals, 90):.3f}')
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(f'{OUT}/{fname}', dpi=110)
    plt.close(fig)
    print('wrote', fname, 'n =', len(vals), 'median', round(float(np.median(vals)), 4))


def example_run():
    cfg = Config()
    cfg.model.model_file = default_path('model.json')
    cfg.route.map_file = default_path('route_map.json')
    res = R.replay(BAG, cfg)
    grid, vref, pref = R.reference_on_grid(res)
    v = np.interp(grid, res.t, res.v)
    p = np.column_stack([np.interp(grid, res.t, res.p[:, k]) for k in range(3)])
    d = np.linalg.norm((p - pref)[:, :2], axis=1)
    kappa = np.interp(grid, res.t, res.kappa)

    fig, ax = plt.subplots(4, 1, figsize=(13, 11))
    ax[0].plot(grid, vref, lw=2.2, color='#111', label='GNSS reference')
    ax[0].plot(grid, v, lw=1.2, color='#e53e3e', label='estimate')
    ax[0].set_ylabel('speed [m/s]')
    ax[0].legend(loc='upper right')
    ax[0].grid(alpha=0.3)
    ax[0].set_title(f'backup odometry, run {BAG.split("/")[-1]}')

    ax[1].plot(grid, v - vref, lw=0.8, color='#2b6cb0')
    ax[1].axhline(0, color='#999', lw=0.8)
    ax[1].set_ylabel('velocity error [m/s]')
    ax[1].set_ylim(-0.6, 0.6)
    ax[1].grid(alpha=0.3)

    ax[2].plot(pref[:, 0], pref[:, 1], lw=2.2, color='#111', label='GNSS reference')
    ax[2].plot(p[:, 0], p[:, 1], lw=1.2, color='#e53e3e', label='estimate (map matched)')
    ax[2].set_aspect('equal')
    ax[2].set_xlabel('x [m]')
    ax[2].set_ylabel('y [m]')
    ax[2].legend(loc='best')
    ax[2].grid(alpha=0.3)

    ax[3].plot(grid, d, lw=0.9, color='#38a169', label='position error')
    ax[3].plot(grid, np.abs(kappa) * 20.0, lw=0.8, color='#d69e2e',
               label='20·|slip ratio|')
    ax[3].set_ylabel('error [m]')
    ax[3].set_xlabel('t [s]')
    ax[3].legend(loc='upper left')
    ax[3].grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(f'{OUT}/example_run.png', dpi=110)
    plt.close(fig)
    print('wrote example_run.png  final position error %.1f m' % d[-1])


if __name__ == '__main__':
    import os
    os.makedirs(OUT, exist_ok=True)
    csvs = sys.argv[1:] or ['/home/ros/tram_ws/calib/scores_map.csv']
    for c in csvs:
        tag = os.path.basename(c).replace('scores_', '').replace('.csv', '')
        hist(c, 'v_rmse', f'velocity RMSE [m/s] ({tag})', f'velocity_rmse_{tag}.png')
        hist(c, 'pos_rmse', f'position RMSE [m] ({tag})', f'position_rmse_{tag}.png', log=True)
        hist(c, 'drift_pct', f'accumulated drift [% of distance] ({tag})',
             f'drift_{tag}.png', log=True)
    example_run()
