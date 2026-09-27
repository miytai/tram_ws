#!/usr/bin/env python3
"""Per-run error profile: where does the position error grow?"""
import sys
sys.path.insert(0, '/home/ros/tram_ws/src/tram_backup_odometry')
import numpy as np
from tram_backup_odometry.params import Config, default_path
from tram_backup_odometry import replay as R

cfg = Config()
cfg.model.model_file = default_path('model.json')
cfg.route.map_file = default_path('route_map.json')
for b in sys.argv[1:]:
    res = R.replay(b, cfg)
    grid, vref, pref = R.reference_on_grid(res)
    v = np.interp(grid, res.t, res.v)
    p = np.column_stack([np.interp(grid, res.t, res.p[:, k]) for k in range(3)])
    d = np.linalg.norm((p - pref)[:, :2], axis=1)
    s = np.interp(grid, res.t, res.kappa)
    print(f'== {b.split("/")[-1]}  align={res.align_status} res={res.align_residual:.1f} ws={res.wheel_scale:.4f}')
    print('   t[s]   refX     refY    estX     estY     err   s_map')
    n = len(grid)
    for i in range(0, n, max(1, n // 18)):
        print(f'{grid[i]:7.0f} {pref[i,0]:8.1f} {pref[i,1]:8.1f} {p[i,0]:8.1f} {p[i,1]:8.1f} {d[i]:8.1f}')
    print(f'   final err {d[-1]:.1f} m   max {d.max():.1f} m   ref dist {np.hypot(np.diff(pref[:,0]),np.diff(pref[:,1])).sum():.0f} m')
