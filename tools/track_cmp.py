#!/usr/bin/env python3
"""Run-to-run track comparison on a common odometry arc-length grid."""
import sys, math
sys.path.insert(0, '/home/ros/tram_ws/src/tram_backup_odometry')
import numpy as np
from tram_backup_odometry.params import Config, default_path
from tram_backup_odometry import replay as R
from tram_backup_odometry.geodesy import enu

bags = sys.argv[1:] or ['30618_0e41eac3', '30618_27e994fc', '30618_af7496f0',
                        '30618_0652866c', '30618_b83d854d', '30618_9c09b081']
T = 400.0
GRID = np.arange(0, 400, 2.0)

tracks = {}
for b in bags:
    msgs = R.read_messages('/home/ros/Downloads/dataset/data/' + b)
    t0 = msgs[0][0]
    # odometry arc length and GNSS ENU, both resampled at 50 Hz
    ft = np.array([t for t, k, p, _ in msgs if k == 'v' and _ == 0]) if False else None
    vf, tr, ur = [], [], []
    pos = []
    lat0 = None
    for t, topic, kind, p in msgs:
        if kind == 'v':
            (vf if topic == R.FRONT else tr).append((t, p))
        elif kind == 'p' and 'master' in topic:
            if lat0 is None:
                lat0 = p
            pos.append((t, enu(*p, *lat0)[:2]))
    if not pos:
        print(b, 'no master fix'); continue
    ta = np.array([x[0] for x in vf]); va = np.array([x[1] for x in vf]) * 0.27783531
    s = np.concatenate([[0.0], np.cumsum((va[:-1] + va[1:]) * 0.5 * np.diff(ta))])
    tp = np.array([x[0] for x in pos]); pp = np.array([x[1] for x in pos])
    g = ta[0] + np.arange(0, ta[-1] - ta[0], 0.02)
    sg = np.interp(g, ta, s)
    px = np.interp(g, tp, pp[:, 0]); py = np.interp(g, tp, pp[:, 1])
    m = sg <= GRID[-1]
    tracks[b] = (sg[m], px[m], py[m])
    print(f'{b}: arc@T={sg[-1]:.1f} m  n={m.sum()}')

ref = bags[0]
rs, rx, ry = tracks[ref]
print(f'\npositions at common odometry arc (reference {ref}):')
for b in bags[1:]:
    s2, x2, y2 = tracks[b]
    n = min(len(rs), len(s2))
    g = rs[:n]
    x = np.interp(g, s2[:n], x2[:n]); y = np.interp(g, s2[:n], y2[:n])
    d = np.hypot(x - rx[:n], y - ry[:n])
    print(f'  {b}: rms {np.sqrt((d**2).mean()):7.2f}  end {d[-1]:7.2f}  max {d.max():7.2f}')
print('\nsample rows (arc, ref x,y, others):')
n = min(len(tracks[b][0]) for b in tracks)
for i in range(0, n, max(1, n // 12)):
    arc = tracks[ref][0][i]
    row = ' '.join(f'({tracks[b][1][i]:7.1f},{tracks[b][2][i]:7.1f})' for b in bags)
    print(f'  s={arc:6.0f}  {row}')
