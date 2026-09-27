#!/usr/bin/env python3
"""Whole-track alignment between runs: find the arc offset that explains them."""
import sys, math
sys.path.insert(0, '/home/ros/tram_ws/src/tram_backup_odometry')
import numpy as np
from tram_backup_odometry.params import Config, default_path
from tram_backup_odometry import replay as R
from tram_backup_odometry.geodesy import enu
from tram_backup_odometry.route import RouteMap, _rigid_similarity


def obs_track(bag, step=2.0, src='master'):
    msgs = R.read_messages('/home/ros/Downloads/dataset/data/' + bag)
    vf, pos = [], []
    lat0 = None
    for t, topic, kind, p in msgs:
        if kind == 'v' and topic == R.FRONT:
            vf.append((t, p))
        elif kind == 'p' and src in topic:
            if lat0 is None:
                lat0 = p
            pos.append((t, enu(*p, *lat0)[:2]))
    ta = np.array([x[0] for x in vf]); va = np.array([x[1] for x in vf]) * 0.27783531
    s = np.concatenate([[0.0], np.cumsum((va[:-1] + va[1:]) * 0.5 * np.diff(ta))])
    tp = np.array([x[0] for x in pos]); pp = np.array([x[1] for x in pos])
    g = ta[0] + np.arange(0, ta[-1] - ta[0], 0.02)
    sg = np.interp(g, ta, s)
    px = np.interp(g, tp, pp[:, 0]); py = np.interp(g, tp, pp[:, 1])
    d = np.hypot(np.diff(px), np.diff(py))
    keep = d < 4.0
    sg = np.maximum.accumulate(np.r_[0.0, np.cumsum(np.where(keep, d, 0.0))])
    m = keep
    sg = np.r_[sg[0], sg[1:][m]]
    px = np.r_[px[0], px[1:][m]]
    py = np.r_[py[0], py[1:][m]]
    tgt = np.arange(0, sg[-1], step)
    i = np.clip(np.searchsorted(sg, tgt), 0, len(sg) - 1)
    return tgt, np.column_stack([px[i], py[i]])


bags = sys.argv[1:]
ref_bag = bags[0]
rs, rp = obs_track(ref_bag)
print(f'reference {ref_bag}: {len(rs)} nodes, {rs[-1]:.0f} m (odometry arc)')
print(f'{"bag":16}{"gnssL":>9}{"res":>9}{"s0":>9}{"yaw":>8}{"scale":>9}{"end":>9}')
for b in bags[1:]:
    s2, p2 = obs_track(b)
    # reference polyline built from GNSS-derived arc (smoothed)
    src = rp[:, 0] + 0j
    Lr = rs[-1]
    best = None
    for s0 in np.arange(-200, 200, 2.0):
        idx = (rs[::20] + s0) / 20.0
        m = (idx >= 0) & (idx <= len(s2) - 1)
        if m.sum() < 20:
            continue
        ref_pts = rp[::20][m]
        cand = p2[idx[m].astype(int)]
        flat = [v for pt in cand for v in pt] + [0.0] * len(cand)
        yaw, tx, ty, res, sc = _rigid_similarity(flat, [tuple(p) for p in ref_pts])
        if best is None or res < best[0]:
            best = (res, s0, yaw, sc)
    if best:
        print(f'{b:16}{s2[-1]:9.0f}{best[0]:9.2f}{best[1]:9.0f}'
              f'{math.degrees(best[2]):8.2f}{best[3]:9.5f}{best[0]:9.2f}')
