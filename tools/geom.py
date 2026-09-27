#!/usr/bin/env python3
"""Trajectory geometry: are routes repeatable? how much does heading vary?"""
import sys
sys.path.insert(0, '/home/ros/tram_ws/tools')
import numpy as np
from bag_io import read_run, find_bags, gnss_position

DATA = '/home/ros/Downloads/dataset/data'
bags = find_bags(DATA)
sel = sys.argv[1:] or []
if sel:
    bags = [b for b in bags if any(s in b.name for s in sel)]

for b in bags[:8]:
    r = read_run(b)
    p = gnss_position(r)
    if p is None:
        print(f'{r.name}: no gnss'); continue
    t, enu, _ = p
    # robust: reject jumps > 30 m in one step
    st = np.hypot(np.diff(enu[:, 0]), np.diff(enu[:, 1]))
    keep = np.ones(len(t), bool)
    bad = st > 25
    idx = np.where(bad)[0]
    for i in idx:
        keep[i + 1] = False
    e = enu[keep]
    s = np.concatenate([[0], np.cumsum(np.hypot(np.diff(e[:, 0]), np.diff(e[:, 1])))])
    h = np.unwrap(np.arctan2(np.diff(e[:, 1]), np.diff(e[:, 0])))
    h = np.concatenate([h, h[-1:]])
    print(f'{r.name}: n={len(t)} dur={t[-1]:7.1f} path={s[-1]:8.1f} m  '
          f'x[{e[:,0].min():8.1f},{e[:,0].max():8.1f}] y[{e[:,1].min():8.1f},{e[:,1].max():8.1f}] '
          f'head0={np.degrees(h[0]):7.1f} head_end={np.degrees(h[-1]):7.1f} dhead={np.degrees(h[-1]-h[0]):7.1f} '
          f'z[{e[:,2].min():6.1f},{e[:,2].max():6.1f}] bad={bad.sum()}')
    # cumulative absolute heading change
    print(f'    total |dheading| = {np.degrees(np.abs(np.diff(h)).sum()):.1f} deg   '
          f'net turn = {np.degrees(h[-1]-h[0]):.1f} deg')
