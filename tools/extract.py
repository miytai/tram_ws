#!/usr/bin/env python3
"""Extract dynamics dataset: v, a, notch per bag on a 20 Hz grid (online-safe)."""
import sys, os, pickle
sys.path.insert(0, '/home/ros/tram_ws/tools')
import numpy as np
from bag_io import read_run, find_bags, gnss_speed, gnss_position

DATA = '/home/ros/Downloads/dataset/data'
OUT = '/home/ros/tram_ws/calib/runs.pkl'
os.makedirs('/home/ros/tram_ws/calib', exist_ok=True)
DT = 0.05

bags = find_bags(DATA)
runs = []
for k, b in enumerate(bags):
    r = read_run(b)
    tf, vf = r.t['front'], r.v_front
    tr, vr = r.t['rear'], r.v_rear
    tn, nn = r.t['notch'], r.notch
    t0 = min(tf[0], tr[0], tn[0])
    T = max(tf[-1], tr[-1], tn[-1]) - t0
    if T < 5:
        continue
    g = np.arange(0, T, DT) + t0
    front = np.interp(g, tf, vf)
    rear = np.interp(g, tr, vr)
    notch = np.interp(g, tn, nn)
    rec = dict(name=r.name, t=g - t0, front=front, rear=rear, notch=notch)

    gt, gv = gnss_speed(r)
    if gt is not None and len(gt) > 10:
        tgx = gt + r.gnss['t0_rel']
        rec['gt'] = gx = g - t0 + t0
        rec['gv'] = np.hypot(np.interp(g, tgx, gv[:, 0]), np.interp(g, tgx, gv[:, 1]))
        rec['gvx'] = np.interp(g, tgx, gv[:, 0])
        rec['gvy'] = np.interp(g, tgx, gv[:, 1])
    p = gnss_position(r)
    if p is not None:
        t, enu, _ = p
        tgx = t + r.gnss['t0_rel']
        rec['pos'] = np.column_stack([np.interp(g, tgx, enu[:, k]) for k in range(3)])
    runs.append(rec)
    print(f'[{k+1}/{len(bags)}] {r.name} T={T:.0f}s gnss={"gv" in rec} n={len(g)}', flush=True)

with open(OUT, 'wb') as f:
    pickle.dump(runs, f)
print('saved', OUT, len(runs), 'runs')
