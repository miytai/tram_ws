#!/usr/bin/env python3
"""Baseline accuracy: naive wheel integration vs GNSS reference."""
import sys, pickle
sys.path.insert(0, '/home/ros/tram_ws/tools')
import numpy as np

runs = pickle.load(open('/home/ros/tram_ws/calib/runs.pkl', 'rb'))
SC = 1.0 / 3.6


def along_track_err(rec, v):
    """signed along-track and cross-track error vs GNSS path, in the GNSS frame."""
    if 'pos' not in rec:
        return None
    p = rec['pos']
    d = np.diff(p[:, :2], axis=0)
    seg = np.hypot(d[:, 0], d[:, 1])
    # GNSS jumps: split the path at them
    good = seg < 5.0
    # cumulative distance using only good segments
    s = np.concatenate([[0], np.cumsum(np.where(good, seg, 0.0))])
    s = np.maximum.accumulate(s)
    # estimated s from odometry
    shat = np.concatenate([[0], np.cumsum(0.5 * (v[:-1] + v[1:]) * np.diff(rec['t']))])
    e = shat - s
    valid = np.concatenate([[True], good])
    e = e[valid]
    return e[-1], s[-1], shat[-1], np.sqrt(np.mean(e ** 2)), np.max(np.abs(e))


rows = []
for r in runs:
    v = r['front'] * SC
    out = along_track_err(r, v)
    if out is None:
        continue
    e_end, s_end, sh_end, rmse, mx = out
    # velocity rmse
    vr = None
    if 'gv' in r:
        d = r['gv'] - v
        vr = (float(np.sqrt(np.mean(d ** 2))), float(d.mean()))
    rows.append((r['name'], s_end, e_end, 100 * e_end / s_end, rmse, mx, vr))

print(f'{"bag":16}{"gnssS":>9}{"errEnd":>9}{"drift%":>8}{"rmseS":>8}{"maxS":>8}{"vRMSE":>8}{"vBias":>8}')
for r in rows:
    vr = f'{r[6][0]:8.3f}{r[6][1]:8.3f}' if r[6] else '       -       -'
    print(f'{r[0]:16}{r[1]:9.1f}{r[2]:9.1f}{r[3]:8.2f}{r[4]:8.1f}{r[5]:8.1f}{vr}')
a = np.array([[r[2], r[3], r[4], r[6][0] if r[6] else np.nan] for r in rows])
print(f'\nMEAN drift% = {np.nanmean(a[:,1]):.3f}  median = {np.nanmedian(a[:,1]):.3f}  '
      f'std = {np.nanstd(a[:,1]):.3f}')
print(f'MEAN vRMSE  = {np.nanmean(a[:,3]):.4f} m/s   median {np.nanmedian(a[:,3]):.4f}')
print(f'MEAN rmseS  = {np.nanmean(a[:,2]):.1f} m')
