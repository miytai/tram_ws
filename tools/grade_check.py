#!/usr/bin/env python3
"""Does removing the track grade (from the GNSS altitude profile) explain the
scatter of the a(u,v) table?"""
import sys, pickle, json
sys.path.insert(0, '/home/ros/tram_ws/tools')
import numpy as np

runs = pickle.load(open('/home/ros/tram_ws/calib/runs.pkl', 'rb'))
SC = 1.0 / 3.6
G = 9.80665
NV = 15
DT = 0.05
WIN = 101          # 5 s smoothing window for dz/ds


def grade_of(z, t):
    """robust dz/ds from the (noisy, jump-containing) altitude profile"""
    zi = z.copy()
    # remove big jumps by clipping the derivative
    d = np.gradient(zi, t)
    lim = 0.25
    d = np.clip(d, -lim, lim)
    k = np.ones(WIN) / WIN
    ds = np.convolve(np.pad(d, (WIN // 2, WIN // 2), mode='edge'), k, mode='valid')
    return -G * ds           # a_grade [m/s^2]


for r in runs:
    v = np.convolve(r['front'] * SC, np.ones(11) / 11, mode='same')
    a = np.gradient(v, DT)
    ok = np.ones(len(v), bool); ok[:30] = False; ok[-30:] = False
    r['_v'], r['_a'], r['_ok'] = v, a, ok
    r['_n'] = np.round(r['notch']).astype(int)
    r['_vb'] = np.clip(v.astype(int), 0, NV - 1)
    if 'pos' in r:
        r['_ag'] = np.where(ok, grade_of(r['pos'][:, 2], r['t']), 0.0)
    else:
        r['_ag'] = np.zeros(len(v))


def table(sel):
    acc = np.zeros((31, NV)); wgt = np.zeros((31, NV))
    for r in runs:
        a = r['_a'] - (r[sel] if sel else 0.0)
        for u in range(-15, 16):
            m = r['_ok'] & (r['_n'] == u)
            if not m.any():
                continue
            np.add.at(acc, (u + 15, r['_vb'][m]), a[m] * m[m])
            np.add.at(wgt, (u + 15, r['_vb'][m]), m[m].astype(float))
    mn = wgt > 200
    return np.where(mn, acc / np.maximum(wgt, 1e-9), np.nan), mn


A0, mn = table(None)
Ag, mng = table('_ag')
g = np.concatenate([r['_ag'][r['_ok']] for r in runs])
print(f'grade accel: std {g.std():.4f}  p1 {np.percentile(g,1):+.3f}  p99 {np.percentile(g,99):+.3f} '
      f'max|.| {np.abs(g).max():.3f}')

# per-cell scatter across runs (irreducible noise of the table)
def cell_std():
    acc = np.zeros((31, NV)); acc2 = np.zeros((31, NV)); wgt = np.zeros((31, NV))
    for r in runs:
        a = r['_a'] - r['_ag']
        for u in range(-15, 16):
            m = r['_ok'] & (r['_n'] == u)
            if not m.any():
                continue
            np.add.at(acc, (u + 15, r['_vb'][m]), a[m] * m[m])
            np.add.at(acc2, (u + 15, r['_vb'][m]), a[m] ** 2 * m[m])
            np.add.at(wgt, (u + 15, r['_vb'][m]), m[m].astype(float))
    return acc, acc2, wgt


acc, acc2, wgt = cell_std()
mn2 = wgt > 500
mu = acc / np.maximum(wgt, 1)
var = np.maximum(acc2 / np.maximum(wgt, 1) - mu ** 2, 0)
sd = np.sqrt(var)
print(f'\ncell std (grade removed): median {np.median(sd[mn2]):.4f}  p90 {np.percentile(sd[mn2],90):.4f}  '
      f'max {sd[mn2].max():.4f}')
print(f'cell weight: median {np.median(wgt[mn2]):.0f}')
print('\na(u,v) with grade removed  rows v=0.5..14.5, cols u=-15..15')
print(np.array2string(np.where(mng, Ag, np.nan), precision=2, max_line_width=260))
np.save('/home/ros/tram_ws/calib/table_grade_removed.npy', Ag)
