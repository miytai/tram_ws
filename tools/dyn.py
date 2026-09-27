#!/usr/bin/env python3
"""Analyse dynamics: notch->acceleration map, slip statistics, wheel-scale check."""
import sys, pickle
sys.path.insert(0, '/home/ros/tram_ws/tools')
import numpy as np

runs = pickle.load(open('/home/ros/tram_ws/calib/runs.pkl', 'rb'))
SC = 1.0 / 3.6

# ---- 1. global wheel scale vs gnss -----------------------------------------
num = den = 0.0
res = []
for r in runs:
    if 'gv' not in r:
        continue
    m = r['gv'] > 1.0
    if m.sum() < 100:
        continue
    num += float((r['front'][m] * SC * r['gv'][m]).sum())
    den += float((r['gv'][m] ** 2).sum())
    e = r['front'][m] * SC - r['gv'][m]
    res.append((e.mean(), e.std(), m.sum()))
print('wheel scale (LS vs gnss): %.6f  (1.0 == exactly km/h)' % (num / den))
E = np.array([x[0] for x in res]); S = np.array([x[1] for x in res])
print('per-run bias: mean %+.4f std %.4f  |  per-run resid std: mean %.4f' % (E.mean(), E.std(), S.mean()))

# front vs rear consistency
d = []
for r in runs:
    d.append(r['front'] - r['rear'])
d = np.concatenate(d) * SC
print('front-rear: mean %+.5f  std %.5f  p99.9 %.3f  p0.1 %.3f' % (
    d.mean(), d.std(), np.percentile(np.abs(d), 99.9), np.percentile(d, 0.1)))

# ---- 2. acceleration vs notch ----------------------------------------------
# use gnss-based accel where available, wheel accel otherwise
def smooth(x, w=9):
    k = np.ones(w) / w
    return np.convolve(x, k, mode='same')


rows = []
for r in runs:
    src = r['front'] * SC
    a = np.gradient(smooth(src, 11), 0.05)
    a[:20] = a[20]; a[-20:] = a[-20]
    n = r['notch']
    for u in range(-15, 16):
        m = (n == u) & (np.abs(np.gradient(smooth(src, 11), 0.05)) < 1.5)
        if m.sum() > 30:
            rows.append((u, src[m].mean(), a[m].mean(), a[m].std(), m.sum()))
rows = np.array(rows)
print('\n notch   v_mean    a_mean    a_std     n')
for u in range(-15, 16):
    s = rows[rows[:, 0] == u]
    if len(s):
        wgt = s[:, 4]
        am = np.average(s[:, 2], weights=wgt)
        vs = np.average(s[:, 1], weights=wgt)
        print(f' {int(u):4d}  {vs:7.2f}  {am:+7.3f}  {np.average(s[:,3],weights=wgt):7.3f}  {int(wgt.sum()):9d}')
