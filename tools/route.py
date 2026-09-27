#!/usr/bin/env python3
"""Route repeatability with smoothing + 2-point alignment."""
import sys, pickle
sys.path.insert(0, '/home/ros/tram_ws/tools')
import numpy as np

runs = pickle.load(open('/home/ros/tram_ws/calib/runs.pkl', 'rb'))


def path_of(r, ds_step=1.0, smooth_s=2.0):
    p = r['pos']
    d = np.diff(p[:, :2], axis=0)
    seg = np.hypot(d[:, 0], d[:, 1])
    good = seg < 5.0
    s = np.maximum.accumulate(np.concatenate([[0], np.cumsum(np.where(good, seg, 0.0))]))
    ok = np.concatenate([[True], good])
    x, y = p[ok, 0], p[ok, 1]
    ds = np.arange(0, s[ok][-1], ds_step)
    if len(ds) < 5:
        return None
    xi = np.interp(ds, s[ok], x); yi = np.interp(ds, s[ok], y)
    w = max(1, int(round(smooth_s / ds_step)))
    k = np.ones(w) / w
    xi = np.convolve(xi, k, mode='same'); yi = np.convolve(yi, k, mode='same')
    n = w
    xi[:n] = xi[n]; yi[:n] = yi[n]
    xi[-n:] = xi[-n - 1]; yi[-n:] = yi[-n - 1]
    return ds, xi, yi


def align(ref, cur, s1=50, s2=800):
    ds, rx, ry = ref
    _, cx, cy = cur
    i1 = int(np.argmin(abs(ds - s1))); i2 = int(np.argmin(abs(ds - s2)))
    if i2 - i1 < 5:
        return None
    ddx = rx[i2] - rx[i1]; ddy = ry[i2] - ry[i1]
    cdx = cx[i2] - cx[i1]; cdy = cy[i2] - cy[i1]
    th = np.arctan2(cdy, cdx) - np.arctan2(ddy, ddx)
    c, si = np.cos(-th), np.sin(-th)
    X = c * cx - si * cy; Y = si * cx + c * cy
    X += rx[i1] - X[i1]; Y += ry[i1] - Y[i1]
    return th, X, Y, ds


groups = {'neg': [], 'pos': []}
for r in runs:
    if 'pos' not in r:
        continue
    groups['neg' if r['pos'][-1, 0] < 0 else 'pos'].append(r)

paths = {}
for g, rs in groups.items():
    ref = max(rs, key=lambda r: r['pos'][-1, 0] * (1 if g == 'pos' else -1))
    R = path_of(ref)
    paths[g] = (ref['name'], R)
    print(f'=== group {g}: {len(rs)} runs, ref={ref["name"]} L={R[0][-1]:.0f} m ===')
    tot = []
    for r in rs:
        C = path_of(r)
        if C is None or C[0][-1] < 1500:
            continue
        al = align(R, C)
        if al is None:
            continue
        th, X, Y, ds = al
        n = min(len(X), len(R[1]))
        e = np.hypot(X[:n] - R[1][:n], Y[:n] - R[2][:n])
        tot.append((r['name'], e[-1], np.sqrt(np.mean(e ** 2)), np.percentile(e, 95), e.max(), np.degrees(th)))
    for t in tot[:14]:
        print(f'  {t[0]:16} rot={t[5]:+7.2f}  ctErr end={t[1]:7.1f} rmse={t[2]:6.1f} p95={t[3]:6.1f} max={t[4]:7.1f}')
    a = np.array([[t[1], t[2], t[3], t[4]] for t in tot])
    if len(a):
        print(f'  --> n={len(a)} end: med={np.median(a[:,0]):.1f} p90={np.percentile(a[:,0],90):.1f} max={a[:,0].max():.1f} | '
              f'rmse med={np.median(a[:,1]):.1f} p90={np.percentile(a[:,1],90):.1f} | max med={np.median(a[:,3]):.1f}')
