#!/usr/bin/env python3
"""Is the 2% path-length gap real? Compare GNSS-derived speed, GNSS path length, wheel."""
import sys, pickle
sys.path.insert(0, '/home/ros/tram_ws/tools')
import numpy as np

runs = pickle.load(open('/home/ros/tram_ws/calib/runs.pkl', 'rb'))
SC = 1.0 / 3.6

out = []
for r in runs:
    if 'pos' not in r or 'gv' not in r:
        continue
    if r['pos'][-1, 0] < 1000 and r['pos'][-1, 0] > -1000:
        continue
    p = r['pos']
    t = r['t']
    d = np.diff(p[:, :2], axis=0)
    seg = np.hypot(d[:, 0], d[:, 1])
    dt = np.diff(t)
    # robust: reject steps implying |a| > 5 m/s^2 relative to wheel-derived speed
    vw = r['front'][:-1] * SC
    vg = seg / np.maximum(dt, 1e-3)
    ok = (seg < 20) & (dt > 0.02) & (np.abs(vg - vw) < 3.0)
    path = seg[ok].sum()
    dur = dt[ok].sum()
    # speed comparison on clean points
    m = ok & (vg > 1.0)
    scale = (vw[m] * vg[m]).sum() / max((vg[m] ** 2).sum(), 1e-9)
    # gnss vel topic
    vt = np.abs(r['gv'] - 0)  # speed magnitude already stored
    out.append((r['name'], r['t'][-1], dur, path, vw[ok].mean() * 0 + np.trapz(vw, t) if False else np.trapz(r['front'] * SC, t),
                path / dur, scale, float(np.mean(vt))))
    print(f'{out[-1][0]:16} T={out[-1][1]:7.1f} path={path:8.1f} wheelD={out[-1][4]:8.1f} '
          f'ratio={path/out[-1][4]:6.4f} vg_path={path/dur:6.3f} scale_vs_posvel={scale:6.4f} '
          f'mean|gv|={out[-1][7]:6.3f}')

a = np.array([[x[3] / x[4], x[6]] for x in out])
print(f'\npath/wheel ratio : med {np.median(a[:,0]):.5f}  p10 {np.percentile(a[:,0],10):.5f}  p90 {np.percentile(a[:,0],90):.5f}')
print(f'wheel/posvel scale: med {np.median(a[:,1]):.5f}  p10 {np.percentile(a[:,1],10):.5f}  p90 {np.percentile(a[:,1],90):.5f}')
