#!/usr/bin/env python3
import sys
sys.path.insert(0, '/home/ros/tram_ws/tools')
import numpy as np
from bag_io import read_run, find_bags, gnss_speed, gnss_position

DATA = '/home/ros/Downloads/dataset/data'
for pat in (sys.argv[1:] or ['30618_0e41eac3']):
    b = [x for x in find_bags(DATA) if pat in x.name][0]
    r = read_run(b)
    print('=' * 72)
    print(r.name)
    for k, val in r.gnss.items():
        if k == 't0_rel' or not len(val[0]):
            continue
        t, _ = val
        d = np.diff(t)
        print(f'  {k}: n={len(t)} t0={t[0]:.2f} t1={t[-1]:.2f} dt med={np.median(d):.3f} gaps>1s={int((d>1).sum())}')
    tf = r.t['front']
    print(f'  wheel: n={len(tf)} span={tf[-1]-tf[0]:.1f}s int_dist={np.trapz(r.v_front, tf):.1f} max={r.v_front.max():.3f}')

    t0, enu, _ = gnss_position(r)
    rel = r.gnss['t0_rel']
    t0a = t0 + rel
    step = np.hypot(np.diff(enu[:, 0]), np.diff(enu[:, 1]))
    vfix = step / np.diff(t0)
    print(f'  enu path={step.sum():.1f} m   v_fix: med={np.median(vfix):.3f} p95={np.percentile(vfix,95):.3f} max={vfix.max():.3f} m/s')
    w_fix = 0.5 * (np.interp(t0[:-1], tf, r.v_front) + np.interp(t0[1:], tf, r.v_front))
    m = (vfix > 0.3) & (vfix < 60)
    print(f'  wheel vs v_fix: ratio med={np.median(w_fix[m]/vfix[m]):.4f} mean={np.mean(w_fix[m]/vfix[m]):.4f} std={np.std(w_fix[m]/vfix[m]):.4f}')

    t, v = gnss_speed(r)
    va = t + rel
    sp = np.hypot(v[:, 0], v[:, 1])
    print(f'  vel_topic: |v| med={np.median(sp):.3f} p95={np.percentile(sp,95):.3f} max={sp.max():.3f}')
    wv = np.interp(va, tf, r.v_front)
    for k in (1.0, 3.6, 1 / 3.6, 2.0):
        m2 = sp > 1.0
        d = wv[m2] / k - sp[m2]
        print(f'   wheel/{k:5.2f} vs vel: bias={d.mean():+.4f} std={d.std():.4f} rmse={np.sqrt((d**2).mean()):.4f}')
    w = wv
    print('  sample rows:  t   enuX enuY | vfix | wheel | notch | velx vely |speed|')
    for i in range(0, len(t0a), max(1, len(t0a) // 20)):
        print(f'   {t0a[i]-t0a[0]:8.1f} {enu[i,0]:9.1f} {enu[i,1]:8.1f} | {vfix[min(i,len(vfix)-1)]:6.2f} |'
              f' {w[i]:6.2f} | {np.interp(t0a[i], r.t["notch"], r.notch):5.1f} | {v[i,0]:7.2f} {v[i,1]:6.2f} | {sp[i]:6.2f}')
