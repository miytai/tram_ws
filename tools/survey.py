#!/usr/bin/env python3
"""Quick survey of one bag: rates, ranges, notch/velocity relation."""
import sys
sys.path.insert(0, '/home/ros/tram_ws/tools')
import numpy as np
from bag_io import read_run, find_bags, gnss_speed, gnss_position

DATA = '/home/ros/Downloads/dataset/data'

bags = find_bags(DATA)
sel = sys.argv[1:] if len(sys.argv) > 1 else None
if sel:
    bags = [b for b in bags if any(s in b.name for s in sel)]

for b in bags[:4]:
    r = read_run(b)
    t, v = gnss_speed(r)
    print('=' * 70)
    print(r.name)
    print(f'  dur = {max(r.t["notch"][-1], r.t["front"][-1]):.1f} s')
    print(f'  front: n={len(r.v_front)} range=[{r.v_front.min():.3f},{r.v_front.max():.3f}] mean={r.v_front.mean():.3f}')
    print(f'  rear : n={len(r.v_rear)} range=[{r.v_rear.min():.3f},{r.v_rear.max():.3f}] mean={r.v_rear.mean():.3f}')
    print(f'  notch: n={len(r.notch)} range=[{r.notch.min():.0f},{r.notch.max():.0f}] uniq={sorted(set(r.notch.astype(int).ravel().tolist()))}')
    d = np.interp(r.t['front'], r.t['rear'], r.v_rear) - r.v_front
    print(f'  front-rear: mean={d.mean():.4f} std={d.std():.4f} max|.|={np.abs(d).max():.3f}')
    if t is not None:
        print(f'  gnss vel: range=[{v[:,0].min():.3f},{v[:,0].max():.3f}]  (x,y) std={v[:,1].std():.3f}')
        t0 = max(r.t['front'][0], t[0])
        vf = np.interp(t, r.t['front'], r.v_front)
        gv = np.interp(t, r.t['notch'], r.notch)
        m = (t > t0 + 5) & (t < t - 5)
        if m.sum() > 10:
            e = vf[m] - v[m, 0]
            print(f'  front-gnss_x: mean={e.mean():+.4f} std={e.std():.4f} rmse={np.sqrt((e**2).mean()):.4f}')
            e2 = vf[m] - np.hypot(v[m, 0], v[m, 1])
            print(f'  front-gnss_spd: mean={e2.mean():+.4f} std={e2.std():.4f}')
            print(f'  gnss |vy| mean={np.abs(v[m,1]).mean():.4f} max={np.abs(v[m,1]).max():.4f}')
            print(f'  gnss |vz| max={np.abs(v[m,2]).max():.4f}')
    p = gnss_position(r)
    if p is not None:
        tt, enu, _ = p
        print(f'  enu: x[{enu[:,0].min():.1f},{enu[:,0].max():.1f}] y[{enu[:,1].min():.1f},{enu[:,1].max():.1f}] z range {enu[:,2].min():.2f}..{enu[:,2].max():.2f}')
        d2 = np.hypot(np.diff(enu[:, 0]), np.diff(enu[:, 1]))
        print(f'  step dist: sum={d2.sum():.1f} m  max_step={d2.max():.2f}  (dt~{np.median(np.diff(tt)):.3f})')
