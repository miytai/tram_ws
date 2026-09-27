#!/usr/bin/env python3
"""Survey all bags: route repeatability, start/end heading, geometry."""
import sys
sys.path.insert(0, '/home/ros/tram_ws/tools')
import numpy as np
from bag_io import read_run, find_bags, gnss_position

DATA = '/home/ros/Downloads/dataset/data'
bags = find_bags(DATA)


def clean(enu, t):
    st = np.hypot(np.diff(enu[:, 0]), np.diff(enu[:, 1]))
    keep = np.ones(len(t), bool)
    idx = np.where(st > 25)[0]
    for i in idx:
        keep[i + 1] = False
    return enu[keep], t[keep]


def heading_over(t, e, t0, t1):
    m = (t >= t0) & (t <= t1)
    if m.sum() < 5:
        return None, 0
    x, y = e[m, 0], e[m, 1]
    x = x - x.mean(); y = y - y.mean()
    sxx = float((x * x).sum()); syy = float((y * y).sum()); sxy = float((x * y).sum())
    return float(np.degrees(0.5 * np.arctan2(2 * sxy, sxx - syy))), int(m.sum())


rows = []
for b in bags:
    r = read_run(b)
    p = gnss_position(r)
    tf = r.t['front']
    dur = max(tf[-1], r.t['notch'][-1])
    wd = np.trapz(r.v_front, tf) / 3.6
    if p is None:
        rows.append((r.name, dur, wd, None, None, None, None, 0))
        continue
    t, enu, _ = p
    e, tc = clean(enu, t)
    s = np.concatenate([[0], np.cumsum(np.hypot(np.diff(e[:, 0]), np.diff(e[:, 1])))])
    h0, _ = heading_over(tc, e, t[0], t[0] + 25)
    h1, _ = heading_over(tc, e, t[-1] - 25, t[-1])
    # displacement at same arc length 3000 m if possible
    k = np.searchsorted(s, min(3000.0, s[-1] * 0.95))
    rows.append((r.name, dur, wd, s[-1], h0, h1, (e[k, 0], e[k, 1]), 0))

print(f'{"bag":16}{"dur":>8}{"wheelDist":>11}{"gnssPath":>10}{"h0":>8}{"h1":>8}   pos@3km')
for r in rows:
    h0 = f'{r[4]:8.1f}' if r[4] is not None else '     n/a'
    h1 = f'{r[5]:8.1f}' if r[5] is not None else '     n/a'
    gp = f'{r[3]:10.1f}' if r[3] is not None else '       n/a'
    pp = f'({r[6][0]:8.1f},{r[6][1]:8.1f})' if r[6] is not None else ''
    print(f'{r[0]:16}{r[1]:8.1f}{r[2]:11.1f}{gp}{h0}{h1}   {pp}')

withg = [r for r in rows if r[3]]
print('\n--- route endpoints (pos@3km) stats ---')
a = np.array([r[6] for r in withg])
print('x: min %.1f max %.1f  y: min %.1f max %.1f  n=%d' % (a[:, 0].min(), a[:, 0].max(), a[:, 1].min(), a[:, 1].max(), len(a)))
print('wheel dist: med %.1f  gnss path: med %.1f  ratio med %.4f' % (
    np.median([r[2] for r in withg]), np.median([r[3] for r in withg]),
    np.median([r[2] / r[3] for r in withg if r[3] > 100])))
