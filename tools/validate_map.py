#!/usr/bin/env python3
"""Validate the route map: best rigid alignment of each dev run vs the map."""
import sys, json, pickle
sys.path.insert(0, '/home/ros/tram_ws/tools')
import numpy as np

SC = 1.0 / 3.6
runs = pickle.load(open('/home/ros/tram_ws/calib/runs.pkl', 'rb'))
M = json.load(open('/home/ros/tram_ws/src/tram_backup_odometry/config/route_map.json'))
mx = np.array(M['x']); my = np.array(M['y']); ms = np.array(M['s'])
L = ms[-1]


def track_pts(r):
    p = r['pos']
    v = np.convolve(r['front'] * SC, np.ones(21) / 21, mode='same')
    d = np.diff(p[:, :2], axis=0); seg = np.hypot(d[:, 0], d[:, 1]); dt = np.diff(r['t'])
    ok = (seg < 15) & (dt > 0.02) & (np.abs(seg / np.maximum(dt, 1e-3) - v[:-1]) < 2.0)
    q = np.r_[p[:1], p[1:][ok]][:, :2]
    s = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(q[:, 0]), np.diff(q[:, 1])))])
    return s, q


def sim2d(src, dst, w=None):
    """least-squares similarity (scale, rot, trans) mapping src -> dst"""
    if w is None:
        w = np.ones(len(src))
    w = w / w.sum()
    ms_ = (src * w[:, None]).sum(0); md = (dst * w[:, None]).sum(0)
    a = src - ms_; b = dst - md
    th = np.arctan2((w * (a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0])).sum(),
                    (w * (a[:, 0] * b[:, 0] + a[:, 1] * b[:, 1])).sum())
    c, s = np.cos(th), np.sin(th)
    R = np.array([[c, -s], [s, c]])
    Ra = a @ R.T
    den2 = (w[:, None] * a * a).sum()
    sc = (w[:, None] * b * Ra).sum() / den2 if den2 > 1e-9 else 1.0
    t = md - sc * (R @ ms_)
    return sc, th, t, R


res = []
for r in runs:
    if 'pos' not in r or abs(r['pos'][-1, 0]) < 800:
        continue
    s, q = track_pts(r)
    if s[-1] < 900:
        continue
    step = 2.0
    tgt = np.arange(0, s[-1], step)
    i = np.clip(np.searchsorted(s, tgt), 0, len(s) - 1)
    obs = q[i]
    best = None
    # coarse: try every 10 m start offset, both directions
    for rev in (False, True):
        for s0 in np.arange(0, L - s[-1] + 1, 10.0):
            idx = np.clip((s0 + tgt) / 1.0, 0, len(ms) - 1).astype(int)
            if rev:
                idx = (len(ms) - 1) - idx
            ref = np.column_stack([mx[idx], my[idx]])
            msk = ref.sum(1) != 0
            sc, th, t, R = sim2d(obs[msk], ref[msk])
            pr = (sc * (obs @ R.T) + t)
            e = np.hypot(pr[:, 0] - ref[:, 0], pr[:, 1] - ref[:, 1])
            rms = np.sqrt(np.mean(e ** 2))
            if best is None or rms < best[0]:
                best = (rms, e[-1], np.percentile(e, 95), e.max(), sc, np.degrees(th), s0, rev)
    res.append((r['name'],) + (best if best else (np.nan,) * 8))
    if best:
        print(f'{r["name"]:16} rms={best[0]:6.2f} end={best[1]:7.2f} p95={best[2]:6.2f} max={best[3]:6.2f} '
              f'scale={best[4]:.5f} rot={best[5]:+7.2f} s0={best[6]:7.1f} rev={best[7]}')

a = np.array([[x[1], x[2], x[3], x[4]] for x in res])
print(f'\nn={len(a)}  map-fit rms: med {np.median(a[:,0]):.2f} p90 {np.percentile(a[:,0],90):.2f} max {a[:,0].max():.2f}')
print(f'      end residual: med {np.median(a[:,1]):.2f} p90 {np.percentile(a[:,1],90):.2f} max {a[:,1].max():.2f}')
print(f'      scale: med {np.median(a[:,3]):.5f} p10 {np.percentile(a[:,3],10):.5f} p90 {np.percentile(a[:,3],90):.5f}')
