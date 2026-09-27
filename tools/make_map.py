#!/usr/bin/env python3
"""Build the track (pathgraph) map for the two vehicles from the dev dataset.

Output: <pkg>/config/route_map.json  - polyline nodes at 1 m spacing in a
canonical local frame, with tangent heading and a (heavily smoothed) altitude
profile.  The node uses it for heading and absolute positioning after the
initial GNSS-based alignment; it is never used inside the velocity estimator.
"""
import sys, json, pickle
sys.path.insert(0, '/home/ros/tram_ws/tools')
import numpy as np

SC = 1.0 / 3.6
OUT = '/home/ros/tram_ws/src/tram_backup_odometry/config/route_map.json'
runs = pickle.load(open('/home/ros/tram_ws/calib/runs.pkl', 'rb'))
by_name = {r['name']: r for r in runs}


def boxcar(x, w):
    k = np.ones(w) / w
    return np.convolve(np.pad(x, (w // 2, w // 2), mode='edge'), k, mode='valid')


def wheel_len(r):
    v = r['front'] * SC
    return float(np.trapz(v, r['t']))


def clean_track(r):
    """1 m polyline of one run, using GNSS positions, jump-rejected."""
    p = r['pos']
    v = boxcar(r['front'] * SC, 21)
    d = np.diff(p[:, :2], axis=0)
    seg = np.hypot(d[:, 0], d[:, 1])
    dt = np.diff(r['t'])
    ok = (seg < 15) & (dt > 0.02) & (np.abs(seg / np.maximum(dt, 1e-3) - v[:-1]) < 2.0)
    q = np.r_[p[:1], p[1:][ok]]
    z = np.r_[p[:1, 2], p[1:, 2][ok]]
    s = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(q[:, 0]), np.diff(q[:, 1])))])
    return s, q, z


def resample(s, q, z, step=1.0):
    tgt = np.arange(0.0, s[-1], step)
    i = np.clip(np.searchsorted(s, tgt), 0, len(s) - 1)
    return tgt, q[i, 0], q[i, 1], z[i]


def direction(nm):
    return float(np.sign(by_name[nm]['pos'][-1, 0]))


def build_branch(cand, label):
    L, name, s0, q0, z0 = max(cand, key=lambda c: c[0])
    sel = [c for c in cand if c[0] >= 0.90 * L][:24]
    n = int(L)
    st = []
    for _, nm, s, q, z in sel:
        _, xx, yy, zz = resample(s, q, z)
        m = min(len(xx), n)
        fx = np.full(n, np.nan); fy = np.full(n, np.nan); fz = np.full(n, np.nan)
        fx[:m], fy[:m], fz[:m] = xx[:m], yy[:m], zz[:m]
        st.append((fx, fy, fz))
    SX = np.stack([a[0] for a in st])
    SY = np.stack([a[1] for a in st])
    SZ = np.stack([a[2] for a in st])

    def fill(A):
        out = np.zeros(A.shape[1])
        for i in range(A.shape[1]):
            col = A[:, i][~np.isnan(A[:, i])]
            out[i] = np.median(col) if len(col) else np.nan
        idx = np.arange(len(out))
        good = ~np.isnan(out)
        return np.interp(idx, idx[good], out[good])

    X, Y, Z = fill(SX), fill(SY), fill(SZ)
    for _ in range(2):
        X = 0.25 * np.r_[X[0], X[:-1]] + 0.5 * X + 0.25 * np.r_[X[1:], X[-1]]
        Y = 0.25 * np.r_[Y[0], Y[:-1]] + 0.5 * Y + 0.25 * np.r_[Y[1:], Y[-1]]
    S = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(X), np.diff(Y)))])
    keep = np.concatenate([[True], np.diff(S) > 1e-9])
    S, X, Y, Z = S[keep], X[keep], Y[keep], Z[keep]
    tgt = np.arange(0.0, S[-1], 1.0)
    i = np.clip(np.searchsorted(S, tgt), 0, len(S) - 1)
    S, X, Y, Z = tgt, X[i], Y[i], Z[i]
    S = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(X), np.diff(Y)))])
    for _ in range(2):
        Z = np.array([np.median(Z[max(k - 15, 0):k + 16]) for k in range(len(Z))])
    Z = boxcar(Z, 101)
    print(f'  branch {label}: seed {name} ({L:.0f} m), merged {len(sel)} runs -> '
          f'{len(S)} nodes, {S[-1]:.0f} m')
    return S, X, Y, Z


cand = []
for r in runs:
    if 'pos' not in r or abs(r['pos'][-1, 0]) < 1500:
        continue
    s, q, z = clean_track(r)
    wl = wheel_len(r)
    if not (3000.0 < s[-1] < 8000.0):
        continue
    if abs(s[-1] - wl) / max(wl, 1.0) > 0.025:      # GNSS glitch run
        continue
    cand.append((s[-1], r['name'], s, q, z))
print(f'{len(cand)} glitch-free candidates with a GNSS reference')

branches = []
for d, label in ((-1.0, 'a'), (1.0, 'b')):
    sub = [c for c in cand if direction(c[1]) == d]
    if len(sub) < 3:
        continue
    S, X, Y, Z = build_branch(sub, label)
    branches.append({
        'name': f'depot_track_{label}',
        'frame': 'map',
        'description': 'tram depot test track, polyline resampled to 1 m, built '
                       'offline from the development dataset GNSS (RTK grade)',
        'step_m': 1.0,
        'length_m': float(S[-1]),
        'origin': [float(X[0]), float(Y[0]), float(Z[0])],
        's': [round(float(v), 3) for v in S],
        'x': [round(float(v), 4) for v in X],
        'y': [round(float(v), 4) for v in Y],
        'z': [round(float(v), 4) for v in Z],
    })

out = {'version': 2,
       'description': 'tram depot test track, both travel directions; polyline '
                      'nodes at 1 m spacing built offline from the development '
                      'dataset GNSS',
       'branches': branches}
json.dump(out, open(OUT, 'w'))
print(f'written {OUT} ({len(branches)} branches)')

