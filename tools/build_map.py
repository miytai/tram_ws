#!/usr/bin/env python3
"""Build the canonical route map + measure cross-run repeatability."""
import sys, pickle
sys.path.insert(0, '/home/ros/tram_ws/tools')
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

runs = pickle.load(open('/home/ros/tram_ws/calib/runs.pkl', 'rb'))
SC = 1.0 / 3.6


def wheel_s(r):
    v = r['front'] * SC
    t = r['t']
    return np.concatenate([[0], np.cumsum(0.5 * (v[:-1] + v[1:]) * np.diff(t))])


def resample_by_s(r, s, step=1.0):
    tgt = np.arange(0, s[-1], step)
    i = np.clip(np.searchsorted(s, tgt), 0, len(s) - 1)
    return tgt, r['pos'][i, 0], r['pos'][i, 1], r['pos'][i, 2]


groups = {'neg': [], 'pos': []}
for r in runs:
    if 'pos' not in r:
        continue
    groups['neg' if r['pos'][-1, 0] < 0 else 'pos'].append(r)

ref = max(groups['pos'], key=lambda r: r['pos'][-1, 0])
sR = wheel_s(ref)
sR, xR, yR, zR = resample_by_s(ref, sR)
L = len(sR)
print(f'ref={ref["name"]} L={sR[-1]:.1f} m, {L} nodes')

hR = np.unwrap(np.arctan2(np.gradient(yR, sR), np.gradient(xR, sR)))
kR = np.gradient(hR, sR)

fig, ax = plt.subplots(4, 1, figsize=(15, 14))
ax[0].plot(xR, yR); ax[0].set_aspect('equal'); ax[0].grid(True); ax[0].set_title('canonical route')
ax[1].plot(sR, np.degrees(hR)); ax[1].grid(True); ax[1].set_ylabel('heading [deg]')
ax[2].plot(sR, kR); ax[2].grid(True); ax[2].set_ylabel('curvature [1/m]')
ax[3].plot(sR, zR); ax[3].grid(True); ax[3].set_ylabel('altitude [m]'); ax[3].set_xlabel('s [m]')
plt.tight_layout(); plt.savefig('/home/ros/tram_ws/calib/route_detail.png', dpi=90)

print('\ns[m]     x        y       hdg[deg]  curv[1/km]   z')
for i in range(0, L, 50):
    print(f'{sR[i]:6.0f} {xR[i]:9.2f} {yR[i]:9.2f} {np.degrees(hR[i]):9.2f} {kR[i]*1000:+10.3f} {zR[i]:8.2f}')

# repeatability within the 'pos' group, aligned on wheel s
errs = []
for r in groups['pos']:
    if r is ref:
        continue
    s2 = wheel_s(r)
    s2, x2, y2, _ = resample_by_s(r, s2)
    m = min(L, len(s2)) - 1
    if m < 2000:
        continue
    i1, i2 = 100, 900
    th = np.arctan2(y2[i2] - y2[i1], x2[i2] - x2[i1]) - np.arctan2(yR[i2] - yR[i1], xR[i2] - xR[i1])
    c, si = np.cos(-th), np.sin(-th)
    X = c * x2[:m] - si * y2[:m]; Y = si * x2[:m] + c * y2[:m]
    X += xR[i1] - X[i1]; Y += yR[i1] - Y[i1]
    e = np.hypot(X - xR[:m], Y - yR[:m])
    errs.append((r['name'], np.degrees(th), e[0], e[1000], e[-1], np.sqrt(np.mean(e ** 2)), e.max()))
a = np.array([[x[3], x[4], x[5], x[6]] for x in errs])
print(f'\npos-group repeatability (aligned on wheel-s, rot from s=100/900): n={len(a)}')
print(f'  |err| @1km: med {np.median(a[:,0]):.2f} p90 {np.percentile(a[:,0],90):.2f} max {a[:,0].max():.2f}')
print(f'  |err| @end: med {np.median(a[:,1]):.2f} p90 {np.percentile(a[:,1],90):.2f} max {a[:,1].max():.2f}')
print(f'  rmse      : med {np.median(a[:,2]):.2f} p90 {np.percentile(a[:,2],90):.2f} max {a[:,2].max():.2f}')

# is 'neg' == rot180('pos')?
refn = max(groups['neg'], key=lambda r: -r['pos'][-1, 0])
sn = wheel_s(refn)
sn, xn, yn, _ = resample_by_s(refn, sn)
m = min(L, len(sn)) - 1
X = -xn[:m]; Y = -yn[:m]
X += xR[0] - X[0]; Y += yR[0] - Y[0]
e = np.hypot(X - xR[:m], Y - yR[:m])
print(f'\nneg({refn["name"]}) vs rot180(pos-ref): end {e[-1]:.1f} m rmse {np.sqrt(np.mean(e**2)):.1f} max {e.max():.1f}')
