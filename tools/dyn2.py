#!/usr/bin/env python3
"""2D acceleration map a(notch, v) + slip statistics."""
import sys, pickle
sys.path.insert(0, '/home/ros/tram_ws/tools')
import numpy as np

runs = pickle.load(open('/home/ros/tram_ws/calib/runs.pkl', 'rb'))
SC = 1.0 / 3.6
NV = 15                      # speed bins 0..15 m/s
acc = np.zeros((31, NV))
cnt = np.zeros((31, NV))
for r in runs:
    v = np.convolve(r['front'] * SC, np.ones(11) / 11, mode='same')
    a = np.gradient(v, 0.05)
    ok = np.ones(len(v), bool)
    ok[:30] = False
    ok[-30:] = False
    n = np.round(r['notch']).astype(int)
    vb = np.clip(v.astype(int), 0, NV - 1)
    for u in range(-15, 16):
        m = ok & (n == u)
        if not m.any():
            continue
        np.add.at(acc, (u + 15, vb[m]), a[m])
        np.add.at(cnt, (u + 15, vb[m]), 1.0)
mn = cnt > 30
acc = np.where(mn, acc / np.maximum(cnt, 1), np.nan)
np.set_printoptions(linewidth=400, precision=2, suppress=True)
hdr = '  v\\u ' + ' '.join(f'{u:5d}' for u in range(-15, 16))
print(hdr)
for j in range(NV):
    print(f'  {j:3d} ' + ' '.join((f'{acc[i, j]:+5.2f}' if mn[i, j] else '    .') for i in range(31)))

print('\n coast (u=0):')
for j in range(NV):
    if mn[15, j]:
        print(f'   v~{j+0.5:4.1f}  a={acc[15, j]:+.4f}  n={int(cnt[15, j])}')

print('\n brake (u<0) at v=6..9 m/s:')
for u in range(-15, 0):
    row = ' '.join((f'{acc[u + 15, j]:+5.2f}' if mn[u + 15, j] else '    .') for j in (4, 6, 8, 10, 12, 14))
    print(f'   u={u:3d}  v=4:{row}')

print('\n power (u>0) at v=2,5,8,11,14:')
for u in range(1, 16):
    row = ' '.join((f'{acc[u + 15, j]:+5.2f}' if mn[u + 15, j] else '    .') for j in (1, 3, 5, 7, 9, 11, 13))
    print(f'   u={u:3d}  v=2:{row}')

big = tot = 0
for r in runs:
    d = (r['front'] - r['rear']) * SC
    big += int((np.abs(d) > 0.3).sum()); tot += len(d)
print(f'\n|front-rear|>0.3 m/s: {100.0*big/tot:.3f} % ({big}/{tot})')
