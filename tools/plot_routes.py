#!/usr/bin/env python3
"""Plot GNSS trajectories per group to see route geometry and noise."""
import sys, pickle
sys.path.insert(0, '/home/ros/tram_ws/tools')
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

runs = pickle.load(open('/home/ros/tram_ws/calib/runs.pkl', 'rb'))
fig, axes = plt.subplots(1, 3, figsize=(21, 7))
groups = {'neg': [], 'pos': []}
for r in runs:
    if 'pos' not in r:
        continue
    groups['neg' if r['pos'][-1, 0] < 0 else 'pos'].append(r)

for ax, (g, rs) in zip(axes, groups.items()):
    n = 0
    for r in sorted(rs, key=lambda r: -r['pos'][-1, 0].__abs__())[:40]:
        if r['pos'][-1, 0] * (1 if g == 'pos' else -1) < 500:
            continue
        ax.plot(r['pos'][:, 0], r['pos'][:, 1], lw=0.6, alpha=0.6)
        n += 1
        if n > 12:
            break
    ax.set_title(f'group {g}: GNSS raw paths ({n})')
    ax.set_aspect('equal'); ax.grid(True)

ax = axes[2]
r = max(groups['neg'], key=lambda r: r['pos'][-1, 0])
ax.plot(r['pos'][:, 0], r['pos'][:, 1], lw=0.7, label='GNSS path')
ax.plot(r['pos'][:, 2] * 1, np.zeros(len(r['pos'])), lw=0.5, label='z (m) x10')
ax.set_title(f'{r["name"]} GNSS path + altitude')
ax.set_aspect('equal'); ax.legend(); ax.grid(True)
plt.tight_layout(); plt.savefig('/home/ros/tram_ws/calib/routes.png', dpi=90)
print('ok')

# noise estimate: high-frequency content of GNSS positions
r = groups['neg'][0]
p = r['pos']
d2 = p[2:2000] - 2 * p[1:1999] + p[0:1998]
print('2nd-difference rms per axis (m):', np.sqrt((d2 ** 2).mean(axis=0)), ' -> sigma ~', np.sqrt((d2 ** 2).mean(axis=0)) / np.sqrt(6))
