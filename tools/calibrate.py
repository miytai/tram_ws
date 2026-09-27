#!/usr/bin/env python3
"""Final offline calibration -> calib/model.json

  1. nominal non-linear acceleration characteristic a_nom(u, v)  (31 x NB table)
  2. running-resistance coefficients  F_res(v) = c0 + c1 v + c2 v^2
  3. wheel-unit scale that maps /vehicle/*_bogie_velocity -> m/s
"""
import sys, json, pickle
sys.path.insert(0, '/home/ros/tram_ws/tools')
import numpy as np
from scipy.signal import medfilt
from scipy.ndimage import distance_transform_edt

SC = 1.0 / 3.6
NB = 16                      # speed bins [0,1), [1,2), ... [15,16)
M0 = 40000.0
G = 9.80665
runs = pickle.load(open('/home/ros/tram_ws/calib/runs.pkl', 'rb'))


def boxcar(x, w):
    k = np.ones(w) / w
    return np.convolve(np.pad(x, (w // 2, w // 2), mode='edge'), k, mode='valid')


acc = np.zeros((31, NB))
wgt = np.zeros((31, NB))
for r in runs:
    v = boxcar(r['front'] * SC, 11)          # 0.55 s boxcar
    a = np.gradient(v, 0.05)
    n = medfilt(np.round(r['notch']), 5).astype(int)   # drop single-sample command glitches
    ok = np.ones(len(v), bool)
    ok[:40] = False
    ok[-40:] = False
    vb = np.clip(v.astype(int), 0, NB - 1)
    for u in range(-15, 16):
        m = ok & (n == u)
        if not m.any():
            continue
        np.add.at(acc, (u + 15, vb[m]), a[m] * m[m])
        np.add.at(wgt, (u + 15, vb[m]), m[m].astype(float))

mn = wgt > 120
TAB = np.where(mn, acc / np.maximum(wgt, 1e-9), np.nan)

# ---- fill missing cells with the nearest valid cell (2-D) ------------------
ind = distance_transform_edt(~mn, return_distances=False, return_indices=True)
TAB = TAB[ind[0], ind[1]]

# ---- constant-power extension into the (empty) top speed bin --------------
# a(.,NB-1) = a(.,NB-2) * (NB-2)/(NB-1)  -- keeps the force, not the power
TAB[:, NB - 1] = TAB[:, NB - 2] * (NB - 2.0) / (NB - 1.0)

# ---- light 3x3 smoothing only on the cells that were not directly measured -
sm = np.zeros_like(TAB)
for i in range(31):
    for j in range(NB):
        p = TAB[max(i - 1, 0):i + 2, max(j - 1, 0):j + 2]
        sm[i, j] = p.mean()
TAB = np.where(mn, TAB, 0.5 * TAB + 0.5 * sm)
TAB[15, :] = np.minimum(TAB[15, :], 0.0)          # neutral notch never accelerates

# ------------------------------------------------------- 2. running resistance
cj = [j for j in range(3, NB - 1) if mn[15, j]]
cv = np.array([j + 0.5 for j in cj])
ca = np.array([TAB[15, j] for j in cj])
A = np.column_stack([np.ones_like(cv), cv, cv ** 2])
sol = np.linalg.solve(A.T @ A + np.diag([1.0, 0.02, 0.02]), A.T @ (-ca))
c0m, c1m, c2m = sol
print(f'F_res/m : c0={c0m:.5f}  c1={c1m:.6f}  c2={c2m:.7f}   '
      f'F_res(14)/m={-(c0m + 14 * c1m + 196 * c2m):.5f}  -> Crr={c0m / 9.80665:.5f}')

# --------------------------------------------------- 3. wheel unit scale (LS)
num = den = 0.0
for r in runs:
    if 'gv' not in r:
        continue
    m = r['gv'] > 1.0
    num += float((r['front'][m] * SC * r['gv'][m]).sum())
    den += float((r['gv'][m] ** 2).sum())
scale = num / den
print(f'wheel unit scale: {scale:.6f}  (1.0 => raw/3.6 is m/s)')

model = {
    'mass_kg': M0,
    'wheel_unit_scale': scale,
    'notch_min': -15, 'notch_max': 15,
    'speed_bin_m': 1.0, 'speed_bins': NB,
    'resistance_c0_N': float(c0m * M0),
    'resistance_c1_Nps': float(c1m * M0),
    'resistance_c2_Nps2': float(c2m * M0),
    'accel_table': [[float(x) for x in row] for row in TAB],
    'accel_table_support': [[bool(x) for x in row] for row in mn],
    'max_calib_speed': float(NB - 1),
    'gravity': G,
}
json.dump(model, open('/home/ros/tram_ws/calib/model.json', 'w'), indent=1)
print('written calib/model.json')
np.set_printoptions(linewidth=300, precision=3, suppress=True)
print('\nfinal a_nom(u,v) [m/s^2]  rows v-bin 0..15, cols u=-15..15  (blank = interpolated)')
print(np.array2string(TAB, precision=2, max_line_width=300))
