#!/usr/bin/env python3
"""Two-pass fit of the nonlinear traction / longitudinal-dynamics model.

    a(u,v) = [ Ft(u,v) - Fb(u,v) - Fres(v) ] / m      (u = driver notch, v [m/s])

    Ft(u,v) = T(u) / (1 + (v/vt)^pt)          u > 0   T  = tractive force at rail [N]
    Fb(u,v) = B(u) / (1 + (v/vb)^pb)          u < 0   B  = brake force at rail [N]
    Fres(v) = c0 + c1 v + c2 v^2                     running resistance [N]

Pass 1 fits the model, pass 2 removes the (spatially varying) track grade from the
data using the model residual and refits - the same mechanism the node runs online.
"""
import sys, pickle, json
sys.path.insert(0, '/home/ros/tram_ws/tools')
import numpy as np
from scipy.optimize import least_squares

runs = pickle.load(open('/home/ros/tram_ws/calib/runs.pkl', 'rb'))
SC = 1.0 / 3.6
M0 = 40000.0
NV = 15

# ---------------------------------------------------------------- per-run signals
for r in runs:
    v = np.convolve(r['front'] * SC, np.ones(11) / 11, mode='same')
    a = np.gradient(v, 0.05)
    ok = np.ones(len(v), bool); ok[:30] = False; ok[-30:] = False
    r['_v'], r['_a'], r['_ok'] = v, a, ok
    r['_n'] = np.round(r['notch']).astype(int)
    r['_vb'] = np.clip(v.astype(int), 0, NV - 1)


def build_table(corr=None):
    acc = np.zeros((31, NV)); wgt = np.zeros((31, NV))
    for r in runs:
        a = r['_a'] if corr is None else r['_a'] - corr[r['name']]
        for u in range(-15, 16):
            m = r['_ok'] & (r['_n'] == u)
            if not m.any():
                continue
            np.add.at(acc, (u + 15, r['_vb'][m]), a[m] * m[m])
            np.add.at(wgt, (u + 15, r['_vb'][m]), m[m].astype(float))
    mn = wgt > 200
    A = np.where(mn, acc / np.maximum(wgt, 1e-9), np.nan)
    return A, mn, wgt


UI, MI = np.meshgrid(np.arange(31), np.arange(NV), indexing='ij')
u = (UI - 15.0).astype(float)
vv = MI + 0.5
absu = np.abs(u)
pos, neg = u > 0, u < 0
Tsel = np.where(pos, absu - 1, 0.0)
Bsel = np.where(neg, absu - 1, 0.0)


def model_a(par, U=u, V=vv):
    pt, vt, pb, vb, c0, c1, c2 = par[:7]
    T, B = par[7:22], par[22:37]
    st = 1.0 / (1.0 + (V / vt) ** pt)
    sb = 1.0 / (1.0 + (V / vb) ** pb)
    Fa = np.where(pos, np.interp(Tsel, np.linspace(0, 1, 15), T) * st, 0.0) \
        - np.where(neg, np.interp(Bsel, np.linspace(0, 1, 15), B) * sb, 0.0)
    return (Fa - (c0 + c1 * V + c2 * V ** 2)) / M0


def fit(A, mn, wgt, x0=None):
    W = np.sqrt(np.minimum(wgt, 4000.0))
    Wm = W[mn]

    def resid(par):
        return ((model_a(par) - A) * W)[mn] / Wm * 1.0

    if x0 is None:
        col = 6
        T0 = np.clip(np.nan_to_num(A[16:, col], nan=0.0), 0, None) * M0
        B0 = np.clip(np.nan_to_num(-A[:15, col], nan=0.0), 0, None) * M0
        T0 = np.convolve(np.pad(T0, 3, mode='edge'), np.ones(7) / 7, mode='valid')
        B0 = np.convolve(np.pad(B0, 3, mode='edge'), np.ones(7) / 7, mode='valid')
        x0 = np.concatenate([[1.0, 12.0, 1.0, 12.0, 800.0, 0.0, 100.0], T0, B0])
    lo = np.concatenate([[0.2, 1.0, 0.2, 1.0, 0.0, -600.0, -3000.0], np.zeros(15), np.zeros(15)])
    hi = np.concatenate([[6.0, 60.0, 6.0, 60.0, 5000.0, 600.0, 3000.0],
                        np.full(15, 9e4), np.full(15, 9e4)])
    best = None
    for pt0, pb0, vt0, vb0 in ((1.0, 1.0, 12.0, 12.0), (2.0, 2.0, 20.0, 20.0),
                               (1.0, 0.5, 8.0, 25.0), (1.5, 1.5, 30.0, 8.0)):
        x = x0.copy()
        x[0], x[1], x[2], x[3] = pt0, vt0, pb0, vb0
        x = np.clip(x, lo, hi)
        s = least_squares(resid, x, bounds=(lo, hi), loss='soft_l1', f_scale=0.06,
                          xtol=1e-14, ftol=1e-14, max_nfev=400000)
        if best is None or s.cost < best.cost:
            best = s
    return best


A1, mn1, w1 = build_table()
s1 = fit(A1, mn1, w1)
r1 = (model_a(s1.x) - A1)[mn1]
print(f'pass1: rmse={np.sqrt(np.mean(r1**2)):.4f}  p={s1.x[0]:.2f} vt={s1.x[1]:.1f} '
      f'pb={s1.x[2]:.2f} vb={s1.x[3]:.1f} c0={s1.x[4]:.0f} c1={s1.x[5]:.0f} c2={s1.x[6]:.0f}')

GRID = np.linspace(0, 1, 15)


def model_run(par, r):
    """a_model(u, v) for one run's samples (vectorised)."""
    pt, vt, pb, vb, c0, c1, c2 = par[:7]
    T, B = par[7:22], par[22:37]
    v = np.maximum(r['_v'], 0.0)
    n = r['_n']
    st = 1.0 / (1.0 + (v / vt) ** pt)
    sb = 1.0 / (1.0 + (v / vb) ** pb)
    Fa = np.where(n > 0, np.interp(np.clip(n, 1, 15) - 1, GRID, T) * st, 0.0) \
        - np.where(n < 0, np.interp(np.clip(-n, 1, 15) - 1, GRID, B) * sb, 0.0)
    return (Fa - (c0 + c1 * v + c2 * v ** 2)) / M0

# ------------------------------------------------- pass 2: remove slow model bias
par = s1.x.copy()
for it in range(3):
    corr = {}
    for r in runs:
        am = model_run(par, r)
        res = r['_a'] - am
        # slow low-pass of the residual = grade + model bias (10 s window)
        k = np.ones(201) / 201
        lp = np.convolve(np.pad(res, (100, 100), mode='edge'), k, mode='valid')
        corr[r['name']] = np.where(r['_ok'], lp, 0.0)
    A2, mn2, w2 = build_table(corr)
    s2 = fit(A2, mn2, w2, x0=par)
    r2 = (model_a(s2.x) - A2)[mn2]
    gsig = np.concatenate([corr[r['name']][r['_ok']] for r in runs])
    print(f'pass2.{it}: rmse={np.sqrt(np.mean(r2**2)):.4f}  slow-residual std={gsig.std():.4f}  cost={s2.cost:.3f}')
    par = s2.x

pt, vt, pb, vb, c0, c1, c2 = par[:7]
print(f'\nFINAL  pt={pt:.3f} vt={vt:.2f}  pb={pb:.3f} vb={vb:.2f}')
print(f'Fres: c0={c0:.1f}  c1={c1:.1f}  c2={c1*0+c2:.1f}  -> at v=14: {c0+14*c1+196*c2:.0f} N')
print('T(u) kN:', np.round(par[7:22] / 1000, 2))
print('B(u) kN:', np.round(par[22:37] / 1000, 2))
Ahat = model_a(par)
print('\nresidual table [m/s^2] rows v=0.5..14.5, cols u=-15..15')
print(np.array2string(np.where(mn2, A2 - Ahat, np.nan), precision=2, max_line_width=260))
print('\nmeasured a(u,v) [m/s^2]')
print(np.array2string(np.where(mn2, A2, np.nan), precision=2, max_line_width=260))

# weighted residual std per notch -> noise floor
sd = []
for i in range(31):
    m = mn2[i]
    if m.sum() > 3:
        sd.append((i - 15, np.std((A2 - Ahat)[i][m])))
print('\nper-notch residual std:', ' '.join(f'{int(u)}:{s:.3f}' for u, s in sd))

json.dump({'pt': float(pt), 'vt': float(vt), 'pb': float(pb), 'vb': float(vb),
           'c0': float(c0), 'c1': float(c1), 'c2': float(c2),
           'T': par[7:22].tolist(), 'B': par[22:37].tolist(),
           'fit_mass': M0, 'rmse_acc': float(np.sqrt(np.mean(r2 ** 2)))},
          open('/home/ros/tram_ws/calib/model_fit.json', 'w'), indent=1)
print('\nwritten calib/model_fit.json')
