"""Offline evaluation of the estimator against the GNSS reference.

Used for the accuracy tables of the submission.  It replays the three
permitted input topics of a bag through the *same* estimator code that the
node uses (no ROS middleware), and compares the result with the GNSS
telemetry:

    velocity   RMSE / MAE / bias / max, and the same split into
               accelerating / braking / steady phases
    position   along-track error, cross-track error, 3-D distance,
               accumulated drift in % of the travelled distance
    latency    input stamp -> output stamp
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


def enu(lat, lon, alt, lat0, lon0, alt0):
    a = 6378137.0
    e2 = 6.69437999014e-3
    sl = math.sin(math.radians(lat0))
    n = a / math.sqrt(1.0 - e2 * sl * sl)
    return (math.radians(lon - lon0) * n * math.cos(math.radians(lat0)),
            math.radians(lat - lat0) * n, alt - alt0)


@dataclass
class Score:
    v_rmse: float = float('nan')
    v_mae: float = float('nan')
    v_bias: float = float('nan')
    v_max: float = float('nan')
    v_rmse_acc: float = float('nan')
    v_rmse_brk: float = float('nan')
    v_rmse_coast: float = float('nan')
    pos_rmse: float = float('nan')
    pos_max: float = float('nan')
    along_mean: float = float('nan')
    along_max: float = float('nan')
    cross_rms: float = float('nan')
    drift_pct: float = float('nan')
    dist: float = float('nan')
    matched: bool = False
    slip_fraction: float = float('nan')


def score(est_t, est_v, est_p, ref_t, ref_v, ref_p, notch, slip):
    """est_*/ref_* are 1-D / (N,3) arrays on a common time base."""
    s = Score()
    if len(est_t) == 0:
        return s
    m = np.isfinite(est_v) & np.isfinite(ref_v)
    if m.sum() < 10:
        return s
    e = est_v[m] - ref_v[m]
    s.v_rmse = float(np.sqrt(np.mean(e ** 2)))
    s.v_mae = float(np.mean(np.abs(e)))
    s.v_bias = float(np.mean(e))
    s.v_max = float(np.max(np.abs(e)))
    nn = np.interp(est_t[m], est_t[m], notch[:m.sum()]) if len(notch) == m.sum() else None
    if nn is not None:
        s.v_rmse_acc = _rmse(e, nn > 0)
        s.v_rmse_brk = _rmse(e, nn < 0)
        s.v_rmse_coast = _rmse(e, nn == 0)
    if len(slip) == m.sum():
        s.slip_fraction = float(np.mean(slip.astype(bool)))
    if est_p is not None and ref_p is not None and len(ref_p) == len(est_t):
        d = est_p - ref_p
        dist = np.linalg.norm(d[:, :2], axis=1)
        s.pos_rmse = float(np.sqrt(np.mean(dist ** 2)))
        s.pos_max = float(np.max(dist))
        k = min(len(dist), len(ref_t))
        s.drift_pct = 100.0 * dist[k - 1] / max(ref_t[k - 1] - ref_t[0], 1e-6)
        s.dist = float(ref_t[-1] - ref_t[0])
    return s


def _rmse(e, mask):
    if mask.sum() < 3:
        return float('nan')
    return float(np.sqrt(np.mean(e[mask] ** 2)))


def along_cross_error(est_p, ref_p, ref_cum):
    """Decompose the position error into along-track and cross-track parts."""
    if est_p is None or ref_p is None:
        return float('nan'), float('nan')
    n = min(len(est_p), len(ref_p))
    e = est_p[:n, :2] - ref_p[:n, :2]
    d = np.gradient(ref_p[:n, :2], axis=0)
    norm = np.linalg.norm(d, axis=1, keepdims=True)
    norm[norm < 1e-9] = 1.0
    t = d / norm
    nvec = np.column_stack([-t[:, 1], t[:, 0]])
    along = np.abs(np.sum(e * t, axis=1))
    cross = np.abs(np.sum(e * nvec, axis=1))
    return float(np.mean(along)), float(np.sqrt(np.mean(cross ** 2)))


def fmt(s: Score, name: str = '') -> str:
    def f(x, w=8, p=3):
        return ('%*.*f' % (w, p, x)) if x == x else ' ' * (w - 2) + 'n/a'
    return (f'{name:16}{f(s.v_rmse)}{f(s.v_mae)}{f(s.v_bias)}{f(s.v_max)}'
            f'{f(s.pos_rmse, 9, 2)}{f(s.drift_pct, 8, 3)}{f(s.slip_fraction, 7, 3)}')
