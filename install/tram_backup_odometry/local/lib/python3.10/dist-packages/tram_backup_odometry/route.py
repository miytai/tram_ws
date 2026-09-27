"""Track (pathgraph) map and GNSS-based initial alignment.

The permitted online signals (wheel speed + driver command) carry **no
information about the direction of motion**: a tram on rails has no measurable
yaw rate without an IMU, a steering sensor or a map.  The case statement
therefore lists "координаты вдоль заданной траектории из карты pathgraph" as an
accepted representation of the position and explicitly allows GNSS to be used
for the *initial alignment*.

This module provides

  * :class:`RouteMap` - the pathgraph built offline from the development
    dataset (1 m polyline, tangent heading, smoothed altitude).  It is a
    *static infrastructure* map, exactly like the rail-network database a real
    tram odometry uses; it never enters the velocity estimation loop.
  * :class:`InitialAligner` - consumes GNSS **only** inside the configured
    initialisation window.  It (a) measures the ratio between the true
    travelled arc length and the odometry arc length (one-shot wheel-scale
    calibration), (b) matches the observed track segment onto the map in both
    directions with a closed-form rigid fit (yaw + translation), and stores the
    result.  After the window closes, the GNSS data is ignored completely.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass

import numpy as np

from .params import RouteCfg, deg

_EARTH_A = 6378137.0
_EARTH_E2 = 6.69437999014e-3


class _Branch:
    """Geometry of one mapped branch: a polyline plus its end tangents."""

    def __init__(self, b, max_extension):
        self.step = float(b.get('step_m', 1.0))
        self.s = [float(x) for x in b['s']]
        self.x = [float(x) for x in b['x']]
        self.y = [float(x) for x in b['y']]
        self.z = [float(x) for x in b.get('z', [0.0] * len(self.x))]
        self.length = float(b.get('length_m', self.s[-1] if self.s else 0.0))
        self.n = len(self.s)
        self.max_extension = max_extension
        self.name = b.get('name', 'branch')
        self._h0 = self.heading(0) if self.n > 6 else 0.0
        self._h1 = self.heading(self.n - 1) if self.n > 6 else 0.0
        self._cum_turn = self._build_cum_turn()

    def _build_cum_turn(self):
        """Cumulative absolute heading change - the observability measure.

        A window that contains no curve carries no information about *where*
        along the track the vehicle is: the rigid fit can translate it anywhere
        along a straight.  Matching is therefore only accepted on windows whose
        total turning is large enough to localise the vehicle.
        """
        c = [0.0] * self.n
        acc = 0.0
        h = None
        for i in range(1, self.n):
            hi = self.heading(i)
            if h is not None:
                d = hi - h
                while d > math.pi:
                    d -= 2.0 * math.pi
                while d < -math.pi:
                    d += 2.0 * math.pi
                acc += abs(d)
            h = hi
            c[i] = acc
        return c

    def turning(self, s0: float, s1: float) -> float:
        """Total heading change [rad] between two arc lengths."""
        def idx(s):
            s = min(max(s, 0.0), self.length)
            return min(max(int(s / self.step), 0), self.n - 1)
        return abs(self._cum_turn[idx(s1)] - self._cum_turn[idx(s0)])

    def heading(self, i: int) -> float:
        i0 = max(0, i - 3)
        i1 = min(self.n - 1, i + 3)
        return math.atan2(self.y[i1] - self.y[i0], self.x[i1] - self.x[i0])

    def point(self, s: float):
        if self.n == 0:
            return 0.0, 0.0, 0.0, 0.0
        if s < 0.0:
            d = min(-s, self.max_extension)
            return (self.x[0] - d * math.cos(self._h0),
                    self.y[0] - d * math.sin(self._h0), self.z[0], self._h0)
        if s > self.length:
            d = min(s - self.length, self.max_extension)
            return (self.x[-1] + d * math.cos(self._h1),
                    self.y[-1] + d * math.sin(self._h1), self.z[-1], self._h1)
        i = int(s / self.step)
        if i >= self.n - 1:
            i = self.n - 2
        f = (s - self.s[i]) / self.step
        x = self.x[i] * (1.0 - f) + self.x[i + 1] * f
        y = self.y[i] * (1.0 - f) + self.y[i + 1] * f
        z = self.z[i] * (1.0 - f) + self.z[i + 1] * f
        return x, y, z, self.heading(i)

    def sample(self, s_values, reverse: bool = False):
        out = []
        for s in s_values:
            if reverse:
                s = self.length - s
            p = self.point(s)
            out.append((p[0], p[1]))
        return out


class RouteMap:
    """One or more mapped branches of the track.

    Beyond the first/last node a branch is extrapolated along its end tangent,
    so that a run which is slightly longer than the mapped part of the circuit
    still produces a continuous position instead of a frozen one.  A map file
    may hold several branches (the two travel directions of the depot circuit,
    alternative routes through the turnouts); all of them are tried during the
    initial alignment and the best matching one is used.
    """

    def __init__(self, path: str, max_extension: float = 3000.0):
        with open(path, 'r') as fh:
            d = json.load(fh)
        raw = d['branches'] if 'branches' in d else [d]
        self.max_extension = max_extension
        self.branches = [_Branch(b, max_extension) for b in raw]
        if not self.branches:
            raise ValueError('route map contains no branch')
        self.branch = 0

    # ------------------------------------------------------------- selection
    def n_branches(self) -> int:
        return len(self.branches)

    def select(self, i: int) -> None:
        if 0 <= i < len(self.branches):
            self.branch = i

    @property
    def name(self) -> str:
        return self.branches[self.branch].name

    def point(self, s: float):
        return self.branches[self.branch].point(s)

    def heading(self, i: int) -> float:
        return self.branches[self.branch].heading(i)

    def sample(self, s_values, reverse: bool = False):
        return self.branches[self.branch].sample(s_values, reverse)

    def grade(self, s: float, half_window: float = 25.0) -> float:
        """Track gradient dz/ds around arc length ``s`` (dimensionless)."""
        b = self.branches[self.branch]
        a = self.point(max(0.0, s - half_window))
        c = self.point(min(b.length, s + half_window))
        d = math.hypot(c[0] - a[0], c[1] - a[1])
        return 0.0 if d < 1e-3 else (c[2] - a[2]) / d

    # read-only views of the active branch (used by tests and diagnostics)
    @property
    def step(self):
        return self.branches[self.branch].step

    @property
    def s(self):
        return self.branches[self.branch].s

    @property
    def x(self):
        return self.branches[self.branch].x

    @property
    def y(self):
        return self.branches[self.branch].y

    @property
    def z(self):
        return self.branches[self.branch].z

    @property
    def length(self):
        return self.branches[self.branch].length

    @property
    def n(self):
        return self.branches[self.branch].n


@dataclass
class MatchResult:
    ok: bool = False
    s0: float = 0.0             # matched start of the map arc (search parameter)
    arc0: float = 0.0           # map arc length corresponding to odo_zero
    direction: float = 1.0      # +1 forward along the map, -1 against it
    odo_zero: float = 0.0       # odometry arc length at the first matched fix
    ratio: float = 1.0          # wheel distance scale
    yaw: float = 0.0
    tx: float = 0.0
    ty: float = 0.0
    tz: float = 0.0
    scale: float = 1.0
    residual: float = float('inf')
    reverse: bool = False
    samples: int = 0
    inliers: int = 0
    weak: bool = False
    ambiguous: bool = False
    decisive: bool = False
    ambiguity: float = 0.0          # 0.0 = no competing hypothesis found
    alt_s0: float = 0.0
    branch: int = 0
    yaw_sigma: float = float('inf')
    span_m: float = 0.0


class InitialAligner:
    """Collects the initial GNSS segment and matches it onto the route map."""

    def __init__(self, cfg: RouteCfg, route: RouteMap):
        self.cfg = cfg
        self.route = route
        self.reset()

    # ------------------------------------------------------------------ state
    def reset(self) -> None:
        self.pts = []            # [x0, y0, z0, x1, ...] local ENU of this run
        self.odo = []            # odometry arc length at each fix
        self.result = MatchResult()
        self.done = False
        self.t0 = None
        self._last_t = None
        self.trials = 0
        self._gnss_len = 0.0
        self._ratio = None
        self._z_ref = None
        self.rejected = 0
        self.accepted = 0
        self._gnss_dv = 0.0
        self._v_prev = None
        self._v_prev_t = None
        self._dv_odo = []
        self._dv_odo0 = None

    @property
    def active(self) -> bool:
        return not self.done

    @property
    def ratio(self):
        """One-shot wheel-distance scale measured over the alignment window."""
        return self._ratio

    @property
    def gnss_length(self) -> float:
        return self._gnss_len

    def elapsed(self, t_now: float) -> float:
        if self.t0 is None or t_now is None:
            return 0.0
        return max(t_now - self.t0, 0.0)

    # ----------------------------------------------------------------- ingest
    def update_odometry(self, s: float) -> None:
        self._odo_now = s

    _odo_now = 0.0

    def add_velocity(self, t: float, speed: float) -> None:
        """Ingest a GNSS ground speed sample (used for the wheel-scale check).

        Integrating the GNSS *velocity* is far more robust than differentiating
        the GNSS *position*: the position channel of the dataset carries up to
        4 m of jitter which inflates the polyline length by a factor of two.
        """
        if self.done:
            return
        if self._v_prev is not None:
            dt = t - self._v_prev_t
            if 0.0 < dt < self.cfg.gnss_timeout_s:
                self._gnss_dv += 0.5 * (speed + self._v_prev) * dt
                self._dv_odo.append(self._odo_now)
                if self._dv_odo0 is None:
                    self._dv_odo0 = self._odo_now
        self._v_prev = speed
        self._v_prev_t = t

    def add_fix(self, t: float, x: float, y: float, z: float) -> None:
        if self.done:
            return
        if self.t0 is None:
            self.t0 = t
        if self._last_t is not None and t - self._last_t > self.cfg.gnss_timeout_s:
            # GNSS outage: start a new segment (the case guarantees only the
            # first seconds, so a gap invalidates the previous segment)
            self.trials += 1
            self._start_segment()
        if self.pts:
            px, py = self.pts[-3], self.pts[-2]
            d = math.hypot(x - px, y - py)
            dt = max(t - (self._last_t or t), 1e-3)
            if d > self.cfg.gnss_max_step_m or d / dt > self.cfg.gnss_max_speed_mps:
                self.rejected += 1          # GNSS outlier / glitch
                return
            self._gnss_len += d
        self.pts.extend((x, y, z))
        self.odo.append(self._odo_now)
        self._last_t = t
        self.accepted += 1

    def _start_segment(self) -> None:
        self.pts = []
        self.odo = []
        self._gnss_len = 0.0
        self._last_t = None

    _gnss_total = 0.0

    def stop(self) -> None:
        self.done = True

    # ------------------------------------------------------------------ match
    def try_match(self, min_motion_m: float = None, residual_scale: float = 1.0,
                  final: bool = True):
        """Match the accumulated GNSS segment onto the map.

        ``final=False`` performs an incremental refinement that keeps the
        aligner open (used while GNSS is still available inside the allowed
        initialisation window).  ``final=True`` closes the window.
        """
        cfg = self.cfg
        if (self.done and final) or self.route is None or self.route.n < 10:
            if final:
                self.done = True
            return False
        n = len(self.pts) // 3
        if n < cfg.match_min_samples:
            if final:
                self.done = True
            return False
        odo0 = self.odo[0]
        odo_span = self.odo[-1] - odo0
        need = cfg.match_min_motion_m if min_motion_m is None else min_motion_m
        if odo_span < need:
            return False                        # not enough motion yet

        # ---- one-shot wheel-scale calibration ------------------------------
        # ratio = (distance really travelled) / (odometry arc length).  The
        # GNSS velocity integral is used when available, the (noisy) GNSS path
        # length otherwise, and 1.0 when neither is trustworthy.
        ratio = 1.0
        if cfg.calibrate_wheel_scale:
            odo_dv = (self._dv_odo[-1] - self._dv_odo0) if len(self._dv_odo) > 3 else 0.0
            if odo_dv > cfg.match_min_motion_m * 0.5 and self._gnss_dv > 1.0:
                ratio = self._gnss_dv / odo_dv
            elif self._gnss_len > 1.0 and odo_span > cfg.match_min_motion_m * 0.5:
                ratio = self._gnss_len / odo_span
            ratio = min(max(ratio, cfg.wheel_scale_limits[0]), cfg.wheel_scale_limits[1])
        self._ratio = ratio

        rel = [ratio * (s - odo0) for s in self.odo]
        best = None
        nb = self.route.n_branches()
        for bi in range(nb):
            self.route.select(bi)
            cand = self._search(rel, cfg.match_grid_step_m, residual_scale)
            if cand is not None:
                fine = self._search(rel, cfg.match_refine_step_m, centre=cand,
                                    span=cfg.match_grid_step_m * 2.0,
                                    residual_scale=residual_scale)
                if fine is not None and fine.residual < cand.residual:
                    fine.ambiguity = cand.ambiguity
                    fine.alt_s0 = cand.alt_s0
                    cand = fine
                cand.branch = bi
                if best is None or cand.residual < best.residual:
                    best = cand
        if best is not None:
            self.route.select(best.branch)
        if best is None:
            if final:
                self.done = True
            return False

        zs = sorted(self.pts[2::3])
        z_ref = zs[len(zs) // 2]
        best.direction = -1.0 if best.reverse else 1.0
        best.odo_zero = odo0
        best.ratio = ratio
        best.arc0 = (self.route.length - best.s0) if best.reverse else best.s0
        best.tz = z_ref - self.route.point(best.arc0)[2]
        rot_ok = abs(deg(best.yaw)) <= cfg.match_max_abs_rot_deg
        unamb = best.ambiguity <= cfg.match_max_ambiguity
        best.ok = (rot_ok and unamb
                   and best.residual <= cfg.match_max_residual_m * residual_scale)
        if not best.ok and rot_ok and unamb and \
                best.residual <= cfg.match_weak_residual_m * residual_scale:
            # The route of this run does not follow the mapped branch exactly
            # (the depot has turnouts).  A weak match still beats straight-line
            # dead reckoning, so it is accepted and flagged.
            best.ok = True
            best.weak = True
        if not best.ok and not unamb:
            best.ambiguous = True
        best.samples = n
        best.span_m = odo_span
        if best.ok:
            self.result = best
        if final:
            self.done = True
        return best.ok

    def _obs_xy(self, stride: int = 1) -> np.ndarray:
        """Observed fixes as an (n, 2) array, optionally decimated."""
        a = np.asarray(self.pts, dtype=float).reshape(-1, 3)[:, :2]
        return a[::max(1, stride)]

    def _search(self, rel, step, residual_scale=1.0, centre=None, span=None):
        route = self.route
        L = route.length
        p0 = self.cfg.match_prior_s0_m
        ptol = self.cfg.match_prior_s0_tolerance_m
        if centre is not None:
            cands = [centre.s0 + d for d in _frange(-span, span, step)]
            dirs = [centre.reverse]
        else:
            reach = rel[-1]
            cands = _frange(0.0, L - reach, step)
            if self.cfg.match_use_prior:
                lo, hi = p0 - ptol, p0 + ptol
                cands = [c for c in cands if lo <= c <= hi] or [p0]
            dirs = [False, True]
        lim = self.cfg.match_max_residual_m * residual_scale * 6.0
        rot_lim = math.radians(self.cfg.match_max_abs_rot_deg)
        n_all = len(self.pts) // 3
        stride = max(1, n_all // self.cfg.match_max_fit_points)
        obs = self._obs_xy(stride)
        idx_used = np.arange(0, n_all, stride)
        rel_s = np.asarray(rel, dtype=float)[idx_used]
        cands_res = []                    # (residual, s0, reverse)
        prior_cands = []
        best = None
        ys = self.cfg.match_yaw_prior_sigma
        reach = float(rel[-1]) if len(rel) else 0.0
        min_turn = math.radians(self.cfg.match_min_turning_deg)
        for rev in dirs:
            for s0 in cands:
                if s0 < -30.0 or s0 > L + 30.0:
                    continue
                arc = rel_s + s0
                ref = np.asarray(route.sample(arc, reverse=rev), dtype=float)
                yaw, tx, ty, res, nin = _fit_with_outlier_rejection(
                    obs, ref, ys, self.cfg.match_outlier_rounds,
                    self.cfg.match_outlier_gate_m)
                if res > lim or abs(yaw) > rot_lim:
                    continue
                # observability: a window without a curve cannot localise the
                # vehicle, no matter how small its residual is
                turn = self.route.branches[self.route.branch].turning(s0, s0 + reach)
                if turn < min_turn:
                    continue
                cands_res.append((res, s0, rev))
                if best is None or res < best.residual:
                    best = MatchResult(s0=s0, yaw=yaw, tx=tx, ty=ty,
                                       residual=res, reverse=rev,
                                       inliers=n_all)
        if best is None:
            return None
        # A short window cannot localise the vehicle on a 5.5 km circuit that
        # contains kilometre-long straights.  All recordings of this depot start
        # at the beginning of the circuit, which is used as a documented prior;
        # a candidate outside the prior window is accepted only when it fits
        # *decisively* better (by at least `match_prior_margin`).
        if prior_cands and not self.cfg.match_use_prior:
            bp = min(prior_cands)[0]
            bg = min(cands_res)
            if bg[0] < bp / max(self.cfg.match_prior_margin, 1.0):
                best = MatchResult(s0=bg[1], yaw=best.yaw, tx=best.tx, ty=best.ty,
                                   residual=bg[0], reverse=bg[2], inliers=n_all,
                                   decisive=True)
        # Ambiguity: how much better is the winner than a *physically different*
        # hypothesis?  For the forward direction that is a candidate at least
        # `match_separation_m` away on the same branch; for the reverse
        # direction the arc offset is degenerate (any s0 gives the same fit), so
        # the competitor is the best forward hypothesis.
        sep = self.cfg.match_separation_m
        others = [r for r in cands_res
                  if (r[2] == best.reverse and abs(r[1] - best.s0) > sep)
                  or r[2] != best.reverse]
        if others:
            best.ambiguity = min(r[0] for r in others) / max(best.residual, 1e-6)
            best.alt_s0 = min(others)[1]
        return best


# --------------------------------------------------------------------- helpers
def _frange(a, b, step):
    out = []
    x = a
    n = int((b - a) / step) + 1 if step > 0 else 0
    for _ in range(max(n, 0)):
        out.append(x)
        x += step
    return out


def _fit_numpy(obs, ref, yaw_prior_sigma=0.0):
    """Closed-form rigid fit (yaw + translation) of ``obs`` onto ``ref``.

    Both are ``(n, 2)`` float arrays.  The map and the GNSS fixes live in the
    same local East/North metric frame, so the *expected* rotation is zero and
    only the origin differs: a Gaussian prior ``yaw ~ N(0, sigma^2)`` is applied
    to keep the rotation determined when the observed window is nearly straight.
    Returns ``(yaw, tx, ty, residual_rms)``; residual_rms is a *robust*
    (median based) sigma of the residuals.
    """
    n = obs.shape[0]
    if n < 4:
        return 0.0, 0.0, 0.0, float('inf')
    a = obs - obs.mean(axis=0)
    b = ref - ref.mean(axis=0)
    num = float(a[:, 0] @ b[:, 1] - a[:, 1] @ b[:, 0])
    den = float(a[:, 0] @ b[:, 0] + a[:, 1] @ b[:, 1])
    saa = float((a * a).sum())
    if saa < 1e-9:
        return 0.0, 0.0, 0.0, float('inf')
    yaw = math.atan2(num, den)
    if yaw_prior_sigma > 0.0:
        c0, s0 = math.cos(yaw), math.sin(yaw)
        gx = -s0 * a[:, 0] - c0 * a[:, 1]
        gy = c0 * a[:, 0] - s0 * a[:, 1]
        info = float((gx * gx + gy * gy).sum())
        post = 1.0 / info if info > 1e-12 else 1e6
        pri = yaw_prior_sigma * yaw_prior_sigma
        for _ in range(3):
            yaw *= pri / (pri + post)
            post = 1.0 / (1.0 / pri + 1.0 / post)
    c, s = math.cos(yaw), math.sin(yaw)
    t = ref.mean(axis=0) - np.array([c, s]) * float(a[:, 0] @ a[:, 0]) - \
        np.array([-s, c]) * float(a[:, 1] @ a[:, 1])
    # least squares translation for the (possibly shrunken) rotation
    R = np.array([[c, -s], [s, c]])
    t = (ref - obs @ R.T).mean(axis=0)
    pred = obs @ R.T + t
    err = np.linalg.norm(pred - ref, axis=1)
    return yaw, float(t[0]), float(t[1]), float(np.median(err) * 1.4826)


def _fit_with_outlier_rejection(obs, ref, yaw_prior_sigma=0.0, rounds=2,
                                gate_m=4.0):
    """Rigid fit with iterative trimming of gross GNSS outliers."""
    idx = np.arange(obs.shape[0])
    best = None
    for _ in range(max(1, rounds)):
        yaw, tx, ty, res = _fit_numpy(obs[idx], ref[idx], yaw_prior_sigma)
        R = np.array([[math.cos(yaw), -math.sin(yaw)],
                      [math.sin(yaw), math.cos(yaw)]])
        err = np.linalg.norm(obs[idx] @ R.T + np.array([tx, ty]) - ref[idx], axis=1)
        if best is None or res < best[3]:
            best = (yaw, tx, ty, res, len(idx))
        sig = max(res, 1e-6)
        keep = idx[err <= max(gate_m, 3.0 * sig)]
        if keep.size == idx.size or keep.size < 12:
            break
        idx = keep
    yaw, tx, ty, res, nin = best
    return yaw, tx, ty, res, nin
