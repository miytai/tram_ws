"""Unit tests of the backup-odometry pipeline (no ROS middleware required)."""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tram_backup_odometry.core import BackupOdometryCore
from tram_backup_odometry.params import Config, default_path
from tram_backup_odometry.model import TractionModel, LongitudinalDynamics
from tram_backup_odometry.preprocess import InputPreprocessor
from tram_backup_odometry.estimator import VelocityEstimator
from tram_backup_odometry.route import RouteMap, _fit_numpy
import numpy as np
from tram_backup_odometry import geodesy


def make_cfg():
    cfg = Config()
    cfg.model.model_file = default_path('model.json')
    cfg.route.map_file = default_path('route_map.json')
    return cfg


def cfg_default_scale():
    return Config().inp.wheel_unit_scale


# ----------------------------------------------------------------- preprocessor
def test_preprocessor_rejects_glitches_and_keeps_median():
    pre = InputPreprocessor(Config().inp)
    for i in range(10):
        pre.on_front(10.0, 1.0 + i * 0.1)
        pre.on_rear(10.0, 1.0 + i * 0.1)
    pre.on_notch(0, 1.0)
    for i in range(10, 20):
        pre.on_front(10.0, 2.0 + i * 0.1)
        pre.on_rear(10.0, 2.0 + i * 0.1)
    pre.on_notch(0, 2.0)
    pre.update()
    assert abs(pre.v_front - 10.0 * cfg_default_scale()) < 1e-3
    assert pre.rejected == 0


def test_preprocessor_rejects_single_sample_notch_glitch():
    cfg = Config()
    pre = InputPreprocessor(cfg.inp)
    for i in range(12):
        pre.on_notch(5, 1.0 + 0.1 * i)
    pre.on_notch(-8, 1.0 + 0.1 * 12)     # single-sample glitch
    for i in range(13, 24):
        pre.on_notch(5, 1.0 + 0.1 * i)
    pre.update()
    assert pre.notch_pos == 5


def test_preprocessor_rejects_implausible_speed():
    cfg = Config()
    pre = InputPreprocessor(cfg.inp)
    pre.on_front(10.0, 1.0)
    pre.on_front(1e6, 1.1)
    assert pre.rejected >= 1


def test_preprocessor_watchdog():
    cfg = Config()
    pre = InputPreprocessor(cfg.inp)
    pre.on_front(5.0, 1.0)
    pre.on_rear(5.0, 1.0)
    pre.on_notch(0, 1.0)
    pre.update()
    assert pre.wheel_valid
    pre.t_front -= 5.0
    pre.t_rear -= 5.0
    pre.update()
    assert not pre.wheel_valid


# ----------------------------------------------------------------------- model
def test_model_table_is_monotone_in_notch_for_traction():
    cfg = make_cfg()
    m = TractionModel(cfg.model.model_file, cfg.vehicle, cfg.model)
    prev = -1e9
    for u in range(1, 16):
        a = m.accel(u, 3.0, cfg.vehicle.mass_kg)
        assert a > 0.0
        prev = a
    assert m.accel(10, 3.0, cfg.vehicle.mass_kg) > m.accel(1, 3.0, cfg.vehicle.mass_kg)
    # braking
    for u in range(-1, -16, -1):
        assert m.accel(u, 6.0, cfg.vehicle.mass_kg) < 0.0
    # neutral notch never accelerates
    for v in (0.0, 2.0, 6.0, 12.0):
        assert m.accel(0, v, cfg.vehicle.mass_kg) <= 1e-6


def test_model_speed_derating():
    cfg = make_cfg()
    m = TractionModel(cfg.model.model_file, cfg.vehicle, cfg.model)
    a_low = m.accel(10, 2.0, cfg.vehicle.mass_kg)
    a_high = m.accel(10, 13.0, cfg.vehicle.mass_kg)
    assert a_high < a_low
    # constant-power extrapolation beyond the calibrated range
    f1 = m.force(10, 15.0)
    f2 = m.force(10, 30.0)
    assert f2 < f1
    assert abs(f1 * 15.0 - f2 * 30.0) / max(abs(f1 * 15.0), 1.0) < 0.05


def test_model_adhesion_limit():
    cfg = make_cfg()
    cfg.vehicle.adhesion_mu = 0.05
    cfg.model.use_adhesion_limit = True
    m = TractionModel(cfg.model.model_file, cfg.vehicle, cfg.model)
    f = m.adhesion_limited_force(15, 2.0)
    assert abs(f) <= m.adhesion_limit() + 1e-6
    assert m.adhesion_utilisation(15, 2.0) > cfg.vehicle.adhesion_mu


def test_dynamics_bias_adaptation():
    cfg = make_cfg()
    m = TractionModel(cfg.model.model_file, cfg.vehicle, cfg.model)
    d = LongitudinalDynamics(m, cfg.vehicle, cfg.model)
    for _ in range(20000):                 # 1000 s of simulated time
        d.update_bias(-0.1, 0.05)
    assert abs(d.bias + 0.1) < 0.02        # converges to the innovation
    for _ in range(2000):                  # innovation above the gate: ignored
        d.update_bias(1.0, 0.05)
    assert d.bias <= cfg.model.bias_max_mps2 + 1e-9
    for _ in range(40000):
        d.update_bias(-1.0, 0.05)
    assert d.bias >= -cfg.model.bias_max_mps2 - 1e-9


# ------------------------------------------------------------------- estimator
def test_estimator_tracks_constant_speed():
    cfg = make_cfg()
    cfg.vehicle.adhesion_mu = 0.25
    m = TractionModel(cfg.model.model_file, cfg.vehicle, cfg.model)
    d = LongitudinalDynamics(m, cfg.vehicle, cfg.model)
    e = VelocityEstimator(m, d, cfg.ekf, cfg.inp)
    v = 8.0
    t = 1000.0
    for i in range(2000):
        t += 0.05
        e.step(0, v, v, t, 1.0, True, False, 1.0)
    assert abs(e.velocity - v) < 0.05
    assert abs(e.s - v * (2000 * 0.05)) < 2.0


def test_estimator_keeps_truth_during_slip():
    """A 1.5 m/s wheel over-read (traction slip) must not corrupt the estimate."""
    cfg = make_cfg()
    m = TractionModel(cfg.model.model_file, cfg.vehicle, cfg.model)
    d = LongitudinalDynamics(m, cfg.vehicle, cfg.model)
    e = VelocityEstimator(m, d, cfg.ekf, cfg.inp)
    v_true = 5.0
    t = 1000.0
    for i in range(2000):
        t += 0.05
        slip = 200 <= i < 240              # 2 s of wheel over-read
        e.step(0, v_true + (1.5 if slip else 0.0),
               v_true + (1.5 if slip else 0.0), t, 0.15 if slip else 1.0,
               True, slip, 1.0)
    assert abs(e.velocity - v_true) < 0.35
    assert abs(e.b) < 0.5
    # the arc length must not have absorbed the slip
    assert abs(e.s - v_true * 2000 * 0.05) < 5.0


def test_estimator_survives_sensor_dropout():
    cfg = make_cfg()
    cfg.route.init_mode = 'relative'
    cfg.route.use_map = False
    core = BackupOdometryCore(cfg)
    t = 1000.0
    for i in range(400):
        t += 0.05
        core.on_front(0.0, t)
        core.on_rear(0.0, t)
        core.on_notch(4, t)
        core.step(t)
    assert core.velocity > 0.2             # model takes over: wheel not delivering
    v_before = core.velocity
    for i in range(400):                   # 20 s without any wheel message
        t += 0.05
        core.on_notch(4, t)
        core.step(t)
    assert core.velocity >= 0.0
    assert math.isfinite(core.velocity)
    assert abs(core.velocity - v_before) < 8.0


# ----------------------------------------------------------------------- route
def test_route_map_lookup():
    cfg = make_cfg()
    r = RouteMap(cfg.route.map_file)
    assert r.n > 100
    x0, y0, z0, h0 = r.point(0.0)
    x1, y1, _, _ = r.point(r.step)
    d = math.hypot(x1 - x0, y1 - y0)
    assert abs(d - r.step) < 0.05
    assert abs(h0 - math.atan2(y1 - y0, x1 - x0)) < 0.2
    # beyond the ends the polyline is extrapolated along the end tangent
    xa, ya, _, ha = r.point(-50.0)
    assert abs(math.hypot(xa - x0, ya - y0) - 50.0) < 0.5
    assert abs(ha - h0) < 1e-6
    xb, yb, _, hb = r.point(r.length + 50.0)
    assert abs(math.hypot(xb - r.x[-1], yb - r.y[-1]) - 50.0) < 0.5
    assert abs(hb - r.heading(r.n - 1)) < 1e-6


def test_rigid_fit_recovers_transform():
    import random
    random.seed(7)
    src = np.array([[random.uniform(-500, 500), random.uniform(-500, 500)]
                    for _ in range(60)])
    yaw, tx, ty = 0.37, 12.0, -4.0
    c, s = math.cos(yaw), math.sin(yaw)
    R = np.array([[c, -s], [s, c]])
    dst = src @ R.T + np.array([tx, ty])
    y2, x2, ty2, res = _fit_numpy(src, dst, 0.0)
    assert abs(y2 - yaw) < 1e-9
    assert abs(x2 - tx) < 1e-6
    assert abs(ty2 - ty) < 1e-6
    assert res < 1e-6


def test_yaw_prior_shrinks_unconstrained_rotation():
    """A short, noisy segment must not produce a wild rotation."""
    rng = np.random.default_rng(3)
    src = np.column_stack([np.linspace(0, 6, 20), np.zeros(20)])
    # 2 m of GNSS jitter on a 6 m long window: the rotation is unobservable
    dst = src @ np.array([[1.0, 0.0], [0.0, 1.0]]).T + np.array([3.0, -2.0]) \
        + rng.normal(0.0, 2.0, src.shape)
    free = _fit_numpy(src, dst, 0.0)
    prior = _fit_numpy(src, dst, 0.035)
    assert abs(prior[0]) < abs(free[0]) + 1e-9
    assert abs(prior[0]) < 0.2
    # a genuinely curved, long segment must still be recovered
    tt = np.linspace(0, 400, 200)
    curve = np.column_stack([tt, 30.0 * np.sin(tt / 90.0)])
    Rc = np.array([[math.cos(0.10), -math.sin(0.10)], [math.sin(0.10), math.cos(0.10)]])
    dstc = curve @ Rc.T + np.array([7.0, -2.0])
    yc, _, _, resc = _fit_numpy(curve, dstc, 0.035)
    assert abs(yc - 0.10) < 0.01
    assert resc < 0.5


# ----------------------------------------------------------------- geodesy
def test_enu_roundtrip():
    lat0, lon0, alt0 = 55.75, 37.61, 100.0
    x, y, z = geodesy.enu(lat0, lon0, alt0, lat0, lon0, alt0)
    assert abs(x) < 1e-6 and abs(y) < 1e-6 and abs(z) < 1e-6
    x, y, z = geodesy.enu(lat0 + 1e-3, lon0, alt0, lat0, lon0, alt0)
    assert 100 < y < 120                  # ~111 m per 0.001 deg of latitude
    assert abs(x) < 1.0


# ----------------------------------------------------------------- core / e2e
def test_core_end_to_end_constant_speed():
    cfg = make_cfg()
    cfg.route.init_mode = 'relative'
    cfg.route.use_map = False
    core = BackupOdometryCore(cfg)
    t = 100.0
    v_raw = 30.0                          # km/h
    for i in range(4000):
        t += 0.05
        core.on_front(v_raw, t)
        core.on_rear(v_raw, t)
        core.on_notch(0, t)
        assert core.step(t)
    assert abs(core.velocity - v_raw / 3.6) < 0.05
    x, y, z, yaw, mapped = core.position()
    assert not mapped
    assert abs(x - core.est.s) < 1e-6
    d = core.diagnostics()
    for k in ('velocity_mps', 'slip_ratio', 'arc_length_m', 'align_status'):
        assert k in d


def test_core_rejects_glitches():
    """A periodic single-sample command glitch must not move the estimate."""
    cfg = make_cfg()
    cfg.route.init_mode = 'relative'
    cfg.route.use_map = False
    core = BackupOdometryCore(cfg)
    t = 100.0
    for i in range(2000):
        t += 0.05
        u = -8 if i % 37 == 0 else 0        # single-sample command glitch
        core.on_front(20.0, t)
        core.on_rear(20.0, t)
        core.on_notch(u, t)
        core.step(t)
    assert core.pre.rejected == 0
    assert core.pre.notch_pos == 0
    assert abs(core.velocity - 20.0 / 3.6) < 0.1
    assert abs(core.slip.kappa) < 0.04      # no phantom slip
