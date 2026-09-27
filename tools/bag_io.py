"""Offline rosbag2 reader helpers (calibration / evaluation only).

Not part of the runtime node - used to calibrate the model parameters
offline against the provided dataset and to score the estimator.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

WHEEL = '/vehicle/front_bogie_velocity'
WHEEL_R = '/vehicle/rear_bogie_velocity'
NOTCH = '/vehicle/driver_position_cmd'
GNSS_VEL_M = '/sensing/gnss/master/vel'
GNSS_VEL_R = '/sensing/gnss/rover/vel'
GNSS_FIX_M = '/sensing/gnss/master/fix'
GNSS_FIX_R = '/sensing/gnss/rover/fix'

WANTED = (WHEEL, WHEEL_R, NOTCH, GNSS_VEL_M, GNSS_VEL_R, GNSS_FIX_M, GNSS_FIX_R)


@dataclass
class Run:
    name: str
    t: dict           # topic -> (n,) float64 time [s] relative to t0
    v_front: np.ndarray
    v_rear: np.ndarray
    notch: np.ndarray
    t0: int
    gnss: dict        # 'vel_m' -> (n,3), 'pos' -> (n,3) ENU metres


def _fnv1a64(name: str) -> int:
    hsh = 0xcbf29ce484222325
    for c in name:
        hsh = ((hsh ^ ord(c)) * 0x100000001B3) & 0xFFFFFFFFFFFFFFFF
    return hsh


def read_run(path, topics=WANTED) -> Run:
    path = str(path)
    so = rosbag2_py.SequentialReader()
    so.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
            rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in so.get_all_topics_and_types()}
    keep = [t for t in topics if t in types]
    conns = [c for c in so.get_all_topics_and_types() if c.name in keep]
    so.set_filter(rosbag2_py.StorageFilter(topics=keep))

    cls = {n: get_message(types[n]) for n in keep}
    buf = {n: ([], []) for n in keep}
    t0 = None
    while so.has_next():
        topic, data, stamp = so.read_next()
        if stamp < 10 ** 12:      # guard against zero stamps
            continue
        if t0 is None:
            t0 = stamp
        msg = deserialize_message(data, cls[topic])
        b = buf[topic]
        b[0].append((stamp - t0) * 1e-9)
        if topic in (WHEEL, WHEEL_R):
            b[1].append(msg.velocity)
        elif topic == NOTCH:
            b[1].append(msg.position)
        elif topic in (GNSS_VEL_M, GNSS_VEL_R):
            b[1].append([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
        elif topic in (GNSS_FIX_M, GNSS_FIX_R):
            b[1].append([msg.latitude, msg.longitude, msg.altitude])
    del so

    def arr(topic):
        ts, vs = buf[topic]
        vs = np.asarray(vs, float)
        if vs.ndim == 2 and 1 in vs.shape:
            vs = vs.reshape(-1)
        return np.asarray(ts, float), vs

    tf, vf = arr(WHEEL)
    tr, vr = arr(WHEEL_R)
    tn, nn = arr(NOTCH)
    t0n = 1e18
    for ts, _ in buf.values():
        if len(ts):
            t0n = min(t0n, min(ts))
    g = {}
    for key, topic in (('vel_m', GNSS_VEL_M), ('vel_r', GNSS_VEL_R),
                       ('fix_m', GNSS_FIX_M), ('fix_r', GNSS_FIX_R)):
        ts, vs = buf[topic]
        if ts:
            g[key] = (np.asarray(ts, float), np.asarray(vs, float).reshape(len(ts), -1))
    g['t0_rel'] = t0n
    name = Path(path).name
    return Run(name, {'front': tf, 'rear': tr, 'notch': tn}, vf, vr, nn, t0, g)


def geo_to_enu(lat, lon, alt, lat0, lon0, alt0):
    a = 6378137.0
    e2 = 6.69437999014e-3
    sl = math.sin(math.radians(lat0))
    N = a / math.sqrt(1 - e2 * sl * sl)
    x = math.radians(lon - lon0) * N * math.cos(math.radians(lat0))
    y = math.radians(lat - lat0) * N
    return x, y, alt - alt0


def gnss_position(run: Run, prefer='fix_m'):
    """ENU (metres) trajectory + speed from the available GNSS reference."""
    src = prefer if prefer in run.gnss else ('fix_r' if 'fix_r' in run.gnss else None)
    if src is None:
        return None
    t, f = run.gnss[src]
    if len(t) == 0:
        return None
    t = t - run.gnss['t0_rel']
    lat0, lon0, alt0 = f[0, 0], f[0, 1], f[0, 2]
    enu = np.array([geo_to_enu(*f[i], lat0, lon0, alt0) for i in range(len(t))])
    vsrc = 'vel_m' if 'vel_m' in run.gnss and len(run.gnss['vel_m'][0]) else 'vel_r'
    v = None
    if vsrc in run.gnss:
        vt, vv = run.gnss[vsrc]
        v = np.column_stack([np.interp(t, vt - run.gnss['t0_rel'], vv[:, 0]),
                             np.interp(t, vt - run.gnss['t0_rel'], vv[:, 1])])
    return t, enu, v


def gnss_speed(run: Run):
    for key in ('vel_m', 'vel_r'):
        if key in run.gnss:
            t, v = run.gnss[key]
            if len(t):
                return t - run.gnss['t0_rel'], v
    return None, None


def idx(ds, bag):
    return ds / Path(bag).name


def find_bags(root):
    root = Path(root)
    return sorted([p for p in root.iterdir() if p.is_dir() and (p / 'metadata.yaml').exists()])
