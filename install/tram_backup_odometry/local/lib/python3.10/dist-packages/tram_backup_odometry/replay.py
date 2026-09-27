"""Offline replay of a rosbag2 through the estimator core.

Feeds the recorded messages into :class:`BackupOdometryCore` in timestamp order
(only the three permitted input topics plus, optionally, the GNSS used for the
initial alignment) and returns the published estimate.  This is exactly the
code path of the ROS node, therefore the numbers obtained here are the numbers
the node produces online.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .core import BackupOdometryCore
from .params import Config
from .evaluate import enu

FRONT = '/vehicle/front_bogie_velocity'
REAR = '/vehicle/rear_bogie_velocity'
NOTCH = '/vehicle/driver_position_cmd'
FIX = ('/sensing/gnss/master/fix', '/sensing/gnss/rover/fix')
VEL = ('/sensing/gnss/master/vel', '/sensing/gnss/rover/vel')


@dataclass
class ReplayResult:
    t: np.ndarray = field(default_factory=lambda: np.zeros(0))
    v: np.ndarray = field(default_factory=lambda: np.zeros(0))
    p: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))
    notch: np.ndarray = field(default_factory=lambda: np.zeros(0))
    kappa: np.ndarray = field(default_factory=lambda: np.zeros(0))
    diag: list = field(default_factory=list)
    align_status: str = ''
    align_residual: float = float('nan')
    align_ambiguity: float = float('nan')
    align_span_m: float = float('nan')
    diag_align: object = None
    wheel_scale: float = 1.0
    # reference
    rt: np.ndarray = field(default_factory=lambda: np.zeros(0))
    rv: np.ndarray = field(default_factory=lambda: np.zeros(0))
    rp: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))
    rvt: np.ndarray = field(default_factory=lambda: np.zeros(0))


def read_messages(path, topics=None):
    """Read a rosbag2 (sqlite3) into a list of (t, topic, kind, payload)."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    so = rosbag2_py.SequentialReader()
    so.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
            rosbag2_py.ConverterOptions('', ''))
    all_topics = so.get_all_topics_and_types()
    types = {x.name: x.type for x in all_topics}
    want = list(topics) if topics else [x.name for x in all_topics]
    so.set_filter(rosbag2_py.StorageFilter(topics=want))
    cls = {t: get_message(types[t]) for t in want if t in types}
    out = []
    while so.has_next():
        topic, data, stamp = so.read_next()
        if stamp < 10 ** 12:
            continue
        msg = deserialize_message(data, cls[topic])
        if topic in (FRONT, REAR):
            out.append((stamp * 1e-9, topic, 'v', msg.velocity))
        elif topic == NOTCH:
            out.append((stamp * 1e-9, topic, 'u', int(msg.position)))
        elif topic in FIX:
            out.append((stamp * 1e-9, topic, 'p',
                        (msg.latitude, msg.longitude, msg.altitude)))
        elif topic in VEL:
            out.append((stamp * 1e-9, topic, 'V',
                        (msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z)))
    del so
    out.sort(key=lambda x: x[0])
    return out


def replay(path, cfg: Config, with_reference: bool = True, progress: bool = False):
    msgs = read_messages(path)
    core = BackupOdometryCore(cfg)
    t0 = msgs[0][0] if msgs else 0.0

    lat0 = lon0 = alt0 = None
    ref_t, ref_v, ref_p = [], [], []
    out_t, out_v, out_p, out_n, out_k, diag = [], [], [], [], [], []
    src_of = {FIX[0]: 0, FIX[1]: 1, VEL[0]: 0, VEL[1]: 1}

    for i, (t, topic, kind, payload) in enumerate(msgs):
        if kind == 'v':
            if topic == FRONT:
                core.on_front(payload, t)
            else:
                core.on_rear(payload, t)
        elif kind == 'u':
            core.on_notch(payload, t)
        elif kind == 'p':
            if lat0 is None:
                lat0, lon0, alt0 = payload
            if with_reference:
                ref_t.append(t - t0)
                ref_p.append(enu(*payload, lat0, lon0, alt0))
            core.on_fix(src_of.get(topic, 0), *payload, t)
        elif kind == 'V':
            sp = math.hypot(payload[0], payload[1])
            core.on_gnss_velocity(src_of.get(topic, 0), t, sp)
            if with_reference:
                ref_v.append((t - t0, sp))

        if core.step(t):
            d = core.diagnostics()
            out_t.append(t - t0)
            out_v.append(d['velocity_mps'])
            out_p.append((d['x'], d['y'], d['z']))
            out_n.append(d['notch'])
            out_k.append(d['slip_ratio'])
            diag.append(d)
        if progress and i % 20000 == 0:
            print(f'  {i}/{len(msgs)} t={t - t0:8.1f}s', flush=True)

    res = ReplayResult()
    res.t = np.asarray(out_t)
    res.v = np.asarray(out_v)
    res.p = np.asarray(out_p).reshape(-1, 3)
    res.notch = np.asarray(out_n)
    res.kappa = np.asarray(out_k)
    res.diag = diag
    res.align_status = core.align_status
    res.align_residual = core.aligner.result.residual if core.aligner else float('nan')
    res.align_ambiguity = core.aligner.result.ambiguity if core.aligner else float('nan')
    res.align_span_m = core.aligner.result.span_m if core.aligner else float('nan')
    res.diag_align = core.aligner.result if core.aligner else None
    res.wheel_scale = core.wheel_scale

    if with_reference and ref_t:
        res.rt = np.asarray(ref_t)
        res.rp = np.asarray(ref_p).reshape(-1, 3)
        if ref_v:
            res.rvt = np.asarray([x[0] for x in ref_v])
            res.rv = np.asarray([x[1] for x in ref_v])
        else:
            res.rvt = res.rt
            res.rv = np.zeros_like(res.rt)
    return res


def reference_on_grid(res: ReplayResult, dt: float = 0.05):
    """Resample the GNSS reference (fixes + velocity) onto a common time base."""
    if len(res.rt) < 3 or len(res.t) == 0:
        return res.t, res.rv, res.rp
    keep = np.concatenate([[True], np.diff(res.rt) > 1e-6])
    rt, rp = res.rt[keep], res.rp[keep]
    grid = np.arange(res.t[0], res.t[-1] + 1e-9, dt)
    P = np.column_stack([np.interp(grid, rt, rp[:, k]) for k in range(3)])
    if len(res.rvt) > 2:
        kv = np.concatenate([[True], np.diff(res.rvt) > 1e-6])
        V = np.interp(grid, res.rvt[kv], res.rv[kv])
    else:
        V = np.zeros_like(grid)
    return grid, V, P
