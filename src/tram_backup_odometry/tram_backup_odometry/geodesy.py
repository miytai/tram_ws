"""Minimal WGS-84 <-> local ENU conversion (used for the GNSS initialisation).

The evaluation reference trajectory is a local metric frame, so during the
initialisation window the node converts the geodetic fixes into a local
East/North/Up frame whose origin is the first fix of the run.  This is only
used to *align* the odometry with the reference frame; the velocity estimate
never sees it.
"""
from __future__ import annotations

import math

A = 6378137.0
E2 = 6.69437999014e-3


def enu(lat, lon, alt, lat0, lon0, alt0):
    """Local East/North/Up of (lat, lon, alt) w.r.t. (lat0, lon0, alt0)."""
    sl = math.sin(math.radians(lat0))
    n = A / math.sqrt(1.0 - E2 * sl * sl)
    east = math.radians(lon - lon0) * n * math.cos(math.radians(lat0))
    north = math.radians(lat - lat0) * n
    return east, north, alt - alt0


def lla_to_enu(lat, lon, alt, lat0, lon0, alt0):
    return enu(lat, lon, alt, lat0, lon0, alt0)
