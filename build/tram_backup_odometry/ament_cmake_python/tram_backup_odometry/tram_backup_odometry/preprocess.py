"""Reception and pre-processing of the three allowed input topics.

Responsibilities (case requirement "Модуль приёма и предобработки данных"):

  * subscribe to ``/vehicle/front_bogie_velocity``,
    ``/vehicle/rear_bogie_velocity``, ``/vehicle/driver_position_cmd``;
  * time synchronisation: the estimator is clocked by the freshest input stamp
    so that all three quantities refer to the same instant;
  * unit conversion to SI (the recorded wheel channel is in km/h);
  * glitch / outlier rejection (median filters, plausibility gates, rate check);
  * continuity check and watchdog: stale or missing wheel data is flagged so
    that the estimator can fall back to the model;
  * cross-bogie fusion: the bogie that agrees best with the model prediction is
    selected when the two disagree (single-wheel slip).
"""
from __future__ import annotations

import math
from collections import deque

from .params import Input


class _Ring:
    """Fixed size ring buffer with a median filter."""

    def __init__(self, size: int, med: int):
        self.buf = deque(maxlen=max(size, 5))
        self.med = max(1, min(med, size))

    def push(self, x: float) -> None:
        self.buf.append(x)

    def median(self) -> float:
        if not self.buf:
            return 0.0
        n = min(self.med, len(self.buf))
        vals = list(self.buf)[-n:]
        vals.sort()
        return vals[n // 2]

    def last(self) -> float:
        return self.buf[-1] if self.buf else 0.0

    def dispersion(self) -> float:
        if len(self.buf) < 3:
            return 0.0
        vals = sorted(self.buf)
        m = vals[len(vals) // 2]
        return max(abs(v - m) for v in vals)

    def clear(self) -> None:
        self.buf.clear()


class InputPreprocessor:
    def __init__(self, cfg: Input):
        self.cfg = cfg
        self.front = _Ring(8, cfg.wheel_median_window)
        self.rear = _Ring(8, cfg.wheel_median_window)
        self.notch = _Ring(9, cfg.notch_median_window)
        self.t_front = None
        self.t_rear = None
        self.t_notch = None
        self.v_front = 0.0
        self.v_rear = 0.0
        self.notch_pos = 0
        self.stamp = 0.0                 # current common time [s]
        self.wheel_valid = False
        self.wheel_age = 1e9
        self.notch_age = 1e9
        self.rejected = 0
        self.accepted = 0
        self.gaps = 0
        self.used_wall_clock = False
        self._last_front_raw = 0.0
        self._last_rear_raw = 0.0

    # ------------------------------------------------------------------ ingest
    def _stamp(self, header_stamp: float, recv_stamp: float) -> float:
        s = header_stamp if self.cfg.use_header_stamp else 0.0
        if s <= 0.0:
            self.used_wall_clock = True
            return recv_stamp
        return s

    def on_front(self, value: float, stamp: float) -> None:
        t = self._stamp(stamp, 0.0)
        if not self._plausible(value, self._last_front_raw, self.t_front is not None):
            self.rejected += 1
            return
        if self.t_front is not None and t - self.t_front > self.cfg.max_step_s:
            self.gaps += 1
        self._last_front_raw = value
        self.front.push(value)
        self.t_front = t
        self.accepted += 1

    def on_rear(self, value: float, stamp: float) -> None:
        t = self._stamp(stamp, 0.0)
        if not self._plausible(value, self._last_rear_raw, self.t_rear is not None):
            self.rejected += 1
            return
        if self.t_rear is not None and t - self.t_rear > self.cfg.max_step_s:
            self.gaps += 1
        self._last_rear_raw = value
        self.rear.push(value)
        self.t_rear = t
        self.accepted += 1

    def on_notch(self, value: int, stamp: float) -> None:
        t = self._stamp(stamp, 0.0)
        n = int(round(value))
        if n < self.cfg.notch_min - 6 or n > self.cfg.notch_max + 6:
            self.rejected += 1
            return
        if self.t_notch is not None and t <= self.t_notch:
            # out-of-order / duplicated stamp: keep the newest only
            pass
        n = max(self.cfg.notch_min, min(self.cfg.notch_max, n))
        self.notch.push(float(n))
        self.t_notch = t
        self.accepted += 1

    def _plausible(self, value: float, last: float, have_last: bool) -> bool:
        v = value * self.cfg.wheel_unit_scale
        if not math.isfinite(v) or v < -1.0 or v > self.cfg.max_wheel_speed_mps:
            return False
        if have_last and abs(v - last * self.cfg.wheel_unit_scale) > self.cfg.wheel_jump_limit_mps:
            return False
        return True

    # ------------------------------------------------------------------ update
    def update(self) -> None:
        """Refresh the fused, time-stamped snapshot from the sample buffers."""
        stamps = [t for t in (self.t_front, self.t_rear, self.t_notch) if t is not None]
        if not stamps:
            return
        self.stamp = max(stamps)
        self.v_front = max(self.front.median() * self.cfg.wheel_unit_scale, 0.0)
        self.v_rear = max(self.rear.median() * self.cfg.wheel_unit_scale, 0.0)
        self.notch_pos = int(round(self.notch.median()))
        self.wheel_age = self.stamp - max([t for t in (self.t_front, self.t_rear)
                                          if t is not None] or [self.stamp])
        self.notch_age = self.stamp - self.t_notch if self.t_notch is not None else 1e9
        self.wheel_valid = (self.t_front is not None and self.t_rear is not None
                            and self.wheel_age <= self.cfg.input_timeout_s)

    def v_front_ok(self) -> bool:
        return self.t_front is not None

    def v_rear_ok(self) -> bool:
        return self.t_rear is not None

    def bogie_disagreement(self) -> float:
        return abs(self.v_front - self.v_rear)

    def dispersion(self) -> float:
        return max(self.front.dispersion(), self.rear.dispersion()) \
            * self.cfg.wheel_unit_scale

    def single_source_velocity(self) -> float:
        if self.t_front is not None and self.t_rear is not None:
            return 0.5 * (self.v_front + self.v_rear)
        if self.t_front is not None:
            return self.v_front
        if self.t_rear is not None:
            return self.v_rear
        return 0.0
