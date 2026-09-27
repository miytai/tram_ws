"""ROS-independent core of the backup-odometry pipeline.

All logical blocks of the case are chained here:

    InputPreprocessor -> TractionModel/LongitudinalDynamics -> SlipDetector
                      -> VelocityEstimator (EKF) -> RouteMap/InitialAligner

Keeping the pipeline free of ROS makes it unit-testable and lets the very same
code replay recorded bags offline (``tram_backup_odometry.replay``) which is
how the accuracy tables of the submission were produced.
"""
from __future__ import annotations

import math

from .estimator import VelocityEstimator
from .model import LongitudinalDynamics, TractionModel
from .params import Config
from .preprocess import InputPreprocessor
from .route import InitialAligner, RouteMap
from .slip import SlipDetector

# slip indicator bit mask
SLIP_RATIO, SLIP_INNOV, SLIP_BOGIE, SLIP_UNCOMMANDED, SLIP_DISPERSION, SLIP_ADHESION = (
    1, 2, 4, 8, 16, 32)


class BackupOdometryCore:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.pre = InputPreprocessor(cfg.inp)
        self.model = TractionModel(cfg.model.model_file, cfg.vehicle, cfg.model)
        self.dyn = LongitudinalDynamics(self.model, cfg.vehicle, cfg.model)
        self.slip = SlipDetector(self.model, cfg.vehicle, cfg.slip)
        self.est = VelocityEstimator(self.model, self.dyn, cfg.ekf, cfg.inp)
        self.route = None
        if cfg.route.use_map:
            try:
                self.route = RouteMap(cfg.route.map_file)
            except Exception:
                self.route = None
        self.aligner = None
        if cfg.route.init_mode != 'relative':
            self.aligner = InitialAligner(cfg.route, self.route) if self.route else None

        self.wheel_scale = 1.0
        self.lat0 = self.lon0 = self.alt0 = None
        self.init_time = None
        self._last_stamp = None
        self.align_status = 'waiting'
        self.align_reason = ''
        self._odom_at_window_start = 0.0
        self._scale_pairs = []           # (odometry arc, gnss arc) during window
        self._gnss_arc_total = 0.0
        self.n_updates = 0
        self._last_align_t = 0.0
        # GNSS source selection: the primary receiver is preferred, the secondary
        # one is used only when the primary is silent (the 30639 recordings have
        # no master receiver at all).
        self._fix_src = 0
        self._fix_t = {0: None, 1: None}
        self._fix_lat0 = {0: None, 1: None}
        self.n_fixes = {0: 0, 1: 0}

    # ------------------------------------------------------------------ ingest
    def on_front(self, value: float, stamp: float) -> None:
        self.pre.on_front(value, stamp)

    def on_rear(self, value: float, stamp: float) -> None:
        self.pre.on_rear(value, stamp)

    def on_notch(self, value: int, stamp: float) -> None:
        self.pre.on_notch(value, stamp)

    def on_fix(self, source: int, lat: float, lon: float, alt: float,
               stamp: float) -> None:
        """Ingest a geodetic fix.  ``source`` 0 = primary, 1 = secondary."""
        self._fix_t[source] = stamp
        self.n_fixes[source] += 1
        if self.aligner is None or not self.aligner.active:
            return
        # pick the source: primary unless it has been silent for a long time
        p, q = self._fix_t[0], self._fix_t[1]
        if p is None and q is not None:
            self._fix_src = 1
        elif p is not None and q is not None and p < q - self.cfg.route.gnss_timeout_s:
            self._fix_src = 1
        elif p is not None and q is not None and q < p - self.cfg.route.gnss_timeout_s:
            self._fix_src = 0
        elif p is not None and q is None:
            self._fix_src = 0
        if source != self._fix_src:
            return
        from . import geodesy
        if self._fix_lat0[source] is None:
            self._fix_lat0[source] = (lat, lon, alt)
        lat0, lon0, alt0 = self._fix_lat0[source]
        x, y, z = geodesy.enu(lat, lon, alt, lat0, lon0, alt0)
        self.aligner.add_fix(stamp, x, y, z)

    def on_gnss_velocity(self, source: int, stamp: float, speed: float) -> None:
        """Ingest a GNSS ground-speed sample (initial alignment window only)."""
        if self.aligner is None or not self.aligner.active:
            return
        if self._fix_t[source] is None or self._fix_src == source:
            self.aligner.add_velocity(stamp, max(speed, 0.0))

    # ------------------------------------------------------------------- step
    def step(self, stamp: float) -> bool:
        """Advance the pipeline to input time ``stamp``.  Returns True if new."""
        cfg = self.cfg
        pre = self.pre
        pre.update()
        if not pre.accepted or stamp <= 0.0:
            return False
        if pre.stamp > stamp:
            stamp = pre.stamp
        if self._last_stamp is not None and stamp <= self._last_stamp:
            return False
        dt = 0.0 if self._last_stamp is None else stamp - self._last_stamp
        if dt < 0.0 or dt > 2.0:
            dt = 0.0
        self._last_stamp = stamp
        if self.init_time is None:
            self.init_time = stamp
        self.n_updates += 1

        notch = pre.notch_pos
        v_front, v_rear = pre.v_front, pre.v_rear
        wheel_valid = pre.wheel_valid
        if wheel_valid and pre.bogie_disagreement() > cfg.slip.bogie_diff_threshold_mps:
            a_ref = self.est.a_model
            e_f = abs(v_front - self.est.velocity - a_ref * max(dt, 0.01))
            e_r = abs(v_rear - self.est.velocity - a_ref * max(dt, 0.01))
            if e_f <= e_r:
                v_rear = v_front
            else:
                v_front = v_rear

        # ---- estimator -----------------------------------------------------
        trust = self.slip.wheel_trust() if wheel_valid else 0.0
        self.est.step(notch, v_front, v_rear, stamp, trust, wheel_valid,
                      self.slip.active, self.wheel_scale)
        if not self.est.initialised and wheel_valid:
            self.est.reset(0.5 * (v_front + v_rear))

        a_model = self.est.a_model
        a_wheel = self.est.a_wheel if wheel_valid else a_model
        self.slip.update(notch, 0.5 * (v_front + v_rear), self.est.velocity,
                         a_model, a_wheel, v_front, v_rear, max(dt, 1e-3), wheel_valid)
        if wheel_valid and dt > 0:
            self.dyn.update_bias(a_wheel - a_model - self.est.b, dt)

        self._update_alignment(stamp, dt)
        return True

    # -------------------------------------------------------------- alignment
    def _update_alignment(self, stamp: float, dt: float) -> None:
        cfg = self.cfg.route
        al = self.aligner
        if al is None:
            return
        if al.active:
            al.update_odometry(self.est.s)
            if cfg.init_mode == 'absolute':
                if al.elapsed(stamp) > 0.5:
                    al.stop()
                    self.align_status = 'parameters'
                return
            if cfg.init_mode != 'auto':
                return
            # -- incremental refinement while GNSS is still inside the window --
            if (cfg.align_update_period_s > 0.0 and al.result.ok
                    and al.elapsed(stamp) - self._last_align_t > cfg.align_update_period_s):
                if al.try_match(final=False):
                    self._last_align_t = al.elapsed(stamp)
                    self.wheel_scale = al.ratio or self.wheel_scale
            elapsed = al.elapsed(stamp)
            if (al.trials > cfg.align_max_trials or elapsed > cfg.gnss_init_window_s) \
                    and elapsed - self._last_align_t >= cfg.match_retry_period_s:
                self._last_align_t = elapsed
                ok = al.try_match(min_motion_m=cfg.match_min_motion_m)
                if not ok and al.result.samples == 0:
                    # not enough motion for a confident match: retry with a
                    # relaxed threshold (the validation bags may only offer a
                    # few seconds of GNSS)
                    ok = al.try_match(min_motion_m=cfg.match_min_motion_fallback_m,
                                      residual_scale=cfg.match_residual_fallback_scale)
                if al.result.ok:
                    self.wheel_scale = al.ratio or 1.0
                    self.align_status = 'matched'
                else:
                    if al.result.ambiguous:
                        self.align_status = 'ambiguous'
                        self.align_reason = (
                            f'two parts of the track explain the {al.result.span_m:.0f} m '
                            f'window equally well (residual {al.result.residual:.1f} m at '
                            f's={al.result.s0:.0f} m vs {al.result.ambiguity:.2f}x worse '
                            f'elsewhere)')
                    else:
                        self.align_status = ('insufficient-motion'
                                             if al.result.samples == 0 else 'rejected')
                        self.align_reason = (
                            f'{al.result.span_m:.0f} m of motion, best residual '
                            f'{al.result.residual:.1f} m, rotation '
                            f'{math.degrees(al.result.yaw):+.1f} deg')
                    self.wheel_scale = 1.0
                if elapsed > cfg.gnss_init_window_s + 60.0:
                    al.stop()
            return

    # ---------------------------------------------------------------- outputs
    @property
    def velocity(self) -> float:
        return self.est.velocity

    def position(self):
        """(x, y, z, yaw, map_aligned) in the local metric frame [m, rad]."""
        cfg = self.cfg.route
        s = self.est.s
        al = self.aligner
        if al is not None and al.result.ok and self.route is not None:
            r = al.result
            self.route.select(r.branch)
            sm = r.arc0 + r.direction * r.ratio * (s - r.odo_zero)
            mx, my, mz, mh = self.route.point(sm)
            c, sn = math.cos(r.yaw), math.sin(r.yaw)
            x = c * mx - sn * my + r.tx
            y = sn * mx + c * my + r.ty
            z = (mz + r.tz) if cfg.publish_map_altitude else 0.0
            yaw = mh + r.yaw + (0.0 if r.direction > 0 else math.pi)
            return x, y, z, yaw, True
        yaw = cfg.init_yaw
        d = s * self.wheel_scale
        return (cfg.init_x + d * math.cos(yaw), cfg.init_y + d * math.sin(yaw),
                0.0, yaw, False)

    def diagnostics(self) -> dict:
        x, y, z, yaw, mapped = self.position()
        al = self.aligner
        return {
            'velocity_mps': self.est.velocity,
            'velocity_bias_mps': self.est.b,
            'velocity_sigma_mps': math.sqrt(self.est.variance),
            'model_accel_mps2': self.est.a_model,
            'wheel_accel_mps2': self.est.a_wheel,
            'innovation_mps2': self.est.innovation,
            'unmodelled_accel_mps2': self.dyn.bias,
            'notch': self.pre.notch_pos,
            'front_bogie_mps': self.pre.v_front,
            'rear_bogie_mps': self.pre.v_rear,
            'slip_ratio': self.slip.kappa,
            'slip_reasons': self.slip.reasons,
            'slip_severity': self.slip.severity,
            'adhesion_mu': self.slip.adhesion,
            'adhesion_required_mu': self.model.adhesion_utilisation(
                self.pre.notch_pos, self.est.velocity),
            'arc_length_m': self.est.s,
            'wheel_scale': self.wheel_scale,
            'x': x, 'y': y, 'z': z, 'yaw': yaw,
            'map_aligned': bool(mapped),
            'map_branch': self.route.name if self.route else '',
            'align_status': self.align_status,
            'align_residual_m': al.result.residual if al else float('nan'),
            'gnss_fixes_primary': self.n_fixes[0],
            'gnss_fixes_secondary': self.n_fixes[1],
            'inputs_rejected': self.pre.rejected,
            'input_gaps': self.pre.gaps,
        }
