"""Slip (wheel-rail creep) detection and estimation.

Only wheel speeds and the driver command are available, so the slip ratio
cannot be measured directly.  It is therefore *estimated* from the difference
between the wheel-derived acceleration and the model prediction, which is the
standard model-based slip observer for a rail vehicle:

    kappa = (v_wheel - v_model) / max(v, v_eps)

with v_model the speed obtained by integrating the model acceleration.  The
estimate is validated by several independent indicators required by the case:

  * sudden change of wheel speed not confirmed by the model  (innovation)
  * disagreement between the two bogies                        (cross-bogie)
  * high tractive/brake force at a low speed derivative   (force/accel test)
  * deceleration at a notch that does not command braking
  * high dispersion of the wheel measurements                  (variance test)
  * adhesion utilisation above the physical limit
"""
from __future__ import annotations

import math

from .params import SlipCfg, Vehicle
from .model import TractionModel


class SlipDetector:
    def __init__(self, model: TractionModel, vehicle: Vehicle, cfg: SlipCfg):
        self.model = model
        self.vehicle = vehicle
        self.cfg = cfg
        self.kappa = 0.0            # filtered slip ratio estimate
        self.kappa_raw = 0.0
        self.active = False
        self.severity = 0.0         # 0..1
        self.adhesion = vehicle.adhesion_mu
        self.reasons = 0           # bit mask of active indicators
        self._hist = []            # (v_f, v_r) for the dispersion test
        self._prev_notch = 0

    # ------------------------------------------------------------------ update
    def update(self, notch: int, v_wheel: float, v_est: float, a_model: float,
               a_wheel: float, v_front: float, v_rear: float,
               dt: float, wheel_valid: bool) -> None:
        cfg = self.cfg
        veps = 0.5
        self.reasons = 0

        if wheel_valid and dt > 1e-3:
            denom = max(abs(v_est), veps)
            self.kappa_raw = (v_wheel - v_est) / denom
            if abs(self.kappa_raw) > 3.0:        # sensor fault, not slip
                self.kappa_raw = 0.0
        else:
            self.kappa_raw = 0.0

        # --- low-pass the slip estimate (slip is a transient phenomenon) ----
        tau = max(cfg.kappa_decay_tau_s, 1e-3)
        a = math.exp(-dt / tau)
        self.kappa = a * self.kappa + (1.0 - a) * self.kappa_raw

        # --- indicators ------------------------------------------------------
        innovation = a_wheel - a_model
        force = self.model.adhesion_limited_force(notch, max(v_wheel, 0.0))
        mu_need = self.model.adhesion_utilisation(notch, max(v_wheel, 0.0))
        f_active = cfg.force_active_fraction * self.model.adhesion_limit()

        if wheel_valid:
            if abs(self.kappa) > cfg.kappa_threshold:
                self.reasons |= 1
            if abs(innovation) > 0.35 and abs(force) > f_active:
                self.reasons |= 2
            if v_wheel < cfg.wheel_stuck_mps and a_model > 0.2:
                # traction commanded and the model says "go", but the wheels say
                # "standing still": the encoder is not delivering -> use the model
                self.reasons |= 64
            diff = abs(v_front - v_rear)
            if diff > cfg.bogie_diff_threshold_mps and max(v_front, v_rear) > 1.0:
                self.reasons |= 4
            if notch >= 0 and a_wheel < -cfg.detect_decel_threshold_mps2:
                self.reasons |= 8          # decelerating without a brake command
            if notch >= 0 and a_wheel > cfg.detect_accel_threshold_mps2:
                self.reasons |= 8          # accelerating without a traction command
            if self._hist:
                arr = self._hist
                if len(arr) > 4:
                    d = [arr[i][0] - arr[i - 1][0] for i in range(1, len(arr))]
                    if d and (max(d) - min(d)) > 0.5 and v_wheel > 3.0:
                        self.reasons |= 16   # dispersion / chatter
            self._hist.append((v_front, v_rear))
            if len(self._hist) > 10:
                self._hist.pop(0)
        if mu_need > self.vehicle.adhesion_mu * 1.02:
            self.reasons |= 32

        self.active = self.reasons != 0
        if self.reasons:
            sev = min(1.0, abs(self.kappa) / max(cfg.kappa_hard, 1e-3))
            self.severity = max(0.35 if self.reasons & ~1 else 0.0, sev)
        else:
            self.severity = 0.0

        # adhesion estimate: relaxes towards the measured utilisation
        if mu_need > 0.05:
            k = 0.02 * min(1.0, dt / max(self.cfg.kappa_decay_tau_s, 1e-3))
            self.adhesion += k * (min(mu_need, 0.9) - self.adhesion)
        self._prev_notch = notch

    # ------------------------------------------------------------------- query
    def wheel_trust(self) -> float:
        """Weight in [0, 1] telling how much the wheel odometry can be trusted."""
        if self.kappa_raw == 0.0 and not self.active:
            return 1.0
        k = abs(self.kappa)
        if k <= self.cfg.kappa_threshold:
            trust = 1.0
        elif k >= self.cfg.kappa_hard:
            trust = 0.05
        else:
            x = (k - self.cfg.kappa_threshold) / (self.cfg.kappa_hard - self.cfg.kappa_threshold)
            trust = 1.0 - 0.95 * x
        if self.reasons & 4:
            trust = min(trust, 0.5)
        if self.reasons & 2:
            trust = min(trust, 0.6)
        if self.reasons & 64:
            trust = min(trust, 0.15)
        return max(trust, 0.05)
