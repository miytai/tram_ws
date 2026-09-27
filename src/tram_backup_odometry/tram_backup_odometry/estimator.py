"""State estimator: 2-state extended Kalman filter fusing wheel odometry and
the non-linear model, plus the along-path (arc-length) integration.

State vector

    x = [ v , b ]^T

    v   longitudinal speed of the vehicle body          [m/s]
    b   velocity bias: the part of the wheel indication that is *not* a real
        velocity of the body, i.e.  v_wheel = v + b.  It collects wheel/rail
        creep, the residual of the non-linear model, the unknown track gradient
        and a slowly varying scale error (wheel wear).

Process model

    v_{k+1} = v_k + a_nom(u, v_k) * dt + b * dt
    b_{k+1} = b_k                                (random walk)

Measurement

    z = v_wheel,   H = [1, 1],   R = sigma_R^2 * trust^(-2)

The measurement noise is inflated by the slip detector (``trust`` in [0,1]),
so during slipping the filter relies on the model, and the bias state absorbs
the difference instead of letting the velocity state jump.
"""
from __future__ import annotations

import math

from .model import LongitudinalDynamics, TractionModel
from .params import EkfCfg, Input


class VelocityEstimator:
    def __init__(self, model: TractionModel, dyn: LongitudinalDynamics,
                 cfg: EkfCfg, inp: Input):
        self.model = model
        self.dyn = dyn
        self.cfg = cfg
        self.inp = inp
        self.v = 0.0
        self.b = 0.0
        self.P = [[cfg.init_v_sigma ** 2, 0.0],
                  [0.0, cfg.init_bias_sigma ** 2]]
        self.s = 0.0                      # arc length [m]
        self.scale = 1.0                  # wheel distance scale (calibrated once)
        self.a_model = 0.0
        self.a_wheel = 0.0
        self.innovation = 0.0
        self.measurement = 0.0
        self.initialised = False
        self._prev_v_wheel = None
        self._prev_stamp = None

    # ------------------------------------------------------------------ helpers
    def reset(self, v0: float = 0.0) -> None:
        self.v = max(v0, 0.0)
        self.b = 0.0
        self.s = 0.0
        self.P = [[self.cfg.init_v_sigma ** 2, 0.0],
                  [0.0, self.cfg.init_bias_sigma ** 2]]
        self._prev_v_wheel = None
        self._prev_stamp = None
        self.initialised = True

    # -------------------------------------------------------------------- step
    def step(self, notch: int, v_front: float, v_rear: float, stamp: float,
             trust: float, wheel_valid: bool, slip_active: bool,
             wheel_scale: float = 1.0) -> None:
        cfg = self.cfg
        # ---- time base: the freshest input stamp, never the wall clock -------
        if stamp is None or stamp <= 0.0:
            return
        if self._prev_stamp is None:
            self._prev_stamp = stamp
            self._prev_v_wheel = 0.5 * (v_front + v_rear)
            if wheel_valid:
                self.v = 0.5 * (v_front + v_rear)
                self.initialised = True
            return
        dt = stamp - self._prev_stamp
        if dt <= 0.0:
            return
        if dt > 0.5:                      # long dropout: restart the filter
            self._prev_stamp = stamp
            self._prev_v_wheel = 0.5 * (v_front + v_rear)
            if wheel_valid:
                self.v = 0.5 * (v_front + v_rear)
            return
        dt = min(dt, 0.2)

        v_meas = 0.5 * (v_front + v_rear)

        # ---- wheel-side acceleration (used by the slip detector / diagnosis) --
        if self._prev_v_wheel is not None and dt > 1e-3 and wheel_valid:
            raw = (v_meas - self._prev_v_wheel) / dt
            self.a_wheel = 0.7 * self.a_wheel + 0.3 * raw
        elif not wheel_valid:
            self.a_wheel = self.a_model

        # ---- process --------------------------------------------------------
        # v is the speed of the *body*: it follows the model, the wheel offset
        # lives entirely in the bias state (a constant offset does not change
        # the wheel-derived acceleration, hence no b*dt term here).
        a = self.dyn.predict(notch, max(self.v, 0.0), dt)
        self.a_model = a
        leak = math.exp(-dt / cfg.bias_leak_tau_s) if cfg.bias_leak_tau_s > 0 else 1.0
        vk = self.v + a * dt
        bk = self.b * leak

        qa = (cfg.q_model_sigma * dt) ** 2
        qb = (cfg.q_bias_sigma * math.sqrt(dt)) ** 2
        Pvk = self.P[0][0] + qa + self.P[0][1] * (1.0 - leak)
        Pv_b = self.P[0][1] * leak
        Pbb = qb + (1.0 - leak * leak) * self.P[1][1]

        # ---- measurement ----------------------------------------------------
        if wheel_valid and trust > 0.0:
            sig = cfg.r_wheel_sigma / max(trust, 0.05)
            R = sig * sig
            z = v_meas
            innov = z - (vk + bk)
            S = Pvk + 2.0 * Pv_b + Pbb + R
            if S > 1e-12:
                Kv = (Pvk + Pv_b) / S
                Kb = (Pv_b + Pbb) / S
                vk += Kv * innov
                bk += Kb * innov
                lim = cfg.bias_max_mps
                if bk > lim:
                    bk = lim
                elif bk < -lim:
                    bk = -lim
                # covariance update: P = (I-KH) P (I-KH)^T + K R K^T,  H = [1, 1]
                n00 = 1.0 - Kv
                n01 = -Kv
                n10 = -Kb
                n11 = 1.0 - Kb
                rr = R
                p00 = n00 * Pvk + n01 * Pv_b + Kv * Kv * rr
                p01 = n00 * Pv_b + n01 * Pbb + Kv * Kb * rr
                p10 = n10 * Pvk + n11 * Pv_b + Kb * Kv * rr
                p11 = n10 * Pv_b + n11 * Pbb + Kb * Kb * rr
                self.P[0][0] = abs(p00) + 1e-9
                self.P[0][1] = p01
                self.P[1][0] = p10
                self.P[1][1] = abs(p11) + 1e-9
            self.innovation = innov
            self.measurement = z
        else:
            # model only: inflate the velocity covariance, freeze the bias
            self.P[0][0] += qa
            self.P[1][1] += qb
            self.innovation = 0.0

        # ---- state update / projection --------------------------------------
        if vk < 0.0:
            vk = 0.0
            if bk < 0.0:
                bk = 0.0
        v_bar = 0.5 * (self.v + vk)
        self.s += v_bar * dt * wheel_scale
        self.v = vk
        self.b = bk
        self._prev_stamp = stamp
        self._prev_v_wheel = v_meas
        if wheel_valid:
            self.initialised = True

    # ------------------------------------------------------------------ outputs
    @property
    def velocity(self) -> float:
        return max(self.v, 0.0)

    @property
    def variance(self) -> float:
        return max(self.P[0][0], 1e-6)
