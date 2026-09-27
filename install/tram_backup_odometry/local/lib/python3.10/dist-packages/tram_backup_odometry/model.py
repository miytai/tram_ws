"""Non-linear traction model and longitudinal dynamics of the tram.

The longitudinal equation of motion of the whole vehicle is

    (m + J/R^2) dv/dt = F_t(u, v) - F_b(u, v) - F_res(v) - F_grade(s) - F_curve(v)

where

  u          driver controller position, -15..+15 (0 = neutral)
  v          longitudinal speed of the vehicle body, m/s
  F_t(u, v)  tractive force at the rail, N                     (traction model)
  F_b(u, v)  brake force at the rail, N                         (traction model)
  F_res(v)   running resistance = c0 + c1 v + c2 v^2, N        (rolling + aero)
  F_grade    m g sin(theta(s))                                 (route gradient)
  F_curve    m v^2 / R_min(s) on curves, approximated by a drag term

The traction characteristic is strongly non-linear and non-monotonic in u (see
``config/model.json``, calibrated on the development dataset): the plateau
around u = +9..+10, the shallow steps and the release step at u = -8.  It is
therefore stored as a table a_nom(u, v) (accelerations) that is bilinear
interpolated, i.e. a hybrid "table + analytic" model:

    a_nom(u, v) = table lookup, extended outside the calibrated range by
                   constant-power derating   a(v) *= v_max / v   for v > v_max
                   and a low-speed linear taper to zero at v = 0

The individual forces are recovered from the table so that the adhesion limit
can be evaluated and the slip detector can be given physically meaningful
inputs:

    F_t = m * max(a_nom, 0) ,  F_b = m * max(-a_nom, 0)
    F_t,lim = mu * m * g      (adhesion)
"""
from __future__ import annotations

import json
import math

from .params import ModelCfg, Vehicle


class TractionModel:
    """a_nom(u, v) lookup with constant-power extrapolation."""

    def __init__(self, path: str, vehicle: Vehicle, cfg: ModelCfg):
        with open(path, 'r') as fh:
            d = json.load(fh)
        self.cfg = cfg
        self.vehicle = vehicle
        self.n0 = int(d.get('notch_min', -15))
        self.n1 = int(d.get('notch_max', 15))
        self.nb = int(d['speed_bins'])
        self.dv = float(d.get('speed_bin_m', 1.0))
        self.v_max = self.dv * (self.nb - 1)
        self.table = [[float(x) for x in row] for row in d['accel_table']]
        self.res_c0 = float(d.get('resistance_c0_N', 0.0))
        self.res_c1 = float(d.get('resistance_c1_Nps', 0.0))
        self.res_c2 = float(d.get('resistance_c2_Nps2', 0.0))
        self.calibrated_mass = float(d.get('mass_kg', vehicle.mass_kg))
        # the table holds accelerations for the calibrated mass; store the forces
        self._F = [[a * self.calibrated_mass for a in row] for row in self.table]

    # ------------------------------------------------------------------ table
    def force(self, notch: int, v: float) -> float:
        """Nominal net longitudinal force at the rail [N] (signed)."""
        n = notch
        if n < self.n0:
            n = self.n0
        elif n > self.n1:
            n = self.n1
        vf = v / self.dv - 0.5
        j = int(math.floor(vf))
        if j < 0:
            j = 0
        elif j > self.nb - 2:
            j = self.nb - 2
        w = vf - j
        if w < 0.0:
            w = 0.0
        elif w > 1.0:
            w = 1.0
        i = n - self.n0
        f0 = self._F[i][j] * (1.0 - w) + self._F[i][j + 1] * w
        if v > self.v_max and f0 > 0.0:
            f0 *= self.v_max / v            # constant power beyond calibration
        elif v > self.v_max:
            f0 *= 1.0                        # braking is not power limited
        return f0

    def accel(self, notch: int, v: float, mass_kg: float) -> float:
        return self.force(notch, v) / mass_kg

    def resistance(self, v: float) -> float:
        return self.res_c0 + self.res_c1 * v + self.res_c2 * v * v

    def tractive_force(self, notch: int, v: float) -> float:
        return max(self.force(notch, v), 0.0)

    def brake_force(self, notch: int, v: float) -> float:
        return max(-self.force(notch, v), 0.0)

    def adhesion_limit(self) -> float:
        return self.vehicle.adhesion_mu * self.vehicle.mass_kg * self.vehicle.gravity

    def model_v_max(self) -> float:
        return self.v_max

    def adhesion_utilisation(self, notch: int, v: float) -> float:
        """required mu for the requested force, 0 when coasting."""
        f = abs(self.force(notch, v))
        if f <= 1.0:
            return 0.0
        return f / (self.vehicle.mass_kg * self.vehicle.gravity)

    def adhesion_limited_force(self, notch: int, v: float) -> float:
        """Requested force after the wheel/rail adhesion limit is applied."""
        f = self.force(notch, v)
        if not self.cfg.use_adhesion_limit:
            return f
        lim = self.adhesion_limit()
        if abs(f) > lim:
            return math.copysign(lim, f)
        return f


class LongitudinalDynamics:
    """dv/dt of the vehicle body from the driver command and the state.

    The unmodelled part of the longitudinal equation (route gradient, load
    change, wheel wear, sensor bias) is absorbed by an online first-order
    estimate ``bias`` [m/s^2] driven by the innovation of the filter.
    """

    def __init__(self, model: TractionModel, vehicle: Vehicle, cfg: ModelCfg):
        self.model = model
        self.vehicle = vehicle
        self.cfg = cfg
        self.bias = 0.0
        self.grade_rad = 0.0            # externally supplied (from the map)
        self.last_force = 0.0

    def predict(self, notch: int, v: float, dt: float) -> float:
        f = self.model.adhesion_limited_force(notch, v)
        if self.vehicle.driveline_efficiency > 0:
            f *= self.vehicle.driveline_efficiency
        self.last_force = f
        a = f / self.vehicle.mass_kg + self.bias - self.model.resistance(v) \
            / self.vehicle.mass_kg
        return a

    def step(self, notch: int, v: float, dt: float) -> float:
        """Euler-propagated speed (used when no wheel data is available)."""
        a = self.predict(notch, v, dt)
        return max(0.0, v + a * dt)

    def update_bias(self, innovation: float, dt: float) -> None:
        """Slow first-order tracking of the unmodelled longitudinal term.

        The steady state is ``bias == innovation``, i.e. the model prediction is
        corrected by exactly the part of the measured acceleration the
        calibrated characteristic cannot explain (route gradient, load change,
        wear, slow sensor drift).  The effective time constant is
        ``bias_tau_s / bias_learning_rate``.
        """
        tau = self.cfg.bias_tau_s
        if tau <= 0.0 or dt <= 0.0:
            return
        if abs(innovation) > self.cfg.bias_gate_mps2:
            return
        k = min(1.0, self.cfg.bias_learning_rate * dt / tau)
        self.bias += k * (innovation - self.bias)
        lim = self.cfg.bias_max_mps2
        if self.bias > lim:
            self.bias = lim
        elif self.bias < -lim:
            self.bias = -lim
