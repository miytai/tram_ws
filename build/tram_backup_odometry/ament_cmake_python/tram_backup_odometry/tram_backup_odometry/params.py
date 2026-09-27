"""Configuration of the backup-odometry node.

All parameters can be set as ROS 2 parameters (see ``config/tram_params.yaml``).
Defaults are the values calibrated on the development dataset of the case.
"""
from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, field, fields

_HERE = os.path.dirname(os.path.abspath(__file__))
_SHARE = os.path.join(os.path.dirname(_HERE), 'share', 'tram_backup_odometry')


def default_path(*parts: str) -> str:
    """Resolve a file shipped in the package share/ directory.

    Tries, in order: the ament index (installed package), ``../config`` next to
    this module (running from the source tree) and finally the ament index
    default location.
    """
    candidates = []
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(os.path.join(get_package_share_directory(
            'tram_backup_odometry'), 'config'))
    except Exception:                                        # pragma: no cover
        pass
    candidates.append(os.path.abspath(os.path.join(_HERE, '..', 'config')))
    candidates.append(os.path.join(_SHARE, 'config'))
    for base in candidates:
        p = os.path.join(base, *parts)
        if os.path.isfile(p):
            return p
    return os.path.join(candidates[0], *parts)


@dataclass
class Topics:
    front_velocity: str = '/vehicle/front_bogie_velocity'
    rear_velocity: str = '/vehicle/rear_bogie_velocity'
    driver_command: str = '/vehicle/driver_position_cmd'
    gnss_fix_primary: str = '/sensing/gnss/master/fix'
    gnss_fix_secondary: str = '/sensing/gnss/rover/fix'
    gnss_vel_primary: str = '/sensing/gnss/master/vel'
    gnss_vel_secondary: str = '/sensing/gnss/rover/vel'
    out_velocity: str = '/result/velocity'
    out_position: str = '/result/position'
    out_diagnostics: str = '/result/diagnostics'
    out_slip_ratio: str = '/result/slip_ratio'
    out_slip_active: str = '/result/slipping'
    out_model_accel: str = '/result/model_accel'


@dataclass
class Vehicle:
    """Physical parameters of the tram."""

    mass_kg: float = 40000.0            # traction mass (loaded)
    wheel_radius_m: float = 0.35        # effective running wheel radius
    driveline_efficiency: float = 0.90
    adhesion_mu: float = 0.25           # wheel/rail adhesion limit (dry rail)
    gravity: float = 9.80665


@dataclass
class Input:
    """Reception and pre-processing."""

    # v[m/s] = raw * wheel_unit_scale.  The recorded wheel channel is in km/h
    # although the message comment states m/s; the factor below is
    # (1/3.6) * 1.000207, the second factor being the least-squares calibration
    # of the unit conversion against the GNSS velocity.
    wheel_unit_scale: float = 0.27783531
    max_wheel_speed_mps: float = 25.0
    notch_min: int = -15
    notch_max: int = 15
    notch_median_window: int = 5        # single-sample command glitches
    wheel_median_window: int = 3
    wheel_jump_limit_mps: float = 8.0   # per-sample plausibility limit
    use_header_stamp: bool = True
    input_timeout_s: float = 0.6        # no fresh wheel data for this long -> coast
    max_step_s: float = 0.5             # ignore absurd dt


@dataclass
class ModelCfg:
    model_file: str = field(default_factory=lambda: default_path('model.json'))
    # online adaptation of the unmodelled longitudinal term (track grade,
    # load change, wear): first order lag, disabled when tau <= 0
    bias_tau_s: float = 45.0
    bias_max_mps2: float = 0.45
    bias_gate_mps2: float = 0.45        # innovation above this is not fed back
    bias_learning_rate: float = 0.35
    use_adhesion_limit: bool = True


@dataclass
class SlipCfg:
    kappa_threshold: float = 0.04       # |kappa| above this -> slipping
    kappa_hard: float = 0.25            # full odometry rejection
    bogie_diff_threshold_mps: float = 0.12
    kappa_decay_tau_s: float = 1.2
    adhesion_gain: float = 1.0
    detect_accel_threshold_mps2: float = 0.45
    detect_decel_threshold_mps2: float = 0.55
    force_active_fraction: float = 0.10  # |F| > fraction * mu*m*g -> slip possible
    wheel_stuck_mps: float = 0.30        # below this the wheel is "not delivering"


@dataclass
class EkfCfg:
    r_wheel_sigma: float = 0.060        # wheel speed measurement noise [m/s]
    q_model_sigma: float = 0.16         # model acceleration uncertainty [m/s^2]
    q_bias_sigma: float = 0.020         # random walk of the velocity bias
    bias_leak_tau_s: float = 300.0      # prevents the bias state from wandering
    bias_max_mps: float = 3.0
    init_v_sigma: float = 0.5
    init_bias_sigma: float = 0.30
    wheel_slowdown_s: float = 2.0       # gain taper when wheel data is stale


@dataclass
class RouteCfg:
    map_file: str = field(default_factory=lambda: default_path('route_map.json'))
    use_map: bool = True
    init_mode: str = 'auto'             # auto | absolute | relative
    init_x: float = 0.0
    init_y: float = 0.0
    init_yaw: float = 0.0
    gnss_init_window_s: float = 60.0    # GNSS may be used only in this window
    gnss_timeout_s: float = 2.0
    gnss_max_step_m: float = 8.0        # outlier gate for the initial fixes
    gnss_max_speed_mps: float = 40.0
    match_min_motion_m: float = 60.0    # odometry motion needed to match
    match_min_motion_fallback_m: float = 20.0
    match_residual_fallback_scale: float = 3.0
    match_min_samples: int = 40
    match_grid_step_m: float = 10.0
    match_refine_step_m: float = 0.5
    match_max_fit_points: int = 200      # decimation of the fixes for the search
    match_retry_period_s: float = 2.0     # throttle of the (expensive) search
    match_max_residual_m: float = 8.0
    match_weak_residual_m: float = 12.0   # accepted but flagged
    match_separation_m: float = 200.0     # two candidates this far apart
    match_min_turning_deg: float = 0.0    # observability gate for the s0 search
    match_use_prior: bool = True          # restrict the search to the prior
    match_prior_s0_m: float = 0.0        # a-priori start arc of the branch
    match_prior_s0_tolerance_m: float = 0.0
    match_prior_margin: float = 2.0      # decisive-fit margin to leave the prior
    match_max_ambiguity: float = 1.35     # 2nd best residual / best
    match_max_abs_rot_deg: float = 60.0
    match_yaw_prior_sigma: float = 0.035   # 2 deg: map and GNSS share the ENU axes
    match_outlier_rounds: int = 2
    match_outlier_gate_m: float = 4.0
    match_scale_limits: tuple = (0.98, 1.02)
    align_update_period_s: float = 10.0  # refine while GNSS is still available
    align_max_trials: int = 3
    calibrate_wheel_scale: bool = True
    wheel_scale_limits: tuple = (0.985, 1.015)
    heading_smooth_m: float = 6.0
    publish_map_altitude: bool = True


@dataclass
class Output:
    publish_rate_hz: float = 50.0
    frame_id: str = 'map'
    publish_covariance: bool = True
    pos_sigma_base: float = 0.30
    pos_sigma_rate: float = 0.002
    vel_sigma: float = 0.08
    log_throttle_s: float = 5.0
    queue_depth: int = 5


@dataclass
class Config:
    topics: Topics = field(default_factory=Topics)
    vehicle: Vehicle = field(default_factory=Vehicle)
    inp: Input = field(default_factory=Input)
    model: ModelCfg = field(default_factory=ModelCfg)
    slip: SlipCfg = field(default_factory=SlipCfg)
    ekf: EkfCfg = field(default_factory=EkfCfg)
    route: RouteCfg = field(default_factory=RouteCfg)
    out: Output = field(default_factory=Output)

    # ------------------------------------------------------------------ ROS
    def declare(self, node) -> None:
        """Declare every leaf parameter on ``node`` with its default."""
        for f in fields(self):
            sub = getattr(self, f.name)
            prefix = f.name if f.name != 'inp' else 'input'
            for sf in fields(sub):
                default = getattr(sub, sf.name)
                name = f'{prefix}.{sf.name}'
                if isinstance(default, bool):
                    node.declare_parameter(name, default)
                elif isinstance(default, tuple):
                    node.declare_parameter(name, list(default))
                else:
                    node.declare_parameter(name, default)

    def load(self, node) -> None:
        """Override the defaults with the values found on ``node``."""
        for f in fields(self):
            sub = getattr(self, f.name)
            prefix = f.name if f.name != 'inp' else 'input'
            for sf in fields(sub):
                name = f'{prefix}.{sf.name}'
                try:
                    val = node.get_parameter(name).value
                except Exception:
                    continue
                if val is None or (isinstance(val, str) and not val.strip()):
                    continue          # empty in the yaml -> keep the default
                if isinstance(val, list):
                    val = tuple(val)
                setattr(sub, sf.name, val)
        self.resolve_paths()

    def resolve_paths(self) -> None:
        """Fill in the packaged model/map files when they were left empty."""
        if not self.model.model_file:
            self.model.model_file = default_path('model.json')
        if not self.route.map_file:
            self.route.map_file = default_path('route_map.json')

    @classmethod
    def from_dict(cls, d: dict) -> 'Config':
        cfg = cls()
        for key, sub in (('topics', 'topics'), ('vehicle', 'vehicle'), ('input', 'inp'),
                         ('model', 'model'), ('slip', 'slip'), ('ekf', 'ekf'),
                         ('route', 'route'), ('output', 'out')):
            if key in d and isinstance(d[key], dict):
                target = getattr(cfg, sub)
                for k, v in d[key].items():
                    if not hasattr(target, k):
                        continue
                    if v is None or (isinstance(v, str) and not v.strip()):
                        continue
                    if isinstance(v, list):
                        v = tuple(v)
                    setattr(target, k, v)
        cfg.resolve_paths()
        return cfg


def load_model_file(path: str) -> dict:
    with open(path, 'r') as fh:
        return json.load(fh)


def accel_to_force(a, mass_kg: float) -> float:
    return a * mass_kg


def force_to_accel(f, mass_kg: float) -> float:
    return f / mass_kg


def rad(d):
    return d * math.pi / 180.0


def deg(r):
    return r * 180.0 / math.pi
