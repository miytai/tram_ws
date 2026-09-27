# Резервная одометрия трамвая по модели — ROS 2 Humble

Model-based backup odometry for an autonomous tram: the longitudinal velocity
and the position are estimated **online** from the driver controller position and
the two bogie wheel speeds only — no GNSS and no IMU inside the estimation loop.

```
inputs                                  outputs
/vehicle/front_bogie_velocity   ┐        /result/velocity     tram_vehicle_msgs/msg/VelocitySensor  [m/s]
/vehicle/rear_bogie_velocity    ├─►      /result/position     nav_msgs/msg/Odometry                 [m]
/vehicle/driver_position_cmd    ┘        /result/diagnostics  diagnostic_msgs/msg/DiagnosticArray
/sensing/gnss/*/fix  (initial    ──►     /result/slip_ratio    std_msgs/msg/Float32
 alignment window only)                 /result/slipping      std_msgs/msg/Bool
/result/model_accel  std_msgs/msg/Float32
```

Full Russian documentation of the model, the parameters, the measurements and
the limitations: **[docs/MODEL.md](docs/MODEL.md)**.

---

## Build

```bash
cd ~/tram_ws
colcon build --packages-select tram_vehicle_msgs tram_backup_odometry
source install/setup.bash
```

`tram_vehicle_msgs` is the custom message package taken from the case dataset
(the only change is the addition of the `<maintainer>` tag, which
`catkin_pkg` requires).  No internet access is needed.

Dependencies: ROS 2 Humble (rclpy, nav_msgs, sensor_msgs, geometry_msgs,
diagnostic_msgs, rosbag2_py) and `python3-numpy`.

## Run

```bash
# 1) start the estimator
ros2 launch tram_backup_odometry backup_odometry.launch.py

# 2) replay a recording
ros2 bag play ~/Downloads/dataset/data/30618_0e41eac3

# both at once (what the jury should do)
ros2 launch tram_backup_odometry eval_bag.launch.py \
    bag:=$HOME/Downloads/dataset/data/30618_0e41eac3

# emulate the validation conditions: GNSS only in the first 8 s
ros2 launch tram_backup_odometry eval_bag.launch.py \
    bag:=$HOME/Downloads/dataset/data/30618_0e41eac3 \
    gnss_init_window_s:=8.0
```

Useful launch arguments: `params_file`, `map_file`, `model_file`,
`gnss_init_window_s`, `init_mode` (`auto` | `absolute` | `relative`),
`use_map`, `publish_rate_hz`, `log_level`.

## Inspect

```bash
ros2 topic hz /result/velocity
ros2 topic echo /result/velocity --once
ros2 topic echo /result/position --once
ros2 topic echo /result/diagnostics --once     # full state vector + slip flags
ros2 topic echo /result/slipping
```

## Accuracy report

`evaluate_bag` replays recordings through **the same code** the node uses
(`core.BackupOdometryCore`, ROS-free) and scores the result against the GNSS
telemetry:

```bash
# one run
ros2 run tram_backup_odometry evaluate_bag ~/Downloads/dataset/data/30618_0e41eac3

# the whole development dataset
ros2 run tram_backup_odometry evaluate_bag --all ~/Downloads/dataset/data \
    --csv scores.csv

# pure relative dead reckoning (no map, no GNSS) - ablation
ros2 run tram_backup_odometry evaluate_bag --all ~/Downloads/dataset/data --relative

# test conditions: GNSS available in the first 8 s only
ros2 run tram_backup_odometry evaluate_bag --all ~/Downloads/dataset/data --window 8
```

Metrics printed per run: velocity RMSE / MAE / bias / max (and RMSE split into
accelerating / braking / coasting), position RMSE / max, mean along-track and
cross-track error, accumulated drift in % of the travelled distance, fraction of
time with slipping detected, and the alignment status.

## Accuracy report (measured on the 87 development runs that carry a GNSS reference)

| metric | median | p90 | requirement |
|---|---|---|---|
| velocity RMSE | **0.093 m/s** (0.33 km/h) | 0.401 m/s | — |
| velocity bias | **−0.011 m/s** | −0.002 m/s | no systematic bias |
| position RMSE | **107 m** | 5672 m | — |
| mean along-track error | **59 m** | 3721 m | — |
| mean cross-track error | **84 m** | 2950 m | — |
| accumulated drift | **0.17 %** of the distance | 4.84 % | — |
| time with slipping detected | 1.3 % | 5.0 % | — |
| `/result/velocity` rate | **49.7 Hz** | — | ≥ 10 Hz |
| input → publication latency | **8.9 ms** (p95 19 ms) | 30 ms | ≤ 100 ms |
| CPU / RAM | 10.9 % of one core, 61 MB | — | ≤ 2 cores, ≤ 0.5 GB |

With an 8 s GNSS window (the guaranteed validation condition) the position RMSE
median rises to 176 m and the drift to 0.26 %; the velocity is unchanged.
Ablation without the route map (pure relative dead reckoning, still legal under
the rules) gives 793 m position RMSE and 2.31 % drift — the map is worth a
factor of 7 in position and 13 in drift; the model and the unit conversion give
the remaining two orders of magnitude in velocity.

Full methodology, regime breakdown, ablations and figures:
[docs/RESULTS.md](docs/RESULTS.md).  Per-run CSV: [docs/data/](docs/data/).
Model and assumptions: [docs/MODEL.md](docs/MODEL.md); offline calibration
procedure: [docs/CALIBRATION.md](docs/CALIBRATION.md).

## Tests

```bash
colcon test --packages-select tram_backup_odometry && colcon test-result
```

17 unit tests: unit conversion and glitch rejection, watchdog, non-monotonic
traction table and constant-power extrapolation, adhesion limit, convergence of
the grade bias, slip immunity of the estimator, survival of a sensor dropout,
map lookup / extrapolation, rigid-fit and yaw-prior behaviour, and end-to-end
runs of the core.

## Package layout

| path | content |
|---|---|
| `tram_backup_odometry/preprocess.py` | input reception, unit conversion, glitch/outlier rejection, time synchronisation, watchdog |
| `tram_backup_odometry/model.py` | `TractionModel` (non-linear `a_nom(u,v)` characteristic, constant-power extrapolation, adhesion limit) and `LongitudinalDynamics` (resistance, adaptive unmodelled-acceleration bias) |
| `tram_backup_odometry/slip.py` | slip ratio estimation, six slip indicators, adhesion estimate, odometry trust |
| `tram_backup_odometry/estimator.py` | 2-state EKF (velocity + velocity bias) and the arc-length integration |
| `tram_backup_odometry/route.py` | `RouteMap` (pathgraph, several branches, tangent extrapolation) and `InitialAligner` (GNSS initial alignment + one-shot wheel-scale calibration) |
| `tram_backup_odometry/core.py` | ROS-free pipeline wiring all blocks together |
| `tram_backup_odometry/node.py` | the ROS 2 node |
| `tram_backup_odometry/replay.py` | offline replay of rosbag2 through the core |
| `tram_backup_odometry/evaluate.py` | metric computation |
| `tram_backup_odometry/geodesy.py` | WGS-84 → local ENU (initial alignment only) |
| `config/model.json` | calibrated traction characteristic and resistance |
| `config/route_map.json` | track map, 1 m polyline, both travel directions |
| `config/tram_params.yaml` | all ROS parameters with defaults |
| `launch/*.launch.py` | node launch and demo-with-bag launch |
| `scripts/evaluate_bag` | offline scorer / report generator |
| `docs/MODEL.md` | model, algorithms, parameters, measurements, limitations |
| `docs/RESULTS.md` | accuracy and performance tables |

## Key design decisions

* **The wheel channel is in km/h**, not m/s as the message comment claims.  The
  conversion is a parameter (`input.wheel_unit_scale = 0.27783531 = 1/3.6 ×
  1.000207`, the second factor calibrated against the GNSS velocity over all
  122 recordings).
* **The estimator is clocked by the input stamps**, never by the system clock, so
  `header.stamp` of both outputs is the bag time of the corresponding input.
* **GNSS is used only inside the initialisation window** and is then ignored;
  the route map is a static infrastructure object (the same idea as a rail
  network database) and never enters the velocity loop.  The starting arc along
  the map comes from a documented prior of the test scenario (all depot
  recordings begin at the start of the circuit) while the rigid transform and
  the wheel-distance scale are estimated from data — a short GNSS window cannot
  localise the vehicle on a 5.5 km circuit that contains kilometre-long straights
  (see `docs/MODEL.md` §4.3).
* **The wheel is the primary velocity source, the model is the arbiter**: during
  slipping the measurement noise is inflated by the slip detector, the estimate
  falls back onto model integration, and the arc length therefore does not
  absorb the slip.
