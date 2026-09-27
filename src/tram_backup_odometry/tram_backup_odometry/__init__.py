"""Model-based backup odometry for an autonomous tram.

The package estimates the longitudinal velocity and the position of the tram
using **only** the driver controller position and the two bogie wheel speeds
(no GNSS, no IMU inside the estimation loop).

Logical blocks (see README.md for the full description):

    preprocess.InputPreprocessor   reception, unit conversion, glitch/outlier
                                   rejection, time synchronisation, watchdog
    model.TractionModel            non-linear traction characteristic  F(u, v)
    model.LongitudinalDynamics     resistance / grade / adhesion limits
    model.AdaptiveBias             online estimate of the unmodelled grade term
    slip.SlipDetector              slip ratio, adhesion estimate, cross-bogie
                                   consistency, event detection
    estimator.VelocityEstimator    2-state EKF fusing wheel odometry with the
                                   model; publishes velocity and arc length
    route.RouteMap                 pathgraph map: arc-length -> position/yaw
    route.InitialAligner           GNSS-based initial alignment (allowed) and
                                   one-shot wheel-scale calibration
"""

__version__ = '1.0.0'
