#!/usr/bin/env python3
"""Measure the online behaviour of the running node: rate, latency, CPU, RAM."""
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from tram_vehicle_msgs.msg import VelocitySensor, DriverControllerCommand
from nav_msgs.msg import Odometry

WINDOW = float(sys.argv[1]) if len(sys.argv) > 1 else 300.0


class Watcher(Node):
    def __init__(self):
        super().__init__('odom_watcher')
        q = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST)
        self.vel = []
        self.pos = []
        self.last_in = {'front': None, 'rear': None, 'notch': None}
        self.lag = []
        self.t0 = time.time()
        self.create_subscription(VelocitySensor, '/result/velocity', self._v, q)
        self.create_subscription(Odometry, '/result/position', self._p, q)
        self.create_subscription(VelocitySensor, '/vehicle/front_bogie_velocity',
                                 lambda m: self._in('front', m), q)
        self.create_subscription(DriverControllerCommand,
                                 '/vehicle/driver_position_cmd',
                                 lambda m: self._in('notch', m), q)

    def _in(self, key, msg):
        self.last_in[key] = (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                             time.time())

    def _v(self, msg):
        s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.vel.append(s)
        now = time.time()
        newest = max((v[1] for v in self.last_in.values() if v), default=now)
        self.lag.append(now - newest)

    def _p(self, msg):
        self.pos.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)


rclpy.init()
w = Watcher()
end = time.time() + WINDOW
while time.time() < end:
    rclpy.spin_once(w, timeout_sec=0.2)

v = np.array(w.vel)
p = np.array(w.pos)
print(f'window {WINDOW:.0f} s')
print(f'/result/velocity  messages {len(v)}  rate {len(v)/WINDOW:.2f} Hz')
print(f'/result/position  messages {len(p)}  rate {len(p)/WINDOW:.2f} Hz')
if len(v) > 10:
    d = np.diff(v)
    print(f'  velocity stamp gaps: median {np.median(d)*1000:.1f} ms  '
          f'max {d.max()*1000:.1f} ms  (>0.05 s: {int((d>0.05).sum())})')
lag = np.array(w.lag) * 1000.0
if len(lag):
    print(f'  input->publication wall lag: median {np.median(lag):.1f} ms  '
          f'p95 {np.percentile(lag,95):.1f} ms  max {lag.max():.1f} ms')
w.destroy_node()
rclpy.shutdown()
