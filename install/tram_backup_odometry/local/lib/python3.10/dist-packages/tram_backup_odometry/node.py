"""ROS 2 node: model-based backup odometry of the tram.

Subscribes (the only inputs used by the estimation loop)
    /vehicle/front_bogie_velocity     tram_vehicle_msgs/msg/VelocitySensor
    /vehicle/rear_bogie_velocity      tram_vehicle_msgs/msg/VelocitySensor
    /vehicle/driver_position_cmd      tram_vehicle_msgs/msg/DriverControllerCommand
and, **only for the initial alignment inside the configured window**
    /sensing/gnss/{master,rover}/fix  sensor_msgs/msg/NavSatFix
    /sensing/gnss/{master,rover}/vel  geometry_msgs/msg/TwistStamped

Publishes
    /result/velocity    tram_vehicle_msgs/msg/VelocitySensor  (m/s)
    /result/position    nav_msgs/msg/Odometry                 (local metric frame)
    /result/diagnostics diagnostic_msgs/msg/DiagnosticArray
    /result/slip_ratio   std_msgs/msg/Float32
    /result/slipping     std_msgs/msg/Bool
    /result/model_accel  std_msgs/msg/Float32
"""
from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Bool, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TwistStamped
from sensor_msgs.msg import NavSatFix
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from tram_vehicle_msgs.msg import VelocitySensor, DriverControllerCommand

from .core import BackupOdometryCore
from .params import Config


class BackupOdometryNode(Node):
    def __init__(self):
        super().__init__('tram_backup_odometry')

        self.cfg = Config()
        self.cfg.declare(self)
        self.cfg.load(self)
        self.core = BackupOdometryCore(self.cfg)

        t, o = self.cfg.topics, self.cfg.out
        qos_in = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                            history=HistoryPolicy.KEEP_LAST)
        self._qos_out = QoSProfile(depth=max(1, o.queue_depth),
                                   reliability=ReliabilityPolicy.RELIABLE,
                                   history=HistoryPolicy.KEEP_LAST)

        self.create_subscription(VelocitySensor, t.front_velocity,
                                 lambda m: self._front(m), qos_in)
        self.create_subscription(VelocitySensor, t.rear_velocity,
                                 lambda m: self._rear(m), qos_in)
        self.create_subscription(DriverControllerCommand, t.driver_command,
                                 lambda m: self._notch(m), qos_in)
        if self.core.aligner is not None:
            self.create_subscription(NavSatFix, t.gnss_fix_primary,
                                     lambda m: self._fix(0, m), qos_in)
            self.create_subscription(NavSatFix, t.gnss_fix_secondary,
                                     lambda m: self._fix(1, m), qos_in)
            self.create_subscription(TwistStamped, t.gnss_vel_primary,
                                     lambda m: self._gnss_vel(0, m), qos_in)
            self.create_subscription(TwistStamped, t.gnss_vel_secondary,
                                     lambda m: self._gnss_vel(1, m), qos_in)

        self.pub_vel = self.create_publisher(VelocitySensor, t.out_velocity, self._qos_out)
        self.pub_pos = self.create_publisher(Odometry, t.out_position, self._qos_out)
        self.pub_diag = self.create_publisher(DiagnosticArray, t.out_diagnostics, self._qos_out)
        self.pub_kappa = self.create_publisher(Float32, t.out_slip_ratio, self._qos_out)
        self.pub_slip = self.create_publisher(Bool, t.out_slip_active, self._qos_out)
        self.pub_amodel = self.create_publisher(Float32, t.out_model_accel, self._qos_out)

        self.timer = self.create_timer(1.0 / max(o.publish_rate_hz, 1.0), self._tick)
        self._diag_every = max(1, int(max(o.publish_rate_hz, 1.0) / 5.0))
        self._diag_n = 0
        self._warned = False
        self._logged_ok = False
        self.get_logger().info(
            'tram_backup_odometry ready | wheel_unit_scale=%.6f  map=%s  init_mode=%s  '
            'rate=%.0f Hz' % (self.cfg.inp.wheel_unit_scale,
                              'yes' if self.core.route else 'no',
                              self.cfg.route.init_mode, o.publish_rate_hz))

    # ------------------------------------------------------------- callbacks
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _front(self, msg: VelocitySensor) -> None:
        s = _stamp(msg.header.stamp)
        self.core.on_front(msg.velocity, s if s > 0.0 else self._now())

    def _rear(self, msg: VelocitySensor) -> None:
        s = _stamp(msg.header.stamp)
        self.core.on_rear(msg.velocity, s if s > 0.0 else self._now())

    def _notch(self, msg: DriverControllerCommand) -> None:
        s = _stamp(msg.header.stamp)
        self.core.on_notch(int(msg.position), s if s > 0.0 else self._now())

    def _fix(self, source: int, msg: NavSatFix) -> None:
        if self.core.aligner is None or not self.core.aligner.active:
            return
        s = _stamp(msg.header.stamp)
        self.core.on_fix(source, msg.latitude, msg.longitude, msg.altitude,
                         s if s > 0.0 else self._now())

    def _gnss_vel(self, source: int, msg: TwistStamped) -> None:
        if self.core.aligner is None or not self.core.aligner.active:
            return
        s = _stamp(msg.header.stamp)
        v = math.hypot(msg.twist.linear.x, msg.twist.linear.y)
        self.core.on_gnss_velocity(source, s if s > 0.0 else self._now(), v)

    # ------------------------------------------------------------------- tick
    def _tick(self) -> None:
        core = self.core
        if not core.pre.accepted:
            return
        stamp = core.pre.stamp
        if core.step(stamp):
            self._publish(stamp)
        else:
            # no new input: republish the current state so that the output rate
            # stays >= publish_rate_hz even across input dropouts
            self._publish(stamp)
        self._log_alignment()

    def _log_alignment(self) -> None:
        core = self.core
        st = core.align_status
        if st == 'matched' and not self._logged_ok:
            al = core.aligner.result
            self._logged_ok = True
            self.get_logger().info(
                'initial alignment OK: map branch "%s" arc0=%.1f m dir=%+d yaw=%+.3f deg '
                'residual=%.2f m%s, ambiguity x%.2f, wheel distance scale=%.5f'
                % (core.route.name if core.route else '?', al.arc0, int(al.direction),
                   math.degrees(al.yaw), al.residual,
                   ' (weak match)' if al.weak else '', al.ambiguity,
                   core.wheel_scale))
        if st in ('rejected', 'insufficient-motion', 'timeout', 'ambiguous') and not self._warned:
            self._warned = True
            self.get_logger().warn(
                'initial alignment %s (%s): falling back to relative odometry '
                'from the start point' % (st, core.align_reason))

    # ---------------------------------------------------------------- publish
    def _publish(self, stamp: float) -> None:
        core, cfg = self.core, self.cfg
        from builtin_interfaces.msg import Time
        ns = int(stamp * 1e9)
        ts = Time(sec=ns // 1000000000, nanosec=ns % 1000000000)

        v = core.velocity
        vel = VelocitySensor()
        vel.header.stamp = ts
        vel.header.frame_id = 'base_link'
        vel.velocity = v
        self.pub_vel.publish(vel)

        x, y, z, yaw, mapped = core.position()
        od = Odometry()
        od.header.stamp = ts
        od.header.frame_id = cfg.out.frame_id if mapped else 'odom'
        od.pose.pose.position.x = x
        od.pose.pose.position.y = y
        od.pose.pose.position.z = z
        od.pose.pose.orientation = Quaternion(z=math.sin(yaw * 0.5), w=math.cos(yaw * 0.5))
        od.twist.twist.linear.x = v
        if cfg.out.publish_covariance:
            pvar = (cfg.out.pos_sigma_base + cfg.out.pos_sigma_rate * core.est.s) ** 2
            od.pose.covariance[0] = pvar
            od.pose.covariance[7] = pvar
            od.pose.covariance[14] = 4.0 * pvar
            od.pose.covariance[21] = 0.05
            vvar = max(core.est.variance, cfg.out.vel_sigma ** 2)
            od.twist.covariance[0] = vvar
            od.twist.covariance[35] = 0.1
        self.pub_pos.publish(od)

        f = Float32()
        f.data = core.slip.kappa
        self.pub_kappa.publish(f)
        b = Bool()
        b.data = bool(core.slip.active)
        self.pub_slip.publish(b)
        f2 = Float32()
        f2.data = core.est.a_model
        self.pub_amodel.publish(f2)

        self._diag_n += 1
        if self._diag_n % self._diag_every == 0:
            self._publish_diag(ts)

    def _publish_diag(self, ts) -> None:
        d = self.core.diagnostics()
        arr = DiagnosticArray()
        arr.header.stamp = ts
        arr.header.frame_id = 'base_link'
        st = DiagnosticStatus()
        st.name = 'tram_backup_odometry: velocity/position estimator'
        st.hardware_id = 'model-based backup odometry'
        st.level = DiagnosticStatus.WARN if self.core.slip.active else DiagnosticStatus.OK
        st.message = ('slip detected (kappa=%.3f)' % self.core.slip.kappa
                      if self.core.slip.active else 'nominal')
        for k, val in d.items():
            kv = KeyValue()
            kv.key = k
            kv.value = ('%.6f' % val) if isinstance(val, float) else str(val)
            st.values.append(kv)
        arr.status.append(st)
        self.pub_diag.publish(arr)


def _stamp(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = BackupOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
