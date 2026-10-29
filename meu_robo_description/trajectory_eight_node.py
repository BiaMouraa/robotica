#!/usr/bin/env python3
"""
Seguimento de trajetoria em 8 (lemniscata de Gerono).

Pratica 05: o controle usa o estado estimado pelo filtro de Kalman
(/odometry/filtered, pacote robot_localization).

pose_source:
  ekf      -> /odometry/filtered   (padrao, pratica 05)
  sensors  -> IMU (yaw) + GPS (x,y) com filtro exponencial (pratica 04)
  odom     -> /odom                (odometria das rodas)

Trajetoria de referencia:
    x_r(t) = A sin(w t)
    y_r(t) = (B/2) sin(2 w t)

Controle: feedforward (kff * velocidade da referencia) + realimentacao
proporcional (kp_x, kp_y) sobre um ponto de controle a "lookahead" metros
a frente do robo, convertido para (v, w) do modelo uniciclo.

Topicos publicados para o PlotJuggler / RViz:
    /cmd_vel                           geometry_msgs/Twist
    /trajectory_eight/reference        geometry_msgs/PointStamped  (x_r, y_r)
    /trajectory_eight/estimate         geometry_msgs/PointStamped  (x, y, yaw usados no controle)
    /trajectory_eight/gps_xy_raw       geometry_msgs/PointStamped  (GPS+IMU brutos)
    /trajectory_eight/groundtruth      geometry_msgs/PointStamped  (pose real Gazebo)
    /trajectory_eight/error            geometry_msgs/Vector3Stamped (e_x, e_y, |e|)
    /trajectory_eight/reference_path   nav_msgs/Path
    /trajectory_eight/estimated_path   nav_msgs/Path
    /trajectory_eight/groundtruth_path nav_msgs/Path
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, PointStamped, Vector3Stamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu, NavSatFix

EARTH_RADIUS = 6378137.0  # WGS84 semi-eixo maior [m]


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class TrajectoryEightNode(Node):
    def __init__(self):
        super().__init__('trajectory_eight_node')

        # ---------------- Parametros ----------------
        self.declare_parameter('A', 1.0)
        self.declare_parameter('B', 2.0)
        self.declare_parameter('omega', 0.5)
        self.declare_parameter('kff', 1.0)
        self.declare_parameter('use_feedback', True)
        self.declare_parameter('kp_x', 1.0)
        self.declare_parameter('kp_y', 1.0)
        self.declare_parameter('pose_source', 'ekf')  # ekf | sensors | odom
        self.declare_parameter('use_sensors', True)   # legado da pratica 04
        self.declare_parameter('lookahead', 0.15)
        self.declare_parameter('v_max', 0.8)
        self.declare_parameter('w_max', 2.5)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('start_delay', 3.0)
        self.declare_parameter('gps_origin_samples', 10)
        self.declare_parameter('gps_x_sign', -1.0)
        self.declare_parameter('gps_y_sign', -1.0)
        self.declare_parameter('gps_filter_alpha', 0.5)
        self.declare_parameter('gps_offset_x', -0.15)
        self.declare_parameter('gps_offset_y', 0.0)
        self.declare_parameter('path_max_points', 2000)

        gp = self.get_parameter
        self.A = gp('A').value
        self.B = gp('B').value
        self.omega = gp('omega').value
        self.kff = gp('kff').value
        self.use_feedback = gp('use_feedback').value
        self.kp_x = gp('kp_x').value
        self.kp_y = gp('kp_y').value
        self.pose_source = str(gp('pose_source').value).lower().strip()
        use_sensors = gp('use_sensors').value
        if isinstance(use_sensors, str):
            use_sensors = use_sensors.lower() in ('true', '1', 'yes')
        # Compatibilidade: use_sensors:=false da pratica 04 equivale a odom
        if not use_sensors and self.pose_source == 'ekf':
            self.pose_source = 'odom'
        if self.pose_source not in ('ekf', 'sensors', 'odom'):
            self.get_logger().warn(
                f'pose_source={self.pose_source} invalido; usando ekf')
            self.pose_source = 'ekf'
        self.lookahead = max(gp('lookahead').value, 0.02)
        self.v_max = gp('v_max').value
        self.w_max = gp('w_max').value
        self.control_rate = gp('control_rate').value
        self.start_delay = gp('start_delay').value
        self.gps_origin_samples = max(int(gp('gps_origin_samples').value), 1)
        self.gps_x_sign = gp('gps_x_sign').value
        self.gps_y_sign = gp('gps_y_sign').value
        self.gps_filter_alpha = min(max(gp('gps_filter_alpha').value, 0.01), 1.0)
        self.gps_offset_x = gp('gps_offset_x').value
        self.gps_offset_y = gp('gps_offset_y').value
        self.path_max_points = int(gp('path_max_points').value)

        # ---------------- Estado usado no controle ----------------
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.have_pos = False
        self.have_yaw = False

        # Sensores brutos (plot) e fallback pose_source=sensors
        self.gps_origin_lat = None
        self.gps_origin_lon = None
        self._gps_acc = []
        self.gps_x_raw = self.gps_y_raw = 0.0
        self.base_origin = None
        self.imu_yaw = 0.0

        self.odom_x = self.odom_y = self.odom_yaw = 0.0
        self.gt_x = self.gt_y = self.gt_yaw = 0.0
        self.have_gt = False

        self.t0 = None
        self.first_ready_time = None
        self.last_log_time = None

        self.ref_path = Path()
        self.ref_path.header.frame_id = 'odom'
        self.est_path = Path()
        self.est_path.header.frame_id = 'odom'
        self.gt_path = Path()
        self.gt_path.header.frame_id = 'odom'

        # ---------------- ROS I/O ----------------
        sensor_qos = QoSProfile(depth=10,
                                reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST)

        self.create_subscription(Imu, '/imu/data', self.imu_callback, sensor_qos)
        self.create_subscription(NavSatFix, '/gps/data', self.gps_callback, sensor_qos)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.ekf_callback, 10)
        self.create_subscription(Odometry, '/ground_truth', self.gt_callback, 10)
        self.create_subscription(PointStamped, '/state/raw', self.raw_callback, 10)

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_ref = self.create_publisher(PointStamped, '/trajectory_eight/reference', 10)
        self.pub_est = self.create_publisher(PointStamped, '/trajectory_eight/estimate', 10)
        self.pub_gps_raw = self.create_publisher(PointStamped, '/trajectory_eight/gps_xy_raw', 10)
        self.pub_gt = self.create_publisher(PointStamped, '/trajectory_eight/groundtruth', 10)
        self.pub_err = self.create_publisher(Vector3Stamped, '/trajectory_eight/error', 10)
        self.pub_ref_path = self.create_publisher(Path, '/trajectory_eight/reference_path', 10)
        self.pub_est_path = self.create_publisher(Path, '/trajectory_eight/estimated_path', 10)
        self.pub_gt_path = self.create_publisher(Path, '/trajectory_eight/groundtruth_path', 10)

        self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)

        self.get_logger().info(
            f'Trajetoria em 8: A={self.A} B={self.B} omega={self.omega} | '
            f'kff={self.kff} kp=({self.kp_x},{self.kp_y}) feedback={self.use_feedback} | '
            f'fonte de pose: {self.pose_source}')

    # ---------------- Callbacks ----------------
    def imu_callback(self, msg: Imu):
        self.imu_yaw = yaw_from_quaternion(msg.orientation)
        if self.pose_source == 'sensors':
            self.yaw = self.imu_yaw
            self.have_yaw = True

    def gps_callback(self, msg: NavSatFix):
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return

        if self.gps_origin_lat is None:
            self._gps_acc.append((msg.latitude, msg.longitude))
            if len(self._gps_acc) < self.gps_origin_samples:
                return
            self.gps_origin_lat = sum(p[0] for p in self._gps_acc) / len(self._gps_acc)
            self.gps_origin_lon = sum(p[1] for p in self._gps_acc) / len(self._gps_acc)

        x, y = self.latlon_to_xy(msg.latitude, msg.longitude)
        self.gps_x_raw, self.gps_y_raw = x, y

        if self.pose_source != 'sensors':
            return

        if not self.have_yaw:
            return
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        x -= c * self.gps_offset_x - s * self.gps_offset_y
        y -= s * self.gps_offset_x + c * self.gps_offset_y

        if self.base_origin is None:
            self.base_origin = (x, y)
        x -= self.base_origin[0]
        y -= self.base_origin[1]

        if not self.have_pos:
            self.x, self.y = x, y
        else:
            a = self.gps_filter_alpha
            self.x = (1.0 - a) * self.x + a * x
            self.y = (1.0 - a) * self.y + a * y
        self.have_pos = True

    def latlon_to_xy(self, lat, lon):
        dlat = math.radians(lat - self.gps_origin_lat)
        dlon = math.radians(lon - self.gps_origin_lon)
        east = EARTH_RADIUS * dlon * math.cos(math.radians(self.gps_origin_lat))
        north = EARTH_RADIUS * dlat
        return self.gps_x_sign * east, self.gps_y_sign * north

    def odom_callback(self, msg: Odometry):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        if self.pose_source == 'odom':
            self.x, self.y, self.yaw = self.odom_x, self.odom_y, self.odom_yaw
            self.have_pos = True
            self.have_yaw = True

    def ekf_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        if self.pose_source == 'ekf':
            self.x, self.y, self.yaw = x, y, yaw
            self.have_pos = True
            self.have_yaw = True

    def gt_callback(self, msg: Odometry):
        self.gt_x = msg.pose.pose.position.x
        self.gt_y = msg.pose.pose.position.y
        self.gt_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.have_gt = True

    def raw_callback(self, msg: PointStamped):
        # Estado bruto ja no frame odom (origem + braco de alavanca compensados)
        self.gps_x_raw = msg.point.x
        self.gps_y_raw = msg.point.y

    # ---------------- Referencia ----------------
    def reference(self, t):
        w = self.omega
        xr = self.A * math.sin(w * t)
        yr = 0.5 * self.B * math.sin(2.0 * w * t)
        vxr = self.A * w * math.cos(w * t)
        vyr = self.B * w * math.cos(2.0 * w * t)
        return xr, yr, vxr, vyr

    # ---------------- Controle ----------------
    def control_loop(self):
        now = self.get_clock().now()

        if not (self.have_pos and self.have_yaw):
            return

        if self.first_ready_time is None:
            self.first_ready_time = now
            self.get_logger().info('Estado pronto; iniciando em '
                                   f'{self.start_delay:.1f}s...')
        if (now - self.first_ready_time).nanoseconds * 1e-9 < self.start_delay:
            return
        if self.t0 is None:
            self.t0 = now
            self.get_logger().info('Seguimento da trajetoria em 8 iniciado.')

        t = (now - self.t0).nanoseconds * 1e-9
        xr, yr, vxr, vyr = self.reference(t)

        ex = xr - self.x
        ey = yr - self.y

        cth, sth = math.cos(self.yaw), math.sin(self.yaw)
        px = self.x + self.lookahead * cth
        py = self.y + self.lookahead * sth

        speed_r = math.hypot(vxr, vyr)
        if speed_r > 1e-3:
            pxr = xr + self.lookahead * vxr / speed_r
            pyr = yr + self.lookahead * vyr / speed_r
        else:
            pxr, pyr = xr, yr

        ux = self.kff * vxr
        uy = self.kff * vyr
        if self.use_feedback:
            ux += self.kp_x * (pxr - px)
            uy += self.kp_y * (pyr - py)

        v = ux * cth + uy * sth
        w = (-ux * sth + uy * cth) / self.lookahead

        v = max(-self.v_max, min(self.v_max, v))
        w = max(-self.w_max, min(self.w_max, w))

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.pub_cmd.publish(cmd)

        self.publish_debug(now, t, xr, yr, ex, ey)

    def publish_debug(self, now, t, xr, yr, ex, ey):
        stamp = now.to_msg()

        ref = PointStamped()
        ref.header.stamp = stamp
        ref.header.frame_id = 'odom'
        ref.point.x, ref.point.y = xr, yr
        self.pub_ref.publish(ref)

        est = PointStamped()
        est.header = ref.header
        est.point.x, est.point.y, est.point.z = self.x, self.y, self.yaw
        self.pub_est.publish(est)

        raw = PointStamped()
        raw.header = ref.header
        raw.point.x, raw.point.y, raw.point.z = self.gps_x_raw, self.gps_y_raw, self.imu_yaw
        self.pub_gps_raw.publish(raw)

        if self.have_gt:
            gt = PointStamped()
            gt.header = ref.header
            gt.point.x, gt.point.y, gt.point.z = self.gt_x, self.gt_y, self.gt_yaw
            self.pub_gt.publish(gt)

        err = Vector3Stamped()
        err.header = ref.header
        err.vector.x, err.vector.y, err.vector.z = ex, ey, math.hypot(ex, ey)
        self.pub_err.publish(err)

        def append_pose(path, x, y, yaw=0.0):
            ps = PoseStamped()
            ps.header = ref.header
            ps.pose.position.x, ps.pose.position.y = x, y
            ps.pose.orientation.z = math.sin(yaw / 2.0)
            ps.pose.orientation.w = math.cos(yaw / 2.0)
            path.header.stamp = stamp
            path.poses.append(ps)

        append_pose(self.ref_path, xr, yr)
        append_pose(self.est_path, self.x, self.y, self.yaw)
        if self.have_gt:
            append_pose(self.gt_path, self.gt_x, self.gt_y, self.gt_yaw)

        if len(self.ref_path.poses) > self.path_max_points:
            del self.ref_path.poses[0]
            del self.est_path.poses[0]
            if self.gt_path.poses:
                del self.gt_path.poses[0]
        self.pub_ref_path.publish(self.ref_path)
        self.pub_est_path.publish(self.est_path)
        if self.gt_path.poses:
            self.pub_gt_path.publish(self.gt_path)

        if self.last_log_time is None or (now - self.last_log_time).nanoseconds * 1e-9 >= 3.0:
            self.last_log_time = now
            gt_txt = (f'gt=({self.gt_x:+.2f},{self.gt_y:+.2f})' if self.have_gt else 'gt=n/a')
            self.get_logger().info(
                f't={t:5.1f}s ref=({xr:+.2f},{yr:+.2f}) est=({self.x:+.2f},{self.y:+.2f}) '
                f'yaw={math.degrees(self.yaw):+6.1f}deg |e|={math.hypot(ex, ey):.3f}m '
                f'{gt_txt} raw=({self.gps_x_raw:+.2f},{self.gps_y_raw:+.2f})')

    def stop(self):
        self.pub_cmd.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryEightNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
