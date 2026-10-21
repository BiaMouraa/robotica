#!/usr/bin/env python3
"""
Seguimento de trajetoria em 8 (lemniscata de Gerono) usando os SENSORES
do robo (Pratica 04 - Exercicio 2.1):

  * Orientacao (yaw)  -> IMU  (/imu/data, sensor_msgs/Imu)
  * Posicao (x, y)    -> GPS  (/gps/data, sensor_msgs/NavSatFix), convertida
                         de (lat, lon) para (X, Y) locais com origem no
                         primeiro fix (projecao equiretangular, ENU).

Com use_sensors:=false o no volta a usar /odom (como na pratica anterior),
o que permite comparar as duas abordagens.

Trajetoria de referencia:
    x_r(t) = A sin(w t)
    y_r(t) = (B/2) sin(2 w t)

Controle: feedforward (kff * velocidade da referencia) + realimentacao
proporcional (kp_x, kp_y) sobre um ponto de controle a "lookahead" metros
a frente do robo, convertido para (v, w) do modelo uniciclo.

Topicos publicados para o PlotJuggler / RViz:
    /cmd_vel                           geometry_msgs/Twist
    /trajectory_eight/reference        geometry_msgs/PointStamped  (x_r, y_r)
    /trajectory_eight/estimate         geometry_msgs/PointStamped  (x, y estimados; z = yaw)
    /trajectory_eight/gps_xy_raw       geometry_msgs/PointStamped  (GPS -> XY sem filtro)
    /trajectory_eight/error            geometry_msgs/Vector3Stamped (e_x, e_y, |e|)
    /trajectory_eight/reference_path   nav_msgs/Path
    /trajectory_eight/estimated_path   nav_msgs/Path
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
        self.declare_parameter('A', 1.0)             # amplitude em x [m]
        self.declare_parameter('B', 2.0)             # amplitude em y [m] (y = B/2 sin 2wt)
        self.declare_parameter('omega', 0.5)         # frequencia angular [rad/s]
        self.declare_parameter('kff', 1.0)           # ganho de feedforward
        self.declare_parameter('use_feedback', True)
        self.declare_parameter('kp_x', 1.0)
        self.declare_parameter('kp_y', 1.0)
        self.declare_parameter('use_sensors', True)  # True: IMU+GPS; False: /odom
        self.declare_parameter('lookahead', 0.15)    # ponto de controle a frente [m]
        self.declare_parameter('v_max', 0.8)
        self.declare_parameter('w_max', 2.5)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('start_delay', 2.0)   # espera sensores estabilizarem [s]
        self.declare_parameter('gps_origin_samples', 10)  # fixes usados para a origem
        # Sinal dos eixos (X <- Leste, Y <- Norte). O sensor de GPS do Gazebo 11
        # (classic) publica lat/lon com os eixos Leste e Norte INVERTIDOS
        # (bug conhecido, equivale a um heading de 180 graus); por isso o
        # padrao e -1/-1. Em um GPS real use +1/+1.
        self.declare_parameter('gps_x_sign', -1.0)
        self.declare_parameter('gps_y_sign', -1.0)
        # Filtro passa-baixa (media movel exponencial) da posicao do GPS.
        # 1.0 = sem filtro; valores menores suavizam o ruido (e atrasam).
        self.declare_parameter('gps_filter_alpha', 0.5)
        # Posicao da antena GPS no frame do robo (gps_link em relacao a
        # base_link, no URDF: xyz="-0.15 0 0.11"). Usada com o yaw da IMU
        # para trazer a medida do GPS para o centro do robo.
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
        self.use_sensors = gp('use_sensors').value
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

        # ---------------- Estado ----------------
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.have_pos = False
        self.have_yaw = False

        self.gps_origin_lat = None
        self.gps_origin_lon = None
        self._gps_acc = []  # acumula fixes iniciais para a origem
        self.gps_x_raw = self.gps_y_raw = 0.0
        self.base_origin = None

        self.odom_x = self.odom_y = self.odom_yaw = 0.0

        self.t0 = None
        self.first_ready_time = None
        self.last_log_time = None

        self.ref_path = Path()
        self.ref_path.header.frame_id = 'odom'
        self.est_path = Path()
        self.est_path.header.frame_id = 'odom'

        # ---------------- ROS I/O ----------------
        sensor_qos = QoSProfile(depth=10,
                                reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST)

        self.create_subscription(Imu, '/imu/data', self.imu_callback, sensor_qos)
        self.create_subscription(NavSatFix, '/gps/data', self.gps_callback, sensor_qos)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_ref = self.create_publisher(PointStamped, '/trajectory_eight/reference', 10)
        self.pub_est = self.create_publisher(PointStamped, '/trajectory_eight/estimate', 10)
        self.pub_gps_raw = self.create_publisher(PointStamped, '/trajectory_eight/gps_xy_raw', 10)
        self.pub_err = self.create_publisher(Vector3Stamped, '/trajectory_eight/error', 10)
        self.pub_ref_path = self.create_publisher(Path, '/trajectory_eight/reference_path', 10)
        self.pub_est_path = self.create_publisher(Path, '/trajectory_eight/estimated_path', 10)

        self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)

        fonte = 'IMU + GPS' if self.use_sensors else 'odometria (/odom)'
        self.get_logger().info(
            f'Trajetoria em 8: A={self.A} B={self.B} omega={self.omega} | '
            f'kff={self.kff} kp=({self.kp_x},{self.kp_y}) feedback={self.use_feedback} | '
            f'fonte de pose: {fonte}')

    # ---------------- Callbacks ----------------
    def imu_callback(self, msg: Imu):
        yaw = yaw_from_quaternion(msg.orientation)
        if self.use_sensors:
            self.yaw = yaw
            self.have_yaw = True

    def gps_callback(self, msg: NavSatFix):
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return

        # Origem = media dos primeiros N fixes (reduz o vies do ruido inicial)
        if self.gps_origin_lat is None:
            self._gps_acc.append((msg.latitude, msg.longitude))
            if len(self._gps_acc) < self.gps_origin_samples:
                return
            self.gps_origin_lat = sum(p[0] for p in self._gps_acc) / len(self._gps_acc)
            self.gps_origin_lon = sum(p[1] for p in self._gps_acc) / len(self._gps_acc)
            self.get_logger().info(
                f'Origem GPS definida: lat={self.gps_origin_lat:.7f}, lon={self.gps_origin_lon:.7f}')

        x, y = self.latlon_to_xy(msg.latitude, msg.longitude)
        self.gps_x_raw, self.gps_y_raw = x, y

        # Compensa o braco de alavanca da antena: centro = antena - R(yaw) * offset
        if not self.have_yaw:
            return
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        x -= c * self.gps_offset_x - s * self.gps_offset_y
        y -= s * self.gps_offset_x + c * self.gps_offset_y

        # Zera a posicao do CENTRO do robo no inicio (a referencia comeca em 0,0)
        if self.base_origin is None:
            self.base_origin = (x, y)
        x -= self.base_origin[0]
        y -= self.base_origin[1]

        if self.use_sensors:
            if not self.have_pos:
                self.x, self.y = x, y
            else:
                a = self.gps_filter_alpha
                self.x = (1.0 - a) * self.x + a * x
                self.y = (1.0 - a) * self.y + a * y
            self.have_pos = True

    def latlon_to_xy(self, lat, lon):
        """Projecao equiretangular local: X = Leste, Y = Norte (com sinais
        configuraveis, ver gps_x_sign / gps_y_sign)."""
        dlat = math.radians(lat - self.gps_origin_lat)
        dlon = math.radians(lon - self.gps_origin_lon)
        east = EARTH_RADIUS * dlon * math.cos(math.radians(self.gps_origin_lat))
        north = EARTH_RADIUS * dlat
        return self.gps_x_sign * east, self.gps_y_sign * north

    def odom_callback(self, msg: Odometry):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        if not self.use_sensors:
            self.x, self.y, self.yaw = self.odom_x, self.odom_y, self.odom_yaw
            self.have_pos = True
            self.have_yaw = True

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
            self.get_logger().info('Sensores prontos; iniciando em '
                                   f'{self.start_delay:.1f}s...')
        if (now - self.first_ready_time).nanoseconds * 1e-9 < self.start_delay:
            return
        if self.t0 is None:
            self.t0 = now
            self.get_logger().info('Seguimento da trajetoria em 8 iniciado.')

        t = (now - self.t0).nanoseconds * 1e-9
        xr, yr, vxr, vyr = self.reference(t)

        # Erro do centro do robo (para plotagem/analise)
        ex = xr - self.x
        ey = yr - self.y

        # Ponto de controle a frente do robo
        cth, sth = math.cos(self.yaw), math.sin(self.yaw)
        px = self.x + self.lookahead * cth
        py = self.y + self.lookahead * sth

        # Referencia correspondente ao ponto de controle: a referencia do
        # centro deslocada "lookahead" ao longo da tangente da trajetoria.
        # Assim o CENTRO do robo segue (x_r, y_r), sem atraso geometrico.
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

        # Uniciclo: velocidade do ponto de controle -> (v, w)
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
        raw.point.x, raw.point.y = self.gps_x_raw, self.gps_y_raw
        self.pub_gps_raw.publish(raw)

        err = Vector3Stamped()
        err.header = ref.header
        err.vector.x, err.vector.y, err.vector.z = ex, ey, math.hypot(ex, ey)
        self.pub_err.publish(err)

        pr = PoseStamped()
        pr.header = ref.header
        pr.pose.position.x, pr.pose.position.y = xr, yr
        pr.pose.orientation.w = 1.0
        self.ref_path.header.stamp = stamp
        self.ref_path.poses.append(pr)

        pe = PoseStamped()
        pe.header = ref.header
        pe.pose.position.x, pe.pose.position.y = self.x, self.y
        pe.pose.orientation.z = math.sin(self.yaw / 2.0)
        pe.pose.orientation.w = math.cos(self.yaw / 2.0)
        self.est_path.header.stamp = stamp
        self.est_path.poses.append(pe)

        if len(self.ref_path.poses) > self.path_max_points:
            del self.ref_path.poses[0]
            del self.est_path.poses[0]
        self.pub_ref_path.publish(self.ref_path)
        self.pub_est_path.publish(self.est_path)

        if self.last_log_time is None or (now - self.last_log_time).nanoseconds * 1e-9 >= 3.0:
            self.last_log_time = now
            self.get_logger().info(
                f't={t:5.1f}s ref=({xr:+.2f},{yr:+.2f}) est=({self.x:+.2f},{self.y:+.2f}) '
                f'yaw={math.degrees(self.yaw):+6.1f}deg |e|={math.hypot(ex, ey):.3f}m '
                f'odom=({self.odom_x:+.2f},{self.odom_y:+.2f})')

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
