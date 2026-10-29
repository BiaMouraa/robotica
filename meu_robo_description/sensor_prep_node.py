#!/usr/bin/env python3
"""
Prepara as medicoes de IMU e GPS para o EKF do robot_localization
(Pratica 05 - Estimacao de estados).

O Gazebo publica:
  * /imu/data     sensor_msgs/Imu          (orientacao sem ruido; gyro/accel com ruido)
  * /gps/data     sensor_msgs/NavSatFix    (lat/lon com ruido)

Este no:
  1. Converte lat/lon para XY local (mesma projecao da pratica 04, incluindo
     a inversao dos eixos Leste/Norte do GPS do Gazebo Classic) e publica
     /odometry/gps (nav_msgs/Odometry) com covariancia.
  2. Injeta ruido e covariancia na orientacao da IMU (o plugin do Gazebo
     nao ruidifica o quaternion) e republica em /imu/data_cov.
  3. Republica estados em PointStamped para o PlotJuggler:
       /state/raw          GPS+IMU brutos (x, y, yaw)
       /state/filtered     saida do EKF
       /state/groundtruth  pose real do Gazebo (/gazebo/model_states)
"""
import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PointStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from gazebo_msgs.msg import ModelStates

EARTH_RADIUS = 6378137.0


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class SensorPrepNode(Node):
    def __init__(self):
        super().__init__('sensor_prep_node')

        self.declare_parameter('noise_scale', 1.0)
        self.declare_parameter('gps_h_std', 0.05)
        self.declare_parameter('imu_yaw_std', 0.03)      # rad (nominal, escala 1)
        self.declare_parameter('imu_gyro_std', 0.01)     # rad/s
        self.declare_parameter('imu_accel_std', 0.10)    # m/s^2
        self.declare_parameter('gps_origin_samples', 10)
        self.declare_parameter('gps_x_sign', -1.0)
        self.declare_parameter('gps_y_sign', -1.0)
        self.declare_parameter('gps_offset_x', -0.15)
        self.declare_parameter('gps_offset_y', 0.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('robot_model_name', 'meu_robo')

        gp = self.get_parameter
        ns = max(float(gp('noise_scale').value), 1e-6)
        self.gps_h_std = float(gp('gps_h_std').value) * ns
        self.imu_yaw_std = float(gp('imu_yaw_std').value) * ns
        self.imu_gyro_std = float(gp('imu_gyro_std').value) * ns
        self.imu_accel_std = float(gp('imu_accel_std').value) * ns
        self.gps_origin_samples = max(int(gp('gps_origin_samples').value), 1)
        self.gps_x_sign = float(gp('gps_x_sign').value)
        self.gps_y_sign = float(gp('gps_y_sign').value)
        self.gps_offset_x = float(gp('gps_offset_x').value)
        self.gps_offset_y = float(gp('gps_offset_y').value)
        self.odom_frame = gp('odom_frame').value
        self.base_frame = gp('base_frame').value
        self.robot_model_name = gp('robot_model_name').value
        self.gt_origin = None

        self.gps_origin_lat = None
        self.gps_origin_lon = None
        self._gps_acc = []
        self.base_origin = None

        self.yaw = 0.0
        self.have_yaw = False
        self.x_raw = 0.0
        self.y_raw = 0.0
        self.have_pos = False

        sensor_qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        # Saida RELIABLE: o EKF do robot_localization nao usa BEST_EFFORT.
        reliable_qos = 10

        self.create_subscription(Imu, '/imu/data', self.imu_callback, sensor_qos)
        self.create_subscription(NavSatFix, '/gps/data', self.gps_callback, sensor_qos)
        self.create_subscription(ModelStates, '/gazebo/model_states', self.model_states_callback, sensor_qos)
        self.create_subscription(Odometry, '/odometry/filtered', self.ekf_callback, 10)

        self.pub_imu = self.create_publisher(Imu, '/imu/data_cov', reliable_qos)
        self.pub_gps_odom = self.create_publisher(Odometry, '/odometry/gps', reliable_qos)
        self.pub_gt_odom = self.create_publisher(Odometry, '/ground_truth', reliable_qos)
        self.pub_raw = self.create_publisher(PointStamped, '/state/raw', reliable_qos)
        self.pub_gt = self.create_publisher(PointStamped, '/state/groundtruth', reliable_qos)
        self.pub_filt = self.create_publisher(PointStamped, '/state/filtered', reliable_qos)

        self.get_logger().info(
            f'sensor_prep: noise_scale efetivo | GPS std={self.gps_h_std:.3f} m | '
            f'IMU yaw std={self.imu_yaw_std:.3f} rad '
            f'({math.degrees(self.imu_yaw_std):.1f} deg)')

    def imu_callback(self, msg: Imu):
        yaw_true = yaw_from_quaternion(msg.orientation)
        yaw = yaw_true + random.gauss(0.0, self.imu_yaw_std)
        self.yaw = yaw
        self.have_yaw = True

        out = Imu()
        out.header = msg.header
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        out.orientation.x = qx
        out.orientation.y = qy
        out.orientation.z = qz
        out.orientation.w = qw
        out.angular_velocity = msg.angular_velocity
        out.linear_acceleration = msg.linear_acceleration

        yaw_var = self.imu_yaw_std ** 2
        gyro_var = self.imu_gyro_std ** 2
        accel_var = self.imu_accel_std ** 2
        out.orientation_covariance[0] = 0.25
        out.orientation_covariance[4] = 0.25
        out.orientation_covariance[8] = max(yaw_var, 1e-6)
        out.angular_velocity_covariance[0] = gyro_var
        out.angular_velocity_covariance[4] = gyro_var
        out.angular_velocity_covariance[8] = max(gyro_var, 1e-6)
        out.linear_acceleration_covariance[0] = max(accel_var, 1e-6)
        out.linear_acceleration_covariance[4] = max(accel_var, 1e-6)
        out.linear_acceleration_covariance[8] = max(accel_var, 1e-6)

        self.pub_imu.publish(out)
        self.publish_raw(msg.header.stamp)

    def gps_callback(self, msg: NavSatFix):
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return

        if self.gps_origin_lat is None:
            self._gps_acc.append((msg.latitude, msg.longitude))
            if len(self._gps_acc) < self.gps_origin_samples:
                return
            self.gps_origin_lat = sum(p[0] for p in self._gps_acc) / len(self._gps_acc)
            self.gps_origin_lon = sum(p[1] for p in self._gps_acc) / len(self._gps_acc)
            self.get_logger().info(
                f'Origem GPS: lat={self.gps_origin_lat:.7f}, lon={self.gps_origin_lon:.7f}')

        x, y = self.latlon_to_xy(msg.latitude, msg.longitude)

        if not self.have_yaw:
            return
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        x -= c * self.gps_offset_x - s * self.gps_offset_y
        y -= s * self.gps_offset_x + c * self.gps_offset_y

        if self.base_origin is None:
            self.base_origin = (x, y)
        x -= self.base_origin[0]
        y -= self.base_origin[1]

        self.x_raw, self.y_raw = x, y
        self.have_pos = True

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        qx, qy, qz, qw = quaternion_from_yaw(self.yaw)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        var_xy = max(self.gps_h_std ** 2, 1e-6)
        cov = [0.0] * 36
        cov[0] = var_xy
        cov[7] = var_xy
        cov[14] = 1e3
        cov[21] = 1e3
        cov[28] = 1e3
        cov[35] = 1e3
        odom.pose.covariance = cov
        odom.twist.covariance = [0.0] * 36
        self.pub_gps_odom.publish(odom)
        self.publish_raw(msg.header.stamp)

    def latlon_to_xy(self, lat, lon):
        dlat = math.radians(lat - self.gps_origin_lat)
        dlon = math.radians(lon - self.gps_origin_lon)
        east = EARTH_RADIUS * dlon * math.cos(math.radians(self.gps_origin_lat))
        north = EARTH_RADIUS * dlat
        return self.gps_x_sign * east, self.gps_y_sign * north

    def publish_raw(self, stamp):
        if not (self.have_pos and self.have_yaw):
            return
        p = PointStamped()
        p.header.stamp = stamp
        p.header.frame_id = self.odom_frame
        p.point.x = self.x_raw
        p.point.y = self.y_raw
        p.point.z = self.yaw
        self.pub_raw.publish(p)

    def model_states_callback(self, msg: ModelStates):
        if self.robot_model_name not in msg.name:
            return
        i = msg.name.index(self.robot_model_name)
        pose = msg.pose[i]
        twist = msg.twist[i]
        yaw = yaw_from_quaternion(pose.orientation)

        if self.gt_origin is None:
            self.gt_origin = (pose.position.x, pose.position.y)
            self.get_logger().info(
                f'Origem ground truth: x={pose.position.x:.3f}, y={pose.position.y:.3f}')

        x = pose.position.x - self.gt_origin[0]
        y = pose.position.y - self.gt_origin[1]
        yaw_rel = yaw

        stamp = self.get_clock().now().to_msg()
        qx, qy, qz, qw = quaternion_from_yaw(yaw_rel)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        odom.twist.twist = twist
        self.pub_gt_odom.publish(odom)

        p = PointStamped()
        p.header.stamp = stamp
        p.header.frame_id = self.odom_frame
        p.point.x = x
        p.point.y = y
        p.point.z = yaw_rel
        self.pub_gt.publish(p)

    def ekf_callback(self, msg: Odometry):
        p = PointStamped()
        p.header = msg.header
        p.header.frame_id = self.odom_frame
        p.point.x = msg.pose.pose.position.x
        p.point.y = msg.pose.pose.position.y
        p.point.z = yaw_from_quaternion(msg.pose.pose.orientation)
        self.pub_filt.publish(p)


def main(args=None):
    rclpy.init(args=args)
    node = SensorPrepNode()
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
