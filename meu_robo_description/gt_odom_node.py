#!/usr/bin/env python3
"""
Publica odometria ground truth a partir de /gazebo/model_states
(Pratica 05.2 - cenarios sem ruido / odometria perfeita).

Saidas:
  /odom                  nav_msgs/Odometry
  TF odom -> base_footprint
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import TransformStamped
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class GtOdomNode(Node):
    def __init__(self):
        super().__init__('gt_odom_node')
        self.declare_parameter('robot_model_name', 'meu_robo')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_tf', True)

        self.robot_model_name = self.get_parameter('robot_model_name').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value

        self.origin = None
        self.last_pose = None
        self.last_time = None

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(ModelStates, '/gazebo/model_states',
                                 self.callback, qos)
        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info(
            f'gt_odom_node: model={self.robot_model_name} '
            f'-> /odom + TF {self.odom_frame}->{self.base_frame}')

    def callback(self, msg: ModelStates):
        if self.robot_model_name not in msg.name:
            return
        i = msg.name.index(self.robot_model_name)
        pose = msg.pose[i]
        twist_world = msg.twist[i]
        now = self.get_clock().now()

        if self.origin is None:
            self.origin = (pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation))
            self.get_logger().info(
                f'Origem GT odom: x={self.origin[0]:.3f} y={self.origin[1]:.3f}')

        yaw = yaw_from_quaternion(pose.orientation)
        x = pose.position.x - self.origin[0]
        y = pose.position.y - self.origin[1]
        # yaw absoluto (mesmo frame do Gazebo / ENU) — suficiente para SLAM planar
        qz = math.sin(yaw * 0.5)
        qw = math.cos(yaw * 0.5)

        # Velocidade no frame do corpo
        c, s = math.cos(yaw), math.sin(yaw)
        vx = c * twist_world.linear.x + s * twist_world.linear.y
        vy = -s * twist_world.linear.x + c * twist_world.linear.y

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = twist_world.angular.z
        # Covariancia pequena = "tudo groundtruth"
        for idx in (0, 7, 14, 21, 28, 35):
            odom.pose.covariance[idx] = 1e-6
            odom.twist.covariance[idx] = 1e-6
        self.pub.publish(odom)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = GtOdomNode()
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
