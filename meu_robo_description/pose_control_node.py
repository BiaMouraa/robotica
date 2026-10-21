#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry

class PoseControlNode(Node):
    def __init__(self):
        super().__init__('pose_control_node')
        
        # Parametros do controlador
        self.declare_parameter('kp_linear', 0.5)
        self.declare_parameter('kp_angular', 1.5)
        self.declare_parameter('tolerance', 0.05)
        
        self.kp_linear = self.get_parameter('kp_linear').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.tolerance = self.get_parameter('tolerance').value
        
        # Posicao atual e alvo
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        self.target_x = None
        self.target_y = None
        self.target_yaw = None
        
        # Subscribers e Publishers
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub_target = self.create_subscription(PoseStamped, '/target_pose', self.target_callback, 10)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("Nó pose_control_node inscrito em /target_pose e /odom!")

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def target_callback(self, msg):
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y
        
        q = msg.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.target_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.get_logger().info(f"Alvo recebido: x={self.target_x:.2f}, y={self.target_y:.2f}")

    def control_loop(self):
        if self.target_x is None:
            return

        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        distance = math.hypot(dx, dy)

        cmd = Twist()

        if distance > self.tolerance:
            target_angle = math.atan2(dy, dx)
            alpha = target_angle - self.current_yaw
            alpha = math.atan2(math.sin(alpha), math.cos(alpha))

            cmd.linear.x = min(self.kp_linear * distance, 0.5)
            cmd.angular.z = self.kp_angular * alpha
        else:
            angle_error = self.target_yaw - self.current_yaw
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
            
            if abs(angle_error) > 0.05:
                cmd.linear.x = 0.0
                cmd.angular.z = self.kp_angular * angle_error
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.get_logger().info("Alvo alcançado!")
                self.target_x = None

        self.pub_cmd_vel.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PoseControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()    
