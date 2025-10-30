#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile
import numpy as np

class MoveRelative(Node):
    def __init__(self):
        super().__init__('move_relative')
        qos = QoSProfile(depth=10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)
        self.create_subscription(Odometry, 'odom', self.odom_callback, qos)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.get_logger().info('Relative movement node initialized.')

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        _, _, self.theta = self.euler_from_quaternion(msg.pose.pose.orientation)

    def euler_from_quaternion(self, q):
        x, y, z, w = q.x, q.y, q.z, q.w
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return 0.0, 0.0, math.atan2(siny_cosp, cosy_cosp)

    def move_relative(self, dx, dy, dtheta):
        x_goal = self.x + math.cos(self.theta) * dx - math.sin(self.theta) * dy
        y_goal = self.y + math.sin(self.theta) * dx + math.cos(self.theta) * dy
        theta_goal = self.theta + math.radians(dtheta)
        self.move_to(x_goal, y_goal, theta_goal)

    def move_to(self, x_goal, y_goal, theta_goal):
        twist = Twist()
        while rclpy.ok():
            dx = x_goal - self.x
            dy = y_goal - self.y
            distance = math.sqrt(dx**2 + dy**2)
            target_angle = math.atan2(dy, dx)
            angle_diff = self.normalize_angle(target_angle - self.theta)

            if abs(angle_diff) > 0.05:
                twist.linear.x = 0.0
                twist.angular.z = 0.3 * np.sign(angle_diff)
            elif distance > 0.05:
                twist.linear.x = 0.1
                twist.angular.z = 0.0
            elif abs(self.normalize_angle(theta_goal - self.theta)) > 0.05:
                twist.linear.x = 0.0
                twist.angular.z = 0.3 * np.sign(theta_goal - self.theta)
            else:
                break

            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)

        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('Reached relative goal.')

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

def main(args=None):
    rclpy.init(args=args)
    node = MoveRelative()
    try:
        node.move_relative(0.5, 0.0, 90)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
