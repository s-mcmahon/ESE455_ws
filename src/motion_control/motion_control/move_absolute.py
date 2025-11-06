#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class MoveAbsolute(Node):
    def __init__(self):
        super().__init__('move_absolute')

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Motion tuning
        self.dist_tol = 0.25   # allow 25 cm tolerance
        self.angle_tol = 0.2   # allow ~11° tolerance

        self.max_lin = 0.3
        self.max_ang = 0.8

        self.get_logger().info("MoveAbsolute (simple) initialized.")

    def euler_from_quaternion(self, q):
        x, y, z, w = q.x, q.y, q.z, q.w
        siny_cosp = 2 * (w*z + x*y)
        cosy_cosp = 1 - 2 * (y*y + z*z)
        return 0.0, 0.0, math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        _, _, self.theta = self.euler_from_quaternion(msg.pose.pose.orientation)

    def normalize_angle(self, a):
        return math.atan2(math.sin(a), math.cos(a))

    def move_absolute(self, x_goal, y_goal, theta_goal_deg):
        theta_goal = math.radians(theta_goal_deg)
        twist = Twist()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            dx, dy = x_goal - self.x, y_goal - self.y
            distance = math.sqrt(dx**2 + dy**2)
            target_angle = math.atan2(dy, dx)
            angle_diff = self.normalize_angle(target_angle - self.theta)

            if distance < self.dist_tol and abs(self.normalize_angle(theta_goal - self.theta)) < self.angle_tol:
                break

            if abs(angle_diff) > self.angle_tol:
                twist.linear.x = 0.0
                twist.angular.z = np.clip(2.0 * angle_diff, -self.max_ang, self.max_ang)
            else:
                twist.angular.z = 0.0
                twist.linear.x = np.clip(1.0 * distance, 0.05, self.max_lin)

            self.cmd_pub.publish(twist)

        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        self.get_logger().info(f"Reached ({x_goal:.2f}, {y_goal:.2f})")


def main(args=None):
    rclpy.init(args=args)
    node = MoveAbsolute()
    try:
        node.move_absolute(1.0, 0.5, 45)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy