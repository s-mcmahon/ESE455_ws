#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header


class ObstacleDetection(Node):
    def __init__(self):
        super().__init__('obstacle_detection')

        # Grid parameters (map frame, fixed origin)
        self.resolution = 0.05   # meters per cell
        self.width = 400         # 20m × 20m map
        self.height = 400
        self.origin_x = -10.0    # map covers [-10, 10] in x
        self.origin_y = -10.0    # map covers [-10, 10] in y

        # Persistent occupancy grid
        self.grid = np.zeros((self.height, self.width), dtype=np.int8)

        # Robot pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # ROS 2 I/O
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/obstacle_grid', 10)

        self.get_logger().info(" ObstacleDetection started (map frame, persistent grid)")

    # ---- utilities ----
    def euler_from_quaternion(self, q):
        x, y, z, w = q.x, q.y, q.z, q.w
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return 0.0, 0.0, math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        _, _, self.theta = self.euler_from_quaternion(msg.pose.pose.orientation)

    def scan_callback(self, msg):
        # Reset grid for new scan frame (can comment out to accumulate obstacles)
        self.grid.fill(0)

        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or r < msg.range_min or r > msg.range_max:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            wx = self.x + r * math.cos(self.theta + angle)
            wy = self.y + r * math.sin(self.theta + angle)
            gx = int((wx - self.origin_x) / self.resolution)
            gy = int((wy - self.origin_y) / self.resolution)
            if 0 <= gx < self.width and 0 <= gy < self.height:
                self.grid[gy, gx] = 100

        msg_out = OccupancyGrid()
        msg_out.header = Header()
        msg_out.header.stamp = self.get_clock().now().to_msg()
        msg_out.header.frame_id = "map"
        msg_out.info.resolution = self.resolution
        msg_out.info.width = self.width
        msg_out.info.height = self.height
        msg_out.info.origin.position.x = self.origin_x
        msg_out.info.origin.position.y = self.origin_y
        msg_out.data = self.grid.flatten().tolist()

        self.map_pub.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
