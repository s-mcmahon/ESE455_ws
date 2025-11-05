#!/usr/bin/env python3
import math
import random
import numpy as np
import re
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid
from motion_control.move_absolute import MoveAbsolute


class RRTPlanner(Node):
    def __init__(self):
        super().__init__('rrt_planner')

        # -------- Parameters --------
        self.robot_radius = 0.20
        self.safety_margin = 0.10
        self.step_size = 0.25
        self.max_iters = 600
        self.goal_bias = 0.25
        self.goal_radius = 0.50

        self.start = (-1.5, -2.0)
        self.goal = (-2.4, 3.7)
        self.sample_xmin, self.sample_xmax = -10.0, 10.0
        self.sample_ymin, self.sample_ymax = -10.0, 10.0

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.map_ready = False
        self.grid = None
        self.resolution = 0.1
        self.width = 0
        self.height = 0
        self.origin_x = 0.0
        self.origin_y = 0.0

        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(OccupancyGrid, '/obstacle_grid', self.grid_callback, 10)

        self.get_logger().info("RRT Planner initialized with obstacle grid + world file support.")

    # ------------------- ROS Callbacks -------------------
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        _, _, self.theta = self.euler_from_quaternion(msg.pose.pose.orientation)

    def grid_callback(self, msg: OccupancyGrid):
        self.resolution = msg.info.resolution
        self.width = msg.info.width
        self.height = msg.info.height
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y

        data = np.array(msg.data, dtype=np.int16).reshape((self.height, self.width))
        data[data < 0] = 0
        inflate_m = self.robot_radius + self.safety_margin
        inflate_cells = max(1, int(math.ceil(inflate_m / self.resolution)))

        occ = (data >= 50).astype(np.uint8)
        occ_infl = self.inflate_binary(occ, inflate_cells)
        self.grid = (occ_infl * 100).astype(np.int8)
        self.map_ready = True

        self.sample_xmin = self.origin_x
        self.sample_xmax = self.origin_x + self.width * self.resolution
        self.sample_ymin = self.origin_y
        self.sample_ymax = self.origin_y + self.height * self.resolution

    # ------------------- Utilities -------------------
    def euler_from_quaternion(self, q):
        x, y, z, w = q.x, q.y, q.z, q.w
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return 0.0, 0.0, math.atan2(siny_cosp, cosy_cosp)

    def world_to_grid(self, xw, yw):
        gx = int((xw - self.origin_x) / self.resolution)
        gy = int((yw - self.origin_y) / self.resolution)
        return gx, gy

    def is_cell_occupied(self, gx, gy):
        if self.grid is None:
            return False
        if gx < 0 or gx >= self.width or gy < 0 or gy >= self.height:
            return True
        return self.grid[gy, gx] >= 50

    def is_point_occupied(self, xw, yw):
        gx, gy = self.world_to_grid(xw, yw)
        return self.is_cell_occupied(gx, gy)

    def line_collision_free(self, p1, p2):
        if not self.map_ready:
            return True
        x1, y1 = p1
        x2, y2 = p2
        dist = math.hypot(x2 - x1, y2 - y1)
        if dist < 1e-6:
            return not self.is_point_occupied(x1, y1)

        step = max(self.resolution * 0.5, 0.02)
        n = int(math.ceil(dist / step))
        for i in range(n + 1):
            t = i / max(n, 1)
            xt = x1 + t * (x2 - x1)
            yt = y1 + t * (y2 - y1)
            if self.is_point_occupied(xt, yt):
                return False
        return True

    def inflate_binary(self, occ_bin, r_cells):
        H, W = occ_bin.shape
        out = occ_bin.copy()
        ii = np.cumsum(np.cumsum(occ_bin, axis=0), axis=1)

        def rect_sum(x0, y0, x1, y1):
            s = ii[y1, x1]
            if x0 > 0:
                s -= ii[y1, x0 - 1]
            if y0 > 0:
                s -= ii[y0 - 1, x1]
            if x0 > 0 and y0 > 0:
                s += ii[y0 - 1, x0 - 1]
            return s

        for gy in range(H):
            y0 = max(0, gy - r_cells)
            y1 = min(H - 1, gy + r_cells)
            for gx in range(W):
                x0 = max(0, gx - r_cells)
                x1 = min(W - 1, gx + r_cells)
                if rect_sum(x0, y0, x1, y1) > 0:
                    out[gy, gx] = 1
        return out

    def find_nearest_free_point(self, x, y, max_radius=1.0, step=0.05):
        """If robot starts in inflation, find nearest free cell."""
        if not self.map_ready:
            return (x, y)
        if not self.is_point_occupied(x, y):
            return (x, y)
        for r in np.arange(step, max_radius + step, step):
            for angle in np.linspace(0, 2 * math.pi, int(2 * math.pi * r / step)):
                xt = x + r * math.cos(angle)
                yt = y + r * math.sin(angle)
                if not self.is_point_occupied(xt, yt):
                    self.get_logger().warn(
                        f"Start in obstacle inflation — shifting to free point ({xt:.2f}, {yt:.2f})"
                    )
                    return (xt, yt)
        self.get_logger().error("No free start location found within radius!")
        return (x, y)

    # ------------------- World Loader -------------------
    def load_world_info(self, world_path):
        start = goal = None
        size = (10.0, 10.0)
        with open(world_path, 'r') as f:
            text = f.read()
        start_match = re.search(r'<model name="start_marker">.*?<pose>(.*?)</pose>', text, re.DOTALL)
        goal_match = re.search(r'<model name="goal_region">.*?<pose>(.*?)</pose>', text, re.DOTALL)
        size_match = re.search(r'<size>(.*?)</size>', text, re.DOTALL)
        if start_match:
            vals = [float(v) for v in start_match.group(1).split()]
            start = (vals[0], vals[1])
        if goal_match:
            vals = [float(v) for v in goal_match.group(1).split()]
            goal = (vals[0], vals[1])
        if size_match:
            vals = [float(v) for v in size_match.group(1).split()]
            size = (vals[0], vals[1])
        return start, goal, size

    # ------------------- RRT Core -------------------
    def rrt_once(self, start, goal):
        nodes = [start]
        parents = {start: None}
        for _ in range(self.max_iters):
            rand = goal if random.random() < self.goal_bias else (
                random.uniform(self.sample_xmin, self.sample_xmax),
                random.uniform(self.sample_ymin, self.sample_ymax)
            )
            nearest = min(nodes, key=lambda n: np.linalg.norm(np.array(n) - np.array(rand)))
            direction = np.array(rand) - np.array(nearest)
            norm = np.linalg.norm(direction)
            if norm < 1e-6:
                continue
            new_node = tuple(np.array(nearest) + (self.step_size / norm) * direction)
            if not self.line_collision_free(nearest, new_node):
                continue
            nodes.append(new_node)
            parents[new_node] = nearest
            if np.linalg.norm(np.array(new_node) - np.array(goal)) < self.goal_radius:
                return self.reconstruct_path(parents, new_node)
        return []

    def reconstruct_path(self, parents, node):
        path = [node]
        while parents[node] is not None:
            node = parents[node]
            path.append(node)
        return list(reversed(path))

    # ------------------- Main Loop -------------------
    def run(self, world_path=None):
        mover = MoveAbsolute()
        try:
            # Load start/goal from world
            if world_path:
                start, goal, size = self.load_world_info(world_path)
                if start:
                    self.start = start
                if goal:
                    self.goal = goal
                if size:
                    self.sample_xmin, self.sample_xmax = -size[0] / 2, size[0] / 2
                    self.sample_ymin, self.sample_ymax = -size[1] / 2, size[1] / 2
                self.get_logger().info(f"Loaded world file: {world_path}")
                self.get_logger().info(f"Start={self.start}, Goal={self.goal}, Size={size}")

            # Wait for map
            if not self.map_ready:
                self.get_logger().warn("Waiting for /obstacle_grid ...")
                while rclpy.ok() and not self.map_ready:
                    rclpy.spin_once(self, timeout_sec=0.1)

            # Planning loop
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.2)

                raw_start = (self.x, self.y)
                start = self.find_nearest_free_point(*raw_start, max_radius=1.0)
                if np.linalg.norm(np.array(start) - np.array(self.goal)) < self.goal_radius:
                    self.get_logger().info("Reached goal region!")
                    break

                path = self.rrt_once(start, self.goal)
                if not path or len(path) < 2:
                    self.get_logger().warn("No path found — retrying...")
                    continue

                for i in range(1, len(path)):
                    nx, ny = path[i]
                    heading_deg = math.degrees(math.atan2(ny - self.y, nx - self.x))
                    self.get_logger().info(f"Waypoint {i}/{len(path)-1}: ({nx:.2f}, {ny:.2f})")

                    ok = mover.move_absolute(nx, ny, heading_deg)
                    if not ok:
                        self.get_logger().warn("Movement aborted — replanning.")
                        break
        finally:
            mover.destroy_node()


def main(args=None):
    import sys
    rclpy.init(args=args)
    node = RRTPlanner()

    world_path = sys.argv[1] if len(sys.argv) > 1 else None
    node.run(world_path)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
