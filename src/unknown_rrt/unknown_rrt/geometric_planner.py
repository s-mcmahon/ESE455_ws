#!/usr/bin/env python3
import math
import random
import time
import numpy as np
import re
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid
from motion_control.move_absolute import MoveAbsolute


def dynamic_steer(x, y, th, v, w, dt, steps):
    path = []
    def sinc(x):
        return 1.0 if abs(x) < 1e-6 else math.sin(x) / x
    for _ in range(steps):
        a = w * dt / 2.0
        s = sinc(a)
        x += v * dt * s * math.cos(th + a)
        y += v * dt * s * math.sin(th + a)
        th = ((th + w * dt + math.pi) % (2 * math.pi)) - math.pi
        path.append((x, y))
    return (x, y, th), path


class RRTPlanner(Node):
    def __init__(self):
        super().__init__('rrt_planner')
        # ---- parameters ----
        self.robot_radius = 0.20
        self.safety_margin = 0.0
        self.step_size = 0.25
        self.max_iters = 600
        self.goal_bias = 0.25
        self.goal_radius = 0.5
        self.start = (-1.5, -2.0)
        self.goal_xy = (-2.4, 3.7)
        self.sample_xmin, self.sample_xmax = -10.0, 10.0
        self.sample_ymin, self.sample_ymax = -10.0, 10.0

        # dynamics controls (forward + backward)
        self.controls = [(v, w) for v in [0.5, -0.5] for w in np.linspace(-1.0, 1.0, 7)]
        self.tau = 0.3
        self.steer_steps = 10

        # state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # map
        self.map_ready = False
        self.grid = None
        self.resolution = 0.1
        self.width = 0
        self.height = 0
        self.origin_x = 0.0
        self.origin_y = 0.0

        # subscribers
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(OccupancyGrid, '/obstacle_grid', self.grid_callback, 10)

        self.get_logger().info("RRT (dynamics) planner initialized — obstacle grid + world support.")

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

    def grid_to_world(self, gx, gy):
        wx = gx * self.resolution + self.origin_x
        wy = gy * self.resolution + self.origin_y
        return (wx, wy)

    def is_cell_occupied(self, gx, gy):
        if self.grid is None:
            return False
        if gx < 0 or gx >= self.width or gy < 0 or gy >= self.height:
            return True
        return self.grid[gy, gx] >= 50

    def is_point_occupied(self, xw, yw):
        gx, gy = self.world_to_grid(xw, yw)
        return self.is_cell_occupied(gx, gy)

    def path_collision_free(self, pts):
        for (x, y) in pts:
            if self.is_point_occupied(x, y):
                return False
        return True

    def inflate_binary(self, occ_bin, r_cells):
        H, W = occ_bin.shape
        out = occ_bin.copy()
        offsets = []
        for dy in range(-r_cells, r_cells + 1):
            max_dx = int((r_cells**2 - dy**2) ** 0.5)
            for dx in range(-max_dx, max_dx + 1):
                offsets.append((dy, dx))
        for (dy, dx) in offsets:
            y0_src = max(0, -dy); y1_src = min(H, H - dy)
            x0_src = max(0, -dx); x1_src = min(W, W - dx)
            y0_dst = max(0,  dy); y1_dst = min(H, H + dy)
            x0_dst = max(0,  dx); x1_dst = min(W, W + dx)
            if y1_src - y0_src <= 0 or x1_src - x0_src <= 0:
                continue
            out[y0_dst:y1_dst, x0_dst:x1_dst] = np.maximum(
                out[y0_dst:y1_dst, x0_dst:x1_dst],
                occ_bin[y0_src:y1_src, x0_src:x1_src]
            )
        return out

    def find_nearest_free_point(self, x, y, max_radius=5.0, step=0.1):
        if self.grid is None:
            return (x, y)

        gx, gy = self.world_to_grid(x, y)
        h, w = self.grid.shape

        for r in np.arange(0.0, max_radius, step):
            for dx in np.linspace(-r, r, int(2 * r / step) + 1):
                for dy in np.linspace(-r, r, int(2 * r / step) + 1):
                    nx, ny = gx + int(dx / self.resolution), gy + int(dy / self.resolution)
                    if 0 <= nx < w and 0 <= ny < h and self.grid[ny, nx] == 0:
                        return self.grid_to_world(nx, ny)
        bx = x - 0.5 * math.cos(self.theta)
        by = y - 0.5 * math.sin(self.theta)
        self.get_logger().warn(f"⚠No free cell within {max_radius}m — forcing start to ({bx:.2f},{by:.2f})")
        return (bx, by)

    def postprocess_path(self, path, min_lead=0.20, spacing=0.30):
        if not path:
            return []
        cx, cy = self.x, self.y
        i = 0
        n = len(path)
        while i < n and math.hypot(path[i][0] - cx, path[i][1] - cy) < min_lead:
            i += 1
        path = path[i:] if i < n else []

        if not path:
            return []

        out = [path[0]]
        for p in path[1:]:
            if math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) >= spacing:
                out.append(p)

        if out and (out[-1] != path[-1]):
            out.append(path[-1])
        return out

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
    def reconstruct_path_dyn(self, parents, edges, last):
        chain = []
        node = last
        while (node in parents) and (parents[node] is not None):
            seg = edges.get(node)
            if not seg:
                break
            chain.append(seg)
            node = parents[node]
        chain.reverse()
        out = [pt for seg in chain for pt in seg]
        if not out:
            return []
        pruned = [out[0]]
        for p in out[1:]:
            if math.hypot(p[0] - pruned[-1][0], p[1] - pruned[-1][1]) > 0.05:
                pruned.append(p)
        return pruned

    def rrt_once_dyn(self, start_pose, goal_xy, time_budget=2.0):
        nodes = [start_pose]
        parents = {start_pose: None}
        edges = {}
        t0 = time.time()

        for _ in range(self.max_iters):
            if time.time() - t0 > time_budget:
                return []

            if random.random() < self.goal_bias:
                q_rand = (goal_xy[0], goal_xy[1], random.uniform(-math.pi, math.pi))
            else:
                q_rand = (random.uniform(self.sample_xmin, self.sample_xmax),
                          random.uniform(self.sample_ymin, self.sample_ymax),
                          random.uniform(-math.pi, math.pi))

            nearest = min(nodes, key=lambda n: (n[0] - q_rand[0])**2 + (n[1] - q_rand[1])**2)

            best_child, best_path, best_dist = None, None, float('inf')
            for (v, w) in self.controls:
                (x2, y2, th2), pts = dynamic_steer(nearest[0], nearest[1], nearest[2], v, w, self.tau, self.steer_steps)
                if not (self.sample_xmin - 1.0 <= x2 <= self.sample_xmax + 1.0 and
                        self.sample_ymin - 1.0 <= y2 <= self.sample_ymax + 1.0):
                    continue
                if not self.path_collision_free(pts):
                    continue
                d = (x2 - q_rand[0])**2 + (y2 - q_rand[1])**2
                if d < best_dist:
                    best_dist = d
                    best_child = (x2, y2, th2)
                    best_path = pts

            if best_child is None:
                continue

            nodes.append(best_child)
            parents[best_child] = nearest
            edges[best_child] = best_path

            if math.hypot(best_child[0] - goal_xy[0], best_child[1] - goal_xy[1]) < self.goal_radius:
                return self.reconstruct_path_dyn(parents, edges, best_child)
        return []

    # ------------------- Main Loop -------------------
    def run(self, world_path=None):
        mover = MoveAbsolute()
        try:
            if world_path:
                start, goal, size = self.load_world_info(world_path)
                if start:
                    self.start = start
                if goal:
                    self.goal_xy = goal
                if size:
                    self.sample_xmin, self.sample_xmax = -size[0]/2, size[0]/2
                    self.sample_ymin, self.sample_ymax = -size[1]/2, size[1]/2
                self.get_logger().info(f"Loaded world: {world_path} -> Start={self.start}, Goal={self.goal_xy}, Size={size}")

            if not self.map_ready:
                self.get_logger().warn("Waiting for /obstacle_grid ...")
                while rclpy.ok() and not self.map_ready:
                    rclpy.spin_once(self, timeout_sec=0.1)

            last_position = (float('inf'), float('inf'))
            MIN_MOVE_DIST = 0.10

            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.3)

                # goal detection using odometry
                if math.hypot(self.x - self.goal_xy[0], self.y - self.goal_xy[1]) < self.goal_radius:
                    self.get_logger().info(" Reached goal region! Executing final approach.")
                    mover.move_absolute(self.goal_xy[0], self.goal_xy[1], math.degrees(self.theta))
                    break

                raw_start = (self.x, self.y, self.theta)
                sx, sy = self.find_nearest_free_point(raw_start[0], raw_start[1], max_radius=3.0, step=0.05)
                if (sx, sy) == (raw_start[0], raw_start[1]):
                    self.get_logger().warn("Still inside inflation — forcing outward offset")
                    sx += 0.3 * math.cos(self.theta)
                    sy += 0.3 * math.sin(self.theta)
                start_pose = (sx, sy, raw_start[2])

                if math.hypot(sx - last_position[0], sy - last_position[1]) < MIN_MOVE_DIST:
                    rclpy.spin_once(self, timeout_sec=0.2)
                    continue

                self.get_logger().info("Planning (time budget 2.0s)...")
                t0 = time.time()
                raw_path = self.rrt_once_dyn(start_pose, self.goal_xy, time_budget=2.0)
                dt = time.time() - t0
                self.get_logger().info(f"Planning finished in {dt:.2f}s: {'found path' if raw_path else 'no path'}")

                if not raw_path:
                    continue

                path = self.postprocess_path(raw_path, min_lead=0.20, spacing=0.30)
                if not path:
                    self.get_logger().warn("Postprocessed path empty — replanning.")
                    continue

                for i, (nx, ny) in enumerate(path):
                    dist_to_target = math.hypot(nx - self.x, ny - self.y)
                    if dist_to_target < 0.25:
                        continue

                    heading_deg = math.degrees(math.atan2(ny - self.y, nx - self.x))
                    self.get_logger().info(f"Waypoint {i+1}/{len(path)}: ({nx:.2f}, {ny:.2f}), dist={dist_to_target:.2f}")

                    ok = mover.move_absolute(nx, ny, heading_deg)
                    if not ok or math.hypot(self.x - nx, self.y - ny) < 0.15:
                        self.get_logger().warn("Insufficient progress — skipping and continuing.")
                        continue

                    rclpy.spin_once(self, timeout_sec=0.4)

                last_position = (self.x, self.y)
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
