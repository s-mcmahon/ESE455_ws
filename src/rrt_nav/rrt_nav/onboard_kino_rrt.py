#!/usr/bin/env python3

import math
import random
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Bool

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf2_ros import Buffer, TransformListener


# ------------ Angle helper ------------
def q_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class RRTPlannerNav2(Node):
    def __init__(self):
        super().__init__('rrt_planner_nav2')

        # ---------------- Parameters ----------------
        self.declare_parameter('robot_radius', 0.25)
        self.declare_parameter('safety_margin', 0.0)
        self.declare_parameter('front_buffer_extra', 0.1)
        self.declare_parameter('rear_buffer_extra', 0.0)
        self.declare_parameter('max_iters', 1000)
        self.declare_parameter('goal_bias', 0.5)
        self.declare_parameter('goal_radius', 0.3)  # be a bit generous
        self.declare_parameter('goal_x', 3.8)
        self.declare_parameter('goal_y', 3.8)
        self.declare_parameter('unknown_is_free', True)
        self.declare_parameter('v_max', 0.25)
        self.declare_parameter('w_max', 1.82)
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('step_time', 1.0)
        self.declare_parameter('controls_per_expand', 40)
        self.declare_parameter('steer_to_goal_prob', 0.6)
        self.declare_parameter('min_waypoint_separation', 0.6)
        self.declare_parameter('max_waypoints', 30)

        # Big global map span (meters) for our own fused map
        self.declare_parameter('global_span_x', 40.0)  # total width in meters
        self.declare_parameter('global_span_y', 40.0)  # total height in meters

        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        # ---------------- State ----------------
        # Original Nav2 map info
        self.resolution = 0.05
        self.map_w = self.map_h = 0
        self.map_ox = self.map_oy = 0.0

        # Big global grid we maintain ourselves
        self.global_grid = None          # np.int8, -1=unknown, 0=free, 1=occ
        self.global_w = 0
        self.global_h = 0
        self.global_ox = 0            # origin of big grid in world coords
        self.global_oy = 0
        self.global_initialized = False

        self.task_active = False
        self.success_announced = False

        self.last_plan_time = 0.0
        self.plan_cooldown = 3.0

        self.initial_origin_set = False
        self.local_origin = (0.0, 0.0)

        self.current_goal_world = None

        # ---------------- TF + Nav2 ----------------
        self.tf = Buffer(cache_time=Duration(seconds=10.0))
        self.tfl = TransformListener(self.tf, self)
        self.nav = BasicNavigator()

        # ---------------- Subscriptions / Pubs ----------------
        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        self.create_subscription(OccupancyGrid, '/map', self.on_map, map_qos)

        self.path_pub = self.create_publisher(Path, '/rrt_path', 10)
        self.waypoint_pub = self.create_publisher(Path, '/rrt_waypoints', 10)
        self.replan_pub = self.create_publisher(Bool, '/rrt_replan', 10)
        self.goal_reached_pub = self.create_publisher(Bool, '/rrt_goal_reached', 10)

        # NEW: publish our big global map so you can see it in RViz
        self.global_map_pub = self.create_publisher(OccupancyGrid, '/rrt_global_map', 1)

        self.create_timer(0.5, self.tick)
        self.create_timer(0.2, self.check_nav_status)
        self.create_timer(0.5, self.publish_global_map)
        self.create_timer(0.2, self.print_robot_pose)


        self.get_logger().info("Kinodynamic RRTPlannerNav2 with BIG GLOBAL MAP ready.")

    # ------------ Global grid helpers ------------

    def init_global_grid(self, map_msg: OccupancyGrid):
        """Initialize a large global grid that covers a big world area."""
        self.resolution = map_msg.info.resolution

        span_x = float(self.get_parameter('global_span_x').value)
        span_y = float(self.get_parameter('global_span_y').value)

        self.global_w = int(span_x / self.resolution)
        self.global_h = int(span_y / self.resolution)

        mx = map_msg.info.origin.position.x
        my = map_msg.info.origin.position.y

        # BIG FIX: Keep SAME origin as Nav2’s map
        self.global_ox = mx
        self.global_oy = my


        self.global_grid = np.full(
            (self.global_h, self.global_w), -1, dtype=np.int8
        )

        self.global_initialized = True

        self.get_logger().info(
            f"Initialized BIG global grid {self.global_w}x{self.global_h} "
            f"({span_x:.1f}m x {span_y:.1f}m) origin=({self.global_ox:.2f},{self.global_oy:.2f})"
        )

    def print_robot_pose(self):
        # Only print while following a Nav2 path
        if not self.task_active:
            return

        pose, frame = self.get_robot_pose()
        if pose is None:
            return

        x, y, th = pose
        self.get_logger().info(f"[ROBOT] x={x:.3f}, y={y:.3f}, th={th:.3f}")

    def world_to_global_ij(self, x, y):
        """World (x,y) → global grid indices (i,j)."""
        i = int((x - self.global_ox) / self.resolution)
        j = int((y - self.global_oy) / self.resolution)
        return i, j

    def inside_global(self, i, j):
        return 0 <= i < self.global_w and 0 <= j < self.global_h

    # ------------ Occupancy Grid handling ------------
    def on_map(self, msg: OccupancyGrid):
        # First time: initialize big global grid
        if not self.global_initialized:
            self.init_global_grid(msg)

        # Save local map info
        self.map_w = msg.info.width
        self.map_h = msg.info.height
        self.map_ox = msg.info.origin.position.x
        self.map_oy = msg.info.origin.position.y

        data = np.array(msg.data, dtype=np.int16).reshape(self.map_h, self.map_w)

        # Local grid: -1 unknown, 0 free, 1 occupied
        local_grid = np.zeros_like(data, dtype=np.int8)
        local_grid[data == -1] = -1
        local_grid[data >= 50] = 1
        local_grid[(data >= 0) & (data < 50)] = 0

        # Inflate obstacles by robot_radius + safety_margin
        rad = float(self.get_parameter('robot_radius').value) + \
              float(self.get_parameter('safety_margin').value)

        if rad > 0.0:
            r_cells = max(1, int(rad / self.resolution))
            occ = (local_grid == 1).astype(np.uint8)
            dil = occ.copy()

            for di in range(-r_cells, r_cells + 1):
                for dj in range(-r_cells, r_cells + 1):
                    if di * di + dj * dj <= r_cells * r_cells:
                        ys = slice(max(0, dj), self.map_h if dj >= 0 else self.map_h + dj)
                        xs = slice(max(0, di), self.map_w if di >= 0 else self.map_w + di)
                        ys2 = slice(max(0, -dj), self.map_h - max(0, dj))
                        xs2 = slice(max(0, -di), self.map_w - max(0, di))
                        dil[ys, xs] |= occ[ys2, xs2]

            local_grid[dil > 0] = 1

        # Fuse local inflated grid into BIG global grid
        for j in range(self.map_h):
            wy = self.map_oy + j * self.resolution
            for i in range(self.map_w):
                wx = self.map_ox + i * self.resolution
                gi, gj = self.world_to_global_ij(wx, wy)
                if not self.inside_global(gi, gj):
                    continue
                src = local_grid[j, i]
                dst = self.global_grid[gj, gi]

                # Priority: OCC (1) > FREE (0) > UNKNOWN (-1)
                if src == 1:
                    self.global_grid[gj, gi] = 1
                elif src == 0:
                    self.global_grid[gj, gi] = 0      # allow clearing
                elif src == -1:
                    if dst == -1:
                        self.global_grid[gj, gi] = 0
                # if src == -1, keep existing dst

        #self.get_logger().info("Map received and fused into BIG global grid.")

    # ------------ Global map visualization ------------
    def publish_global_map(self):
        if self.global_grid is None or not self.global_initialized:
            return

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.info.resolution = self.resolution
        msg.info.width = self.global_w
        msg.info.height = self.global_h
        msg.info.origin.position.x = self.global_ox
        msg.info.origin.position.y = self.global_oy
        msg.info.origin.orientation.w = 1.0

        flat = []
        for j in range(self.global_h):
            for i in range(self.global_w):
                v = self.global_grid[j, i]
                if v == 1:
                    flat.append(100)
                elif v == 0:
                    flat.append(0)
                else:
                    flat.append(-1)

        msg.data = flat
        self.global_map_pub.publish(msg)

    # ------------ Helpers ------------
    def is_free_xy(self, x, y) -> bool:
        """Collision check using BIG global grid."""
        if self.global_grid is None:
            return False

        gi, gj = self.world_to_global_ij(x, y)
        if not self.inside_global(gi, gj):
            # Outside our big global grid: treat as unknown
            return bool(self.get_parameter('unknown_is_free').value)

        v = self.global_grid[gj, gi]
        if v == 1:
            return False
        if v == -1:
            return bool(self.get_parameter('unknown_is_free').value)
        return True

    def project_to_free(self, x, y, max_r=1.0):
        """Find nearest free cell in global grid around (x,y)."""
        step = self.resolution
        for r in np.arange(0.0, max_r + 1e-6, step):
            for th in np.linspace(0, 2 * math.pi, 48):
                nx = x + r * math.cos(th)
                ny = y + r * math.sin(th)
                if self.is_free_xy(nx, ny):
                    return (nx, ny)
        return None

    # ------------ TF helper ------------
    def get_robot_pose(self):
        try:
            t = self.tf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except Exception:
            try:
                t = self.tf.lookup_transform('map', 'base_link', rclpy.time.Time())
            except Exception:
                return None, None

        x = float(t.transform.translation.x)
        y = float(t.transform.translation.y)
        q = t.transform.rotation

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        th = math.atan2(siny_cosp, cosy_cosp)
        return (x, y, th), 'map'

    # ------------ Kinodynamic RRT core (local frame) ------------
    def kinodynamic_rrt_once(
        self, start, goal, local_origin,
        step_time=None, dt=None, max_iters=None, goal_bias=None
    ):
        origin_x, origin_y = local_origin

        if step_time is None:
            step_time = float(self.get_parameter('step_time').value)
        if dt is None:
            dt = float(self.get_parameter('dt').value)
        if max_iters is None:
            max_iters = int(self.get_parameter('max_iters').value)
        if goal_bias is None:
            goal_bias = float(self.get_parameter('goal_bias').value)

        v_max = float(self.get_parameter('v_max').value)
        w_max = float(self.get_parameter('w_max').value)
        goal_radius = float(self.get_parameter('goal_radius').value)
        controls_per_expand = int(self.get_parameter('controls_per_expand').value)
        steer_p = float(self.get_parameter('steer_to_goal_prob').value)

        front_extra = float(self.get_parameter('front_buffer_extra').value)
        rear_extra = float(self.get_parameter('rear_buffer_extra').value)

        nodes = [start]
        parents = {start: None}

        for _ in range(max_iters):

            # ---- Sample ----
            if random.random() < goal_bias:
                x_rand, y_rand, th_rand = goal
            else:
                # Sample ANYWHERE in the BIG global grid
                xw = random.uniform(self.global_ox, self.global_ox + self.global_w * self.resolution)
                yw = random.uniform(self.global_oy, self.global_oy + self.global_h * self.resolution)
                x_rand = xw - origin_x
                y_rand = yw - origin_y
                th_rand = random.uniform(-math.pi, math.pi)

            if not self.is_free_xy(x_rand + origin_x, y_rand + origin_y):
                continue

            # ---- Nearest neighbor ----
            nearest = min(nodes, key=lambda n: math.hypot(n[0] - x_rand, n[1] - y_rand))

            best_new = None
            min_dist = float('inf')

            for _ in range(controls_per_expand):
                dx = x_rand - nearest[0]
                dy = y_rand - nearest[1]
                desired_yaw = math.atan2(dy, dx)

                yaw_err = (desired_yaw - nearest[2] + math.pi) % (2 * math.pi) - math.pi

                if random.random() < steer_p:
                    # steer toward the sample
                    w = max(-w_max, min(w_max, yaw_err / step_time))
                    u = 0.8 * v_max if abs(yaw_err) < math.pi / 3 else 0.4 * v_max
                else:
                    # bias toward big forward moves
                    if random.random() < 0.8:
                        u = random.uniform(0.7 * v_max, v_max)
                    else:
                        u = random.uniform(0.0, v_max)
                    # bias to mostly-straight
                    if random.random() < 0.7:
                        w = random.uniform(-0.4 * w_max, 0.4 * w_max)
                    else:
                        w = random.uniform(-w_max, w_max)

                x, y, th = nearest
                t = 0.0
                valid = True

                # Rollout with directional extra buffer
                while t < step_time:
                    x += u * math.cos(th) * dt
                    y += u * math.sin(th) * dt
                    th += w * dt

                    # Basic collision check
                    if not self.is_free_xy(x + origin_x, y + origin_y):
                        valid = False
                        break

                    # Extra buffer in front
                    if u > 0.0 and front_extra > 0.0:
                        xf = x + front_extra * math.cos(th)
                        yf = y + front_extra * math.sin(th)
                        if not self.is_free_xy(xf + origin_x, yf + origin_y):
                            valid = False
                            break

                    # Optional extra buffer behind
                    if u < 0.0 and rear_extra > 0.0:
                        xb = x - rear_extra * math.cos(th)
                        yb = y - rear_extra * math.sin(th)
                        if not self.is_free_xy(xb + origin_x, yb + origin_y):
                            valid = False
                            break

                    t += dt

                if not valid:
                    continue

                dist = math.hypot(x - x_rand, y - y_rand)
                if dist < min_dist:
                    min_dist = dist
                    best_new = (x, y, th)

            if best_new is None:
                continue

            nodes.append(best_new)
            parents[best_new] = nearest

            if math.hypot(best_new[0] - goal[0], best_new[1] - goal[1]) < goal_radius:
                path = [best_new]
                cur = best_new
                while parents[cur] is not None:
                    cur = parents[cur]
                    path.append(cur)
                return list(reversed(path))

        return []

    # ------------ Path helpers ------------
    def downsample_path(self, pts, min_sep=0.5):
        if not pts:
            return []
        new_pts = [pts[0]]
        for p in pts[1:]:
            if math.hypot(p[0] - new_pts[-1][0], p[1] - new_pts[-1][1]) >= min_sep:
                new_pts.append(p)
        if new_pts[-1] != pts[-1]:
            new_pts.append(pts[-1])
        return new_pts

    def publish_path(self, pts):
        if len(pts) < 2:
            return [], []

        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        poses = []
        for (x, y, th) in pts:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.header.stamp = msg.header.stamp
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation = q_from_yaw(th)
            poses.append(ps)

        msg.poses = poses
        self.path_pub.publish(msg)
        self.waypoint_pub.publish(msg)
        self.replan_pub.publish(Bool(data=True))

        return poses, pts

    # ------------ Main periodic tick ------------
    def tick(self):
        # Need global map and not already done
        if not self.global_initialized or self.success_announced:
            return

        if self.task_active:
            return

        pose, parent_frame = self.get_robot_pose()
        if pose is None:
            self.get_logger().info("Waiting for TF in map frame...")
            return

        x, y, th = pose

        # Project current robot pose into free space
        sproj = self.project_to_free(x, y, max_r=0.5)
        if sproj is None:
            self.get_logger().warn("Start in collision.")
            return

        start_world = (sproj[0], sproj[1], th)

        # Set local origin ONCE
        if not self.initial_origin_set:
            self.local_origin = (start_world[0], start_world[1])
            self.initial_origin_set = True
            self.get_logger().info(
                f"Local RRT frame origin set at world ({start_world[0]:.2f}, {start_world[1]:.2f})"
            )

        origin_x, origin_y = self.local_origin

        # Convert start to local frame
        start_local = (
            start_world[0] - origin_x,
            start_world[1] - origin_y,
            start_world[2],
        )

        # ---- Goal in world frame (ORIGINAL, NEVER CLIPPED) ----
        gx = float(self.get_parameter('goal_x').value)
        gy = float(self.get_parameter('goal_y').value)
        goal_world_raw = (gx, gy, 0.0)
        self.current_goal_world = goal_world_raw

        # We DO NOT project the goal to free space; we plan toward the real one.
        goal_world = goal_world_raw

        # Convert goal to local frame
        goal_local = (
            goal_world[0] - origin_x,
            goal_world[1] - origin_y,
            0.0,
        )

        now = time.time()
        if now - self.last_plan_time < self.plan_cooldown:
            return
        self.last_plan_time = now

        self.get_logger().info(
            f"Planning in LOCAL frame: start=({start_local[0]:.2f},{start_local[1]:.2f}) "
            f"goal=({goal_local[0]:.2f},{goal_local[1]:.2f}) "
        )

        path_local = self.kinodynamic_rrt_once(
            start=start_local,
            goal=goal_local,
            local_origin=self.local_origin
        )

        if len(path_local) < 2:
            self.get_logger().warn("No path found.")
            return

        # Convert back to world frame
        path_world = [
            (px + origin_x, py + origin_y, th)
            for (px, py, th) in path_local
        ]

        path_ds = self.downsample_path(path_world)
        poses, _ = self.publish_path(path_ds)

        if not poses:
            self.get_logger().warn("No poses to send to Nav2.")
            return

        # Send to Nav2
        if len(poses) <= 2:
            self.nav.goToPose(poses[-1])
        else:
            self.nav.followWaypoints(poses)

        self.task_active = True

    # ------------ Nav2 status monitoring ------------
    def check_nav_status(self):
        if not self.task_active or self.success_announced:
            return

        if not self.nav.isTaskComplete():
            return

        result = self.nav.getResult()
        self.task_active = False

        goal_radius = float(self.get_parameter('goal_radius').value)
        close_enough = False

        try:
            trot = self.tf.lookup_transform("odom", "base_footprint", rclpy.time.Time())
            rx = trot.transform.translation.x
            ry = trot.transform.translation.y

            gx, gy, _ = self.current_goal_world

            goal_ps = PoseStamped()
            goal_ps.header.frame_id = "map"
            goal_ps.pose.position.x = gx
            goal_ps.pose.position.y = gy
            goal_odom = self.tf.transform(goal_ps, "odom")

            gx_o = goal_odom.pose.position.x
            gy_o = goal_odom.pose.position.y

            dist = math.hypot(gx_o - rx, gy_o - ry)
            self.get_logger().info(f"[ODOM dist] {dist:.3f}")
            close_enough = dist < goal_radius

        except Exception as e:
            self.get_logger().warn(f"Transform error in odom distance check: {e}")
            close_enough = False

        if result == TaskResult.SUCCEEDED or close_enough:
            self.get_logger().info(f"Goal reached (Nav2 result={result}). Stopping.")
            self.announce_success()
        else:
            self.get_logger().warn(f"Nav2 failed with result={result}. Goal unreachable, stopping.")
            try:
                self.nav.cancelTask()
            except Exception:
                pass
            self.success_announced = True

    # ------------ Goal reached handling ------------
    def announce_success(self):
        if self.success_announced:
            return
        self.success_announced = True
        self.goal_reached_pub.publish(Bool(data=True))

        try:
            self.nav.cancelTask()
        except Exception:
            pass

        time.sleep(0.5)
        self.destroy_node()
        rclpy.shutdown()
        import sys
        sys.exit(0)


def main():
    rclpy.init()
    executor = rclpy.executors.MultiThreadedExecutor()
    node = RRTPlannerNav2()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()