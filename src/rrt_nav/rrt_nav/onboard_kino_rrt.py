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
        self.declare_parameter('robot_radius', 0.22)
        self.declare_parameter('safety_margin', 0.1)
        self.declare_parameter('front_buffer_extra', 0.1)
        self.declare_parameter('rear_buffer_extra', 0.0)
        self.declare_parameter('max_iters', 2000)
        self.declare_parameter('goal_bias', 0.35)
        self.declare_parameter('goal_radius', 0.05)
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
        self.declare_parameter('max_waypoints', 20)

        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        # ---------------- State ----------------
        self.grid = None
        self.resolution = 0.05
        self.w = self.h = 0
        self.ox = self.oy = 0.0

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

        self.create_timer(0.5, self.tick)
        self.create_timer(0.2, self.check_nav_status)

        self.get_logger().info("Kinodynamic RRTPlannerNav2 ready.")

    # ------------ Occupancy Grid handling ------------
    def on_map(self, msg: OccupancyGrid):
        self.resolution = msg.info.resolution
        self.w = msg.info.width
        self.h = msg.info.height
        self.ox = msg.info.origin.position.x
        self.oy = msg.info.origin.position.y

        data = np.array(msg.data, dtype=np.int16).reshape(self.h, self.w)

        # Create grid: 0=free, 1=occupied, -1=unknown
        grid = np.zeros_like(data, dtype=np.int8)
        grid[data == -1] = -1
        grid[data >= 50] = 1

        # Inflate obstacles by robot_radius + safety_margin
        rad = float(self.get_parameter('robot_radius').value) + \
              float(self.get_parameter('safety_margin').value)

        if rad > 0.0:
            r_cells = max(1, int(rad / self.resolution))
            occ = (grid == 1).astype(np.uint8)
            dil = occ.copy()

            for di in range(-r_cells, r_cells + 1):
                for dj in range(-r_cells, r_cells + 1):
                    if di * di + dj * dj <= r_cells * r_cells:
                        ys = slice(max(0, dj), self.h if dj >= 0 else self.h + dj)
                        xs = slice(max(0, di), self.w if di >= 0 else self.w + di)
                        ys2 = slice(max(0, -dj), self.h - max(0, dj))
                        xs2 = slice(max(0, -di), self.w - max(0, di))
                        dil[ys, xs] |= occ[ys2, xs2]

            grid[dil > 0] = 1

        self.grid = grid
        self.get_logger().info("Map received.")

    # ------------ Helpers ------------
    def world_to_grid(self, x, y):
        i = int((x - self.ox) / self.resolution)
        j = int((y - self.oy) / self.resolution)
        return i, j

    def inside(self, i, j):
        return 0 <= i < self.w and 0 <= j < self.h

    def is_free_xy(self, x, y) -> bool:
        if self.grid is None:
            return False
        i, j = self.world_to_grid(x, y)
        if not self.inside(i, j):
            return False
        v = self.grid[j, i]
        if v == 1:
            return False
        if v == -1:
            return bool(self.get_parameter('unknown_is_free').value)
        return True

    def project_to_free(self, x, y, max_r=1.0):
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
                xw = random.uniform(self.ox, self.ox + self.w * self.resolution)
                yw = random.uniform(self.oy, self.oy + self.h * self.resolution)
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
                    w = max(-w_max, min(w_max, yaw_err / step_time))
                    u = 0.8 * v_max if abs(yaw_err) < math.pi / 3 else 0.4 * v_max
                else:
                    u = random.uniform(0, v_max)
                    w = random.uniform(-w_max, w_max)

                x, y, th = nearest
                t = 0.0
                valid = True

                # Rollout with directional extra buffer
                while t < step_time:
                    x += u * math.cos(th) * dt
                    y += u * math.sin(th) * dt
                    th += w * dt

                    # Basic collision check at robot center
                    if not self.is_free_xy(x + origin_x, y + origin_y):
                        valid = False
                        break

                    # Extra buffer in front if moving forward (0–180 deg sector)
                    if u > 0.0 and front_extra > 0.0:
                        xf = x + front_extra * math.cos(th)
                        yf = y + front_extra * math.sin(th)
                        if not self.is_free_xy(xf + origin_x, yf + origin_y):
                            valid = False
                            break

                    # Optional: extra buffer behind when reversing
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
        # No map or already finished (success or failure)
        if self.grid is None or self.success_announced:
            return

        # If Nav2 is currently executing a task, don't replan
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

        # ---- Set local origin ONCE ----
        if not self.initial_origin_set:
            self.local_origin = (start_world[0], start_world[1])
            self.initial_origin_set = True
            self.get_logger().info(
                f"Local RRT frame origin set at world ({start_world[0]:.2f}, {start_world[1]:.2f})"
            )

        origin_x, origin_y = self.local_origin

        # Convert robot pose into fixed local frame
        start_local = (
            start_world[0] - origin_x,
            start_world[1] - origin_y,
            start_world[2],
        )

        # ---- Goal in world frame ----
        goal_world_raw = (
            float(self.get_parameter('goal_x').value),
            float(self.get_parameter('goal_y').value),
            0.0,
        )

        gproj = self.project_to_free(goal_world_raw[0], goal_world_raw[1], max_r=2.0)
        if gproj is None:
            self.get_logger().warn("Goal not projectable to free space.")
            return

        goal_world = (gproj[0], gproj[1], 0.0)
        self.current_goal_world = goal_world

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
            f"goal=({goal_local[0]:.2f},{goal_local[1]:.2f})"
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

        # -------------------------------
        # NEW ODOM-BASED GOAL DISTANCE CHECK
        # -------------------------------
        try:
            # robot in odom
            trot = self.tf.lookup_transform("odom", "base_footprint", rclpy.time.Time())
            rx = trot.transform.translation.x
            ry = trot.transform.translation.y

            # goal (currently stored in map frame) -> transform to odom
            gx, gy, _ = self.current_goal_world
            # ---------------------------------------------------------
            # DEBUG PRINTS: robot & goal in map, odom, and local RRT
            # ---------------------------------------------------------

            # Robot in MAP frame
            try:
                t_map = self.tf.lookup_transform("map", "base_footprint", rclpy.time.Time())
                self.get_logger().info(
                    f"[DEBUG] Robot (map): x={t_map.transform.translation.x:.3f}, "
                    f"y={t_map.transform.translation.y:.3f}"
                )
            except Exception:
                self.get_logger().warn("[DEBUG] Could not get robot pose in map frame")

            # Goal in MAP frame
            self.get_logger().info(
                f"[DEBUG] Goal  (map): x={gx:.3f}, y={gy:.3f}"
            )

            # Robot in ODOM frame (we already have rx, ry)
            self.get_logger().info(
                f"[DEBUG] Robot (odom): x={rx:.3f}, y={ry:.3f}"
            )

            # Goal in ODOM frame (after transform)
            goal_ps = PoseStamped()
            goal_ps.header.frame_id = "map"
            goal_ps.pose.position.x = gx
            goal_ps.pose.position.y = gy
            goal_odom = self.tf.transform(goal_ps, "odom")

            gx_o = goal_odom.pose.position.x
            gy_o = goal_odom.pose.position.y

            self.get_logger().info(
                f"[DEBUG] Goal  (odom): x={gx_o:.3f}, y={gy_o:.3f}"
            )

            # Also print the LOCAL RRT frame values
            origin_x, origin_y = self.local_origin

            self.get_logger().info(
                f"[DEBUG] Local RRT Start: x={(rx - origin_x):.3f}, y={(ry - origin_y):.3f}"
            )

            self.get_logger().info(
                f"[DEBUG] Local RRT Goal: x={(gx - origin_x):.3f}, y={(gy - origin_y):.3f}"
            )

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

        # If Nav2 succeeded OR physically close:
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
