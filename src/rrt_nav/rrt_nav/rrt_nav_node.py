import math, random, time, numpy as np, rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration
from std_msgs.msg import Bool

def q_from_yaw(yaw):
    from math import sin, cos
    q = Quaternion()
    q.z = sin(yaw / 2.0)
    q.w = cos(yaw / 2.0)
    return q

class RRTPlannerNav2(Node):
    def __init__(self):
        super().__init__('rrt_planner_nav2')

        # base params
        self.declare_parameter('robot_radius', 0.0)
        self.declare_parameter('safety_margin', 0.1)
        self.declare_parameter('step_size', 0.30)
        self.declare_parameter('max_iters', 600)
        self.declare_parameter('goal_bias', 0.35)
        self.declare_parameter('goal_radius', 1.0)
        self.declare_parameter('goal_x',3.8)
        self.declare_parameter('goal_y', 3.8)
        self.declare_parameter('unknown_is_free', True)
        self.declare_parameter('v_max', 0.25)
        self.declare_parameter('w_max', 1.82)
        self.declare_parameter('dt', 0.2)
        self.declare_parameter('min_waypoint_separation', 0.6)
        self.declare_parameter('max_waypoints', 20)
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        # state
        self.grid = None
        self.resolution = 0.05
        self.w = self.h = 0
        self.ox = self.oy = 0.0
        self.task_active = False
        self.success_announced = False

        # planning params
        self.last_plan_time = 0.0
        self.plan_cooldown = 6.0
        self.last_publish_time = 0.0
        self.min_publish_period = 4.0
        self.last_path_sig = None
        self.replan_trigger_distance = 2.0

        # progress params
        self.last_progress_t = 0.0
        self.last_progress_dist = None
        self.stuck_window_sec = 3.0
        self.stuck_delta_thresh = 0.05  # m improvement required over window

        # ----- TF + Nav2 -----
        self.tf = Buffer(cache_time=Duration(seconds=10))
        self.tfl = TransformListener(self.tf, self)
        self.nav = BasicNavigator()

        # ----- pubs/subs -----
        self.create_subscription(OccupancyGrid, '/map', self.on_map, 10)
        self.path_pub = self.create_publisher(Path, '/rrt_path', 10)
        self.replan_pub = self.create_publisher(Bool, '/rrt_replan', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/rrt_pose', 10)
        self.goal_reached_pub = self.create_publisher(Bool, '/rrt_goal_reached', 10)
        self.waypoint_pub = self.create_publisher(Path, '/rrt_waypoints', 10)
        self.create_timer(0.5, self.tick)
        self.get_logger().info("RRTPlannerNav2 ready (drives Nav2 using SLAM /map).")

    # ------------ map ------------
    def on_map(self, msg: OccupancyGrid):
        self.resolution = msg.info.resolution
        self.w, self.h = msg.info.width, msg.info.height
        self.ox = msg.info.origin.position.x
        self.oy = msg.info.origin.position.y
        data = np.array(msg.data, dtype=np.int16).reshape(self.h, self.w)
        # free=0, occ=1, unknown=-1
        grid = np.zeros_like(data, dtype=np.int8)
        grid[data == -1] = -1
        grid[data >= 50] = 1
        # Inflate obstacles 
        rad = float(self.get_parameter('robot_radius').value) + float(self.get_parameter('safety_margin').value)
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

    # ------------ Helper Func ------------
    def world_to_grid(self, x, y):
        return int((x - self.ox) / self.resolution), int((y - self.oy) / self.resolution)

    def inside(self, i, j):
        return 0 <= i < self.w and 0 <= j < self.h

    def is_free_xy(self, x, y):
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

    def collision_free(self, p1, p2):
        if self.grid is None:
            return False
        unknown_ok = bool(self.get_parameter('unknown_is_free').value)
        (x1, y1), (x2, y2) = p1, p2
        d = math.hypot(x2 - x1, y2 - y1)
        n = max(2, int(d / (0.5 * self.resolution)))
        xs = np.linspace(x1, x2, n)
        ys = np.linspace(y1, y2, n)
        for x, y in zip(xs, ys):
            i, j = self.world_to_grid(x, y)
            if not self.inside(i, j):
                return False
            v = self.grid[j, i]
            if v == 1:
                return False
            if v == -1 and not unknown_ok:
                return False
        return True

    def project_to_free(self, x, y, max_r=2.0):
        if self.is_free_xy(x, y):
            return (x, y)
        R = max_r
        steps_r = max(1, int(R / max(self.resolution, 0.05)))
        for k in range(1, steps_r + 1):
            r = k * (R / steps_r)
            samples = 36
            for th_i in range(samples):
                th = (2 * math.pi) * th_i / samples
                nx = x + r * math.cos(th)
                ny = y + r * math.sin(th)
                if self.is_free_xy(nx, ny):
                    return (nx, ny)
        return None 
    
    # ------------ Success? ------------
    def announce_success(self):
        if self.success_announced:
            return
        self.success_announced = True
        self.get_logger().info("Goal reached (within goal_radius). Stopping planner and shutting down.")
        self.goal_reached_pub.publish(Bool(data=True))
        try:
            if hasattr(self.nav, "cancelTask"):
                self.nav.cancelTask()
        except Exception:
            pass
        self.get_clock().sleep_for(Duration(seconds=0.5))
        self.destroy_node()
        rclpy.shutdown()
        import sys
        sys.exit(0)

    # ------------ Publishing PAth Func ------------
    def _smooth_shortcut(self, pts):
        if len(pts) < 3 or self.grid is None:
            return pts[:]
        res = [pts[0]]
        i = 0
        n = len(pts)
        while i < n - 1:
            j = i + 1
            furthest = j
            while j < n and self.collision_free(pts[i], pts[j]):
                furthest = j
                j += 1
            res.append(pts[furthest])
            i = furthest
        return res

    def _decimate_by_separation(self, pts, min_sep):
        if len(pts) <= 2:
            return pts[:]
        out = [pts[0]]
        last = pts[0]
        for p in pts[1:-1]:
            if math.hypot(p[0] - last[0], p[1] - last[1]) >= min_sep:
                out.append(p)
                last = p
        out.append(pts[-1])
        return out

    def _validate_path_tail(self, pts):
        if len(pts) < 2:
            return []
        valid = [pts[0]]
        for i in range(1, len(pts)):
            if not self.is_free_xy(*pts[i]):
                break
            if not self.collision_free(valid[-1], pts[i]):
                break
            valid.append(pts[i])
        return valid if len(valid) >= 2 else []

    def _path_signature(self, pts, stride=3):
        if not pts:
            return ()
        pts_s = pts[::max(1, stride)]
        return tuple((round(x, 2), round(y, 2)) for (x, y) in pts_s)

    def _max_deviation(self, a, b):
        if not a or not b:
            return float('inf')
        m = min(len(a), len(b))
        dmax = 0.0
        for i in range(m):
            d = math.hypot(a[i][0] - b[i][0], a[i][1] - b[i][1])
            dmax = max(dmax, d)
        return dmax

    # ------------ RRT ------------
    def rrt_once(self, start, goal, step, max_iters, goal_bias, goal_radius):
        nodes = [start]
        parents = {start: None}
        xmin, xmax = self.ox, self.ox + self.w * self.resolution
        ymin, ymax = self.oy, self.oy + self.h * self.resolution
        gproj = self.project_to_free(goal[0], goal[1], max_r=2.0)
        if gproj is not None:
            goal = gproj
        for _ in range(max_iters):
            if random.random() < goal_bias:
                rx, ry = goal
            else:
                rx, ry = random.uniform(xmin, xmax), random.uniform(ymin, ymax)
                if not self.is_free_xy(rx, ry):
                    continue
            nearest = min(nodes, key=lambda n: math.hypot(n[0] - rx, n[1] - ry))
            dx, dy = rx - nearest[0], ry - nearest[1]
            L = math.hypot(dx, dy)
            if L < 1e-6:
                continue
            nx, ny = nearest[0] + step * dx / L, nearest[1] + step * dy / L
            if not self.is_free_xy(nx, ny):
                continue
            if not self.collision_free(nearest, (nx, ny)):
                continue
            nodes.append((nx, ny))
            parents[(nx, ny)] = nearest
            if math.hypot(nx - goal[0], ny - goal[1]) < goal_radius:
                path = [(nx, ny)]
                cur = (nx, ny)
                while parents[cur] is not None:
                    cur = parents[cur]
                    path.append(cur)
                return list(reversed(path))
        return []

    def publish_path(self, pts):
        # smooth + space
        pts = self._smooth_shortcut(pts)
        min_sep = float(self.get_parameter('min_waypoint_separation').value)
        pts = self._decimate_by_separation(pts, min_sep)
        pts = self._validate_path_tail(pts)
        if len(pts) < 2:
            return [], []
        max_wps = int(self.get_parameter('max_waypoints').value)
        if len(pts) > max_wps:
            stride = max(1, len(pts) // max_wps)
            pts = pts[::stride] + ([pts[-1]] if pts[-1] != pts[-1] else [])
        # Build Path
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        poses = []
        for k, (x, y) in enumerate(pts):
            nxt = pts[min(k + 1, len(pts) - 1)]
            yaw = math.atan2(nxt[1] - y, nxt[0] - x)
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.header.stamp = msg.header.stamp
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation = q_from_yaw(yaw)
            poses.append(ps)
        msg.poses = poses
        self.path_pub.publish(msg)
        self.waypoint_pub.publish(msg)  
        self.replan_pub.publish(Bool(data=True))
        return poses, pts

    # ------------ Prevent it getting Stuckl ------------
    def _update_progress(self, pose, goal):
        d = math.hypot(pose[0] - goal[0], pose[1] - goal[1])
        now = time.time()
        if self.last_progress_t == 0.0:
            self.last_progress_t = now
            self.last_progress_dist = d
            return False  
        if now - self.last_progress_t >= self.stuck_window_sec:
            improved = (self.last_progress_dist - d) if self.last_progress_dist is not None else 0.0
            self.last_progress_t = now
            self.last_progress_dist = d
            if improved < self.stuck_delta_thresh:
                self.get_logger().warn(f"No progress ({improved:.2f} m in {self.stuck_window_sec:.1f}s). Cancelling and replanning.")
                try:
                    if hasattr(self.nav, "cancelTask"):
                        self.nav.cancelTask()
                except Exception:
                    pass
                self.task_active = False
                self.last_plan_time = 0.0
                return True
        return False

    # ------------ main logic ------------
    def tick(self):
        if self.grid is None or self.success_announced:
            return

        pose = self.get_pose()
        if pose is None:
            self.get_logger().info("Waiting for TF map->base_link…")
            return

        goal = (
            float(self.get_parameter('goal_x').value),
            float(self.get_parameter('goal_y').value),
        )
        goal_radius = float(self.get_parameter('goal_radius').value)
        if math.hypot(pose[0] - goal[0], pose[1] - goal[1]) <= goal_radius:
            self.announce_success()
            return
        try:
           if self.task_active and hasattr(self.nav, "isTaskComplete"):
                if not self.nav.isTaskComplete():
                    stuck = self._update_progress(pose, goal)
                    if not stuck:
                            return  
                if math.hypot(pose[0] - goal[0], pose[1] - goal[1]) <= goal_radius:
                    self.announce_success()
                    return
                self.task_active = False
        except Exception:
            pass
        now = time.time()
        if now - self.last_plan_time < self.plan_cooldown:
            return
        self.last_plan_time = now
        # Plan GLobal
        path = self.rrt_once(
            start=pose,
            goal=goal,
            step=float(self.get_parameter('step_size').value),
            max_iters=int(self.get_parameter('max_iters').value),
            goal_bias=float(self.get_parameter('goal_bias').value),
            goal_radius=goal_radius,
        )
        #Plan Local if that fails
        if len(path) < 2:
            self.get_logger().warn("RRT: no global path, trying local goal...")
            sx, sy = pose
            best, best_dist = None, float('inf')
            for _ in range(80):
                rx = sx + random.uniform(-1.2, 1.2)
                ry = sy + random.uniform(-1.2, 1.2)
                if not self.is_free_xy(rx, ry):
                    continue
                if not self.collision_free((sx, sy), (rx, ry)):
                    continue
                d_goal = math.hypot(goal[0] - rx, goal[1] - ry)
                if d_goal < best_dist:
                    best, best_dist = (rx, ry), d_goal
            if best is None:
                self.get_logger().warn("No reachable local goal in free space.")
                return
            local_path = self.rrt_once(
                start=pose, goal=best,
                step=float(self.get_parameter('step_size').value),
                max_iters=int(self.get_parameter('max_iters').value),
                goal_bias=0.3,
                goal_radius=0.25,
            )
            if len(local_path) < 2:
                self.get_logger().warn("Still no path to local goal, retrying later...")
                return
            poses, pts_sent = self.publish_path(local_path)
            if now - self.last_publish_time < self.min_publish_period and self.task_active:
                return
            sig = self._path_signature(pts_sent)
            if self.last_path_sig is not None:
                prev_pts = [(x, y) for (x, y) in self.last_path_sig]
                dev = self._max_deviation(prev_pts, pts_sent[:len(prev_pts)])
                if dev < self.replan_trigger_distance and self.task_active:
                    return
            if len(poses) <= 2:
                self.nav.goToPose(poses[-1])
                self.get_logger().info("Sent single local goal to Nav2.")
            else:
                self.nav.followWaypoints(poses)
                self.get_logger().info(f"Sent {len(poses)} local waypoints to Nav2.")
            self.task_active = True
            self.last_publish_time = now
            self.last_path_sig = sig
            self.last_progress_t = 0.0
            self.last_progress_dist = None
            return
        poses, pts_sent = self.publish_path(path)
        if not poses:
            return
        if now - self.last_publish_time < self.min_publish_period and self.task_active:
            return
        sig = self._path_signature(pts_sent)
        if self.last_path_sig is not None:
            prev_pts = [(x, y) for (x, y) in self.last_path_sig]
            dev = self._max_deviation(prev_pts, pts_sent[:len(prev_pts)])
            if dev < self.replan_trigger_distance and self.task_active:
                return
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.pose.position.x, ps.pose.position.y = pose
        self.pose_pub.publish(ps)
        if len(poses) <= 2:
            self.nav.goToPose(poses[-1])
            self.get_logger().info("Sent single global goal to Nav2.")
        else:
            self.nav.followWaypoints(poses)
            self.get_logger().info(f"Sent {len(poses)} global waypoints to Nav2.")
        self.task_active = True
        self.last_publish_time = now
        self.last_path_sig = sig
        self.last_progress_t = 0.0
        self.last_progress_dist = None

    # ------------ TF pose ------------
    def get_pose(self):
        try:
            t = self.tf.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = float(t.transform.translation.x)
            y = float(t.transform.translation.y)
            return (x, y)
        except Exception:
            return None

def main():
    rclpy.init()
    node = RRTPlannerNav2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
