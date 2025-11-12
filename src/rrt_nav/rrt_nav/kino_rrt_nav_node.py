# Imports
import math, random, time, numpy as np, rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration
from std_msgs.msg import Bool

# Angle Helper
def q_from_yaw(yaw):
    from math import sin, cos
    q = Quaternion()
    q.z = sin(yaw / 2.0)
    q.w = cos(yaw / 2.0)
    return q

class RRTPlannerNav2(Node):
    def __init__(self):
        super().__init__('rrt_planner_nav2')

        # parameters
        self.declare_parameter('robot_radius', 0.0)
        self.declare_parameter('safety_margin', 0.1)
        self.declare_parameter('max_iters', 2000)
        self.declare_parameter('goal_bias', 0.35)
        self.declare_parameter('goal_radius', 0.8)
        self.declare_parameter('goal_x', 3.8)
        self.declare_parameter('goal_y', 3.8)
        self.declare_parameter('unknown_is_free', True)

        # dynamics parameters
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

        # state 
        self.grid = None
        self.resolution = 0.05
        self.w = self.h = 0
        self.ox = self.oy = 0.0
        self.task_active = False
        self.success_announced = False

        # throttles
        self.last_plan_time = 0.0
        self.plan_cooldown = 6.0
        self.last_publish_time = 0.0
        self.min_publish_period = 4.0

        # TF + Nav2
        self.tf = Buffer(cache_time=Duration(seconds=10))
        self.tfl = TransformListener(self.tf, self)
        self.nav = BasicNavigator()

        # pubs/subs
        self.create_subscription(OccupancyGrid, '/map', self.on_map, 10)
        self.path_pub = self.create_publisher(Path, '/rrt_path', 10)
        self.waypoint_pub = self.create_publisher(Path, '/rrt_waypoints', 10)
        self.replan_pub = self.create_publisher(Bool, '/rrt_replan', 10)
        self.goal_reached_pub = self.create_publisher(Bool, '/rrt_goal_reached', 10)
        self.create_timer(0.5, self.tick)
        self.get_logger().info("Kinodynamic RRTPlannerNav2 ready (drives Nav2 using SLAM /map).")




    # ------------ Occupancy Grid  ------------
    def on_map(self, msg: OccupancyGrid):
        self.resolution = msg.info.resolution
        self.w, self.h = msg.info.width, msg.info.height
        self.ox = msg.info.origin.position.x
        self.oy = msg.info.origin.position.y
        data = np.array(msg.data, dtype=np.int16).reshape(self.h, self.w)
        grid = np.zeros_like(data, dtype=np.int8)
        grid[data == -1] = -1
        grid[data >= 50] = 1
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

    # ------------ Helper Funcs ------------
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

    def project_to_free(self, x, y, max_r=1.0):
        step = self.resolution
        for r in np.arange(0, max_r, step):
            for th in np.linspace(0, 2 * math.pi, 36):
                nx = x + r * math.cos(th)
                ny = y + r * math.sin(th)
                if self.is_free_xy(nx, ny):
                    return (nx, ny)
        return None

    # ------------ Kinodynamic RRT ------------
    def kinodynamic_rrt_once(self, start, goal, step_time=None, dt=None, max_iters=800, goal_bias=0.2):
        if step_time is None:
            step_time = float(self.get_parameter('step_time').value)
        if dt is None:
            dt = float(self.get_parameter('dt').value)
        v_max = float(self.get_parameter('v_max').value)
        w_max = float(self.get_parameter('w_max').value)
        goal_radius = float(self.get_parameter('goal_radius').value)
        controls_per_expand = int(self.get_parameter('controls_per_expand').value)
        steer_p = float(self.get_parameter('steer_to_goal_prob').value)
        nodes = [start]
        parents = {start: None}
        for _ in range(max_iters):
            if random.random() < goal_bias:
                x_rand, y_rand, th_rand = goal
            else:
                x_rand = random.uniform(self.ox, self.ox + self.w * self.resolution)
                y_rand = random.uniform(self.oy, self.oy + self.h * self.resolution)
                th_rand = random.uniform(-math.pi, math.pi)
            if not self.is_free_xy(x_rand, y_rand):
                continue
            nearest = min(nodes, key=lambda n: math.hypot(n[0] - x_rand, n[1] - y_rand))
            best_new, min_dist = None, float('inf')
            # sample controls
            for _ in range(controls_per_expand):
                if random.random() < steer_p:
                    dx, dy = x_rand - nearest[0], y_rand - nearest[1]
                    desired_yaw = math.atan2(dy, dx)
                    yaw_err = (desired_yaw - nearest[2] + math.pi) % (2 * math.pi) - math.pi
                    w = max(-w_max, min(w_max, yaw_err / step_time))
                    u = 0.8 * v_max if abs(yaw_err) < math.pi/3 else 0.4 * v_max
                else:
                    u = random.uniform(0.0, v_max)
                    w = random.uniform(-w_max, w_max)
                x, y, th = nearest
                traj = [(x, y, th)]
                t = 0.0
                valid = True
                while t < step_time:
                    x += u * math.cos(th) * dt
                    y += u * math.sin(th) * dt
                    th += w * dt
                    if not self.is_free_xy(x, y):
                        valid = False
                        break
                    traj.append((x, y, th))
                    t += dt
                if not valid:
                    continue
                dist = math.hypot(x - x_rand, y - y_rand)
                if dist < min_dist:
                    best_new = (x, y, th)
                    min_dist = dist
            if best_new:
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

    # ------------ Publishing PAth ------------
    def downsample_path(self, pts, min_sep=0.5):
        if not pts:
            return []
        new_pts = [pts[0]]
        for p in pts[1:]:
            dx = p[0] - new_pts[-1][0]
            dy = p[1] - new_pts[-1][1]
            if math.hypot(dx, dy) >= min_sep:
                new_pts.append(p)
        # always keep the goal
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
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation = q_from_yaw(th)
            poses.append(ps)
        msg.poses = poses
        self.path_pub.publish(msg)
        self.waypoint_pub.publish(msg)
        self.replan_pub.publish(Bool(data=True))
        return poses, pts

    # ------------ MAin logic ------------
    def tick(self):
        if self.grid is None or self.success_announced:
            return
        try:
            t = self.tf.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = float(t.transform.translation.x)
            y = float(t.transform.translation.y)
            q = t.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            th = math.atan2(siny_cosp, cosy_cosp)
            pose = (x, y, th)
        except Exception:
            self.get_logger().info("Waiting for TF map->base_link…")
            return
        sproj = self.project_to_free(pose[0], pose[1], max_r=0.5)
        if sproj is None:
            self.get_logger().warn("Start in collision, cannot project to free space.")
            return
        pose = (sproj[0], sproj[1], pose[2])

        goal = (
            float(self.get_parameter('goal_x').value),
            float(self.get_parameter('goal_y').value),
            0.0,
        )
        gproj = self.project_to_free(goal[0], goal[1], max_r=2.0)
        if gproj is None:
            self.get_logger().warn("Goal not projectable to free space.")
            return
        goal = (gproj[0], gproj[1], goal[2])
        goal_radius = float(self.get_parameter('goal_radius').value)
        if math.hypot(pose[0] - goal[0], pose[1] - goal[1]) <= goal_radius:
            self.announce_success()
            return
        now = time.time()
        if now - self.last_plan_time < self.plan_cooldown:
            return
        self.last_plan_time = now
        self.get_logger().info("Planning with kinodynamic RRT...")
        path = self.kinodynamic_rrt_once(
            start=pose,
            goal=goal,
            step_time=float(self.get_parameter('step_time').value),
            dt=float(self.get_parameter('dt').value),
            max_iters=int(self.get_parameter('max_iters').value),
            goal_bias=float(self.get_parameter('goal_bias').value),
        )
        if len(path) < 2:
            self.get_logger().warn("No valid path found.")
            return
        min_sep = float(self.get_parameter('min_waypoint_separation').value)
        path_ds = self.downsample_path(path, min_sep)
        poses, pts_sent = self.publish_path(path_ds)
        if not poses:
            return
        if len(poses) <= 2:
            self.nav.goToPose(poses[-1])
            self.get_logger().info("Sent single kinodynamic goal to Nav2.")
        else:
            self.nav.followWaypoints(poses)
            self.get_logger().info(f"Sent {len(poses)} kinodynamic waypoints to Nav2.")
        self.task_active = True
        self.last_publish_time = now

    # ------------ REached Goal? ------------
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
