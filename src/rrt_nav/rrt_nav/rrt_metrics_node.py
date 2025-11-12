import rclpy, numpy as np, cv2, os, time, json
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from datetime import datetime
from nav_msgs.msg import Path


class RRTMetrics(Node):
    def __init__(self):
        super().__init__('rrt_metrics')
        self.grid = None
        self.res = None
        self.ox = self.oy = 0.0
        self.positions = []
        self.replan_times = []
        self.last_replan = None
        self.start_time = time.time()

        # Output folder set-up
        base_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'results')
        os.makedirs(base_path, exist_ok=True)

        run_name = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.run_path = os.path.join(base_path, f'run_{run_name}')
        os.makedirs(self.run_path, exist_ok=True)

        self.get_logger().info(f"Results will be saved to: {self.run_path}")

        # ROS subscriptions
        self.create_subscription(OccupancyGrid, '/map', self.on_map, 10)
        self.create_subscription(PoseStamped, '/rrt_pose', self.on_pose, 10)
        self.create_subscription(Bool, '/rrt_replan', self.on_replan, 10)
        self.timer = self.create_timer(0.1, self.update_plot)

        self.waypoint_sets = [] 
        self.create_subscription(Path, '/rrt_waypoints', self.on_waypoints, 10)

        # Video Params
        self.video_writer = None
        self.fps = 10  


    def on_waypoints(self, msg):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if pts:
            self.waypoint_sets.append(pts)
            self.get_logger().info(f"Received {len(pts)} waypoints (total {len(self.waypoint_sets)} plans).")


    def on_map(self, msg):
        data = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        self.grid = np.flipud(data)
        self.res = msg.info.resolution
        self.ox = msg.info.origin.position.x
        self.oy = msg.info.origin.position.y

    def on_pose(self, msg):
        x, y = msg.pose.position.x, msg.pose.position.y
        self.positions.append((x, y))

    def on_replan(self, msg):
        now = time.time()
        if self.last_replan:
            dt = now - self.last_replan
            self.get_logger().info(f"Replan runtime: {dt:.2f}s")
        self.last_replan = now
        self.replan_times.append(now - self.start_time)

    def update_plot(self):
        if self.grid is None:
            return
        img = np.zeros((*self.grid.shape, 3), dtype=np.uint8)
        img[self.grid == 0] = (255, 255, 255)
        img[self.grid == 1] = (0, 0, 0)
        img[self.grid == -1] = (150, 150, 150)
        for (x, y) in self.positions:
            i = int((x - self.ox) / self.res)
            j = int((y - self.oy) / self.res)
            if 0 <= i < img.shape[1] and 0 <= j < img.shape[0]:
                img[-j-1, i] = (0, 0, 255)
        # Start Video
        if self.video_writer is None:
            height, width, _ = img.shape
            video_path = os.path.join(self.run_path, 'trajectory.mp4')
            self.video_writer = cv2.VideoWriter(video_path, cv2.VideoWriter_fourcc(*'mp4v'), self.fps, (width, height))
            self.get_logger().info(f"Recording video to {video_path}")
        self.video_writer.write(img)
        cv2.imshow('RRT metrics', img)
        cv2.waitKey(1)

    def save_results(self):
        if self.video_writer:
            self.video_writer.release()
        cv2.destroyAllWindows()
        metrics = {
            'total_runtime_sec': time.time() - self.start_time,
            'num_replans': len(self.replan_times),
            'replan_times_sec': self.replan_times,
            'path_length_m': self.compute_path_length()
        }
        with open(os.path.join(self.run_path, 'waypoints.json'), 'w') as f:
            json.dump(self.waypoint_sets, f, indent=2)
        with open(os.path.join(self.run_path, 'metrics.json'), 'w') as f:
            json.dump(metrics, f, indent=2)
        if self.grid is not None:
            img = np.zeros((*self.grid.shape, 3), dtype=np.uint8)
            img[self.grid == 0] = (255,255,255)
            img[self.grid == 1] = (0,0,0)
            img[self.grid == -1] = (150,150,150)
            for (x, y) in self.positions:
                i = int((x - self.ox) / self.res)
                j = int((y - self.oy) / self.res)
                if 0 <= i < img.shape[1] and 0 <= j < img.shape[0]:
                    img[-j-1, i] = (0,0,255)
            cv2.imwrite(os.path.join(self.run_path, 'final_map.png'), img)
        self.get_logger().info(f"Saved all results in {self.run_path}")
    def compute_path_length(self):
        if len(self.positions) < 2:
            return 0.0
        dist = 0.0
        for i in range(1, len(self.positions)):
            x1, y1 = self.positions[i-1]
            x2, y2 = self.positions[i]
            dist += np.hypot(x2 - x1, y2 - y1)
        return dist


def main():
    rclpy.init()
    node = RRTMetrics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted!")
    finally:
        node.save_results()
        node.destroy_node()
        rclpy.shutdown()
