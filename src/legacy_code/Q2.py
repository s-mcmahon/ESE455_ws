import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import math, random, time, argparse
import os 

# ---------------- PARAMETERS ----------------
# MAP
map_dim = 10.0
obs_size = 1.0
num_obs = 5
goal = (9.0, 9.0, 10.0, 10.0)
start = (1.0, 1.0, 0.0)
# ROBOT
robot_size = 0.5
step_size = 0.1
# ALGORITHNM
steer_step = 5      
max_nodes = 20000
goal_bias = 0.0

# ----------------ClASS--------------------
class Node:
    def __init__(self, x, y, theta=0.0, parent=None, control=None, path=None):
        self.x, self.y, self.theta = float(x), float(y), float(theta)
        self.parent = parent
        self.control = control
        self.path = path if path is not None else []

class Obstacle:
    def __init__(self, cx, cy, size=1.0):
        self.cx, self.cy, self.size = float(cx), float(cy), float(size)
    def bounds(self, inflate=0.0):
        half = self.size/2.0 + inflate
        return (self.cx-half, self.cy-half, self.cx+half, self.cy+half)
    def contains(self, x, y, inflate=0.0):
        xmin, ymin, xmax, ymax = self.bounds(inflate)
        return (xmin <= x <= xmax) and (ymin <= y <= ymax)

# ----------------- FUCTIONS ----------------------
# Check if node is in goal region
def in_goal(node, region=goal):
    xmin, ymin, xmax, ymax = region
    return (xmin <= node.x <= xmax) and (ymin <= node.y <= ymax)

# Distance between two nodes
def euclidean(n1, n2):
    return np.hypot(n1.x - n2.x, n1.y - n2.y)

# preforms sinc 
def sinc(x):
    return 1.0 if abs(x) < 1e-8 else math.sin(x)/x

# Move from node using control (v, w) for X steps, returns new node
def dynamic_steer(node, control, dt=step_size, steps=steer_step):
    v, w = control
    x, y, th = node.x, node.y, node.theta
    path = []
    for _ in range(steps):
        a = w * dt / 2.0
        s = sinc(a)
        x += v * dt * s * math.cos(th + a)
        y += v * dt * s * math.sin(th + a)
        th = ((th + w*dt + math.pi) % (2*math.pi)) - math.pi
        path.append((x, y))
    return Node(x, y, th, parent=node, control=control, path=path)

# check if path is collision free with inflated obstacles
def collision_free(path, obstacles, inflate=robot_size):
    for (x, y) in path:
        for obs in obstacles:
            if obs.contains(x, y, inflate=inflate):
                return False
    return True

# ----------------- RRT --------------------------
class RRTKinodynamic:
    def __init__(self, start, goal, bounds, obstacles, dt=step_size, max_nodes=max_nodes):
        self.start = Node(*start)
        self.goal = goal
        self.bounds = bounds
        self.obstacles = obstacles
        self.dt = dt
        self.max_nodes = max_nodes
        self.nodes = [self.start]
        ws = [-1.82,-1.46,-1.09,-0.73,-0.36,0.0,0.36,0.73,1.09,1.46,1.82]
        self.controls = [(u, w) for u in [-0.25, 0.25] for w in ws] + [(0.0, 0.0)]

    # get random state with goal bias
    def sample_state(self):
        if random.random() < goal_bias:  
            gx = 0.5*(self.goal[0]+self.goal[2])
            gy = 0.5*(self.goal[1]+self.goal[3])
            return Node(gx, gy, random.uniform(-math.pi, math.pi))
        return Node(random.uniform(self.bounds[0], self.bounds[1]),
                    random.uniform(self.bounds[2], self.bounds[3]),
                    random.uniform(-math.pi, math.pi))

    # find nearest node in tree to given node
    def nearest(self, node):
        return min(self.nodes, key=lambda n: euclidean(n, node))

    # RRT main loop
    def kino_rrt(self):
        for _ in range(self.max_nodes):
            q_rand = self.sample_state()
            q_near = self.nearest(q_rand)
            best_node, best_dist = None, float("inf")
            for u in self.controls:
                q_new = dynamic_steer(q_near, u, self.dt)
                if not (self.bounds[0] <= q_new.x <= self.bounds[1] and
                        self.bounds[2] <= q_new.y <= self.bounds[3]):
                    continue
                if not collision_free(q_new.path, self.obstacles):
                    continue
                d = euclidean(q_new, q_rand)
                if d < best_dist:
                    best_node, best_dist = q_new, d
            if best_node is None:
                continue
            self.nodes.append(best_node)
            if in_goal(best_node, self.goal):
                return self.reconstruct_path(best_node)
        return None

    # reconstruct path from start to given node
    def reconstruct_path(self, last):
        path, controls = [], []
        n = last
        while n is not None:
            if n.path:
                path.extend(n.path[::-1])
            else:
                path.append((n.x, n.y))
            controls.append(n.control)
            n = n.parent
        return path[::-1], controls[::-1]

# ------------------- PL<OTTING ---------------------
def draw_grid(ax, path, obstacles, title=""):
    ax.clear()
    ax.set_xlim(0, map_dim)
    ax.set_ylim(0, map_dim)
    ax.set_aspect("equal")

    # Plot obstacles and inflated bounds
    for obs in obstacles:
        # Original obstacle (solid black)
        xmin, ymin, xmax, ymax = obs.bounds()
        ax.add_patch(plt.Rectangle((xmin, ymin), xmax-xmin, ymax-ymin,
                                   color="black"))

        # Inflated obstacle (dotted black outline)
        xmin, ymin, xmax, ymax = obs.bounds(inflate=robot_size)
        ax.add_patch(plt.Rectangle((xmin, ymin), xmax-xmin, ymax-ymin,
                                   fill=False, ec="black", ls="--", lw=1))

    # Goal region
    xmin, ymin, xmax, ymax = goal
    ax.add_patch(plt.Rectangle((xmin, ymin), xmax-xmin, ymax-ymin,
                               fill=False, ec="red", lw=2))

    # Path
    if path and len(path) > 1:
        xs, ys = zip(*path)
        ax.plot(xs, ys, "-b", lw=2)

    # Start point
    ax.plot(start[0], start[1], "go", ms=6)
    ax.set_title(title)


# ------------------- MAIN -----------------------
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--trials", type=int, default=10, help="number of runs for average runtime")
    args = parser.parse_args()
    # set random seed and generate obstacles
    np.random.seed(6)
    obstacles = [Obstacle(np.random.uniform(2, 8), np.random.uniform(2, 8), obs_size)
                 for _ in range(num_obs)]
    # intialize statistics
    runtimes, success = [], 0
    final_path = None
    # Create output folder
    out_dir = "kino_trial_results"
    os.makedirs(out_dir, exist_ok=True)

    # Run Trils of RRT
    for t in range(args.trials):
        rrt = RRTKinodynamic(start, goal, (0, map_dim, 0, map_dim), obstacles)
        t0 = time.time()
        result = rrt.kino_rrt()
        elapsed = time.time() - t0
        runtimes.append(elapsed)

        fig, ax = plt.subplots(figsize=(6, 6))
        if result:
            success += 1
            path, controls = result
            draw_grid(ax, path, obstacles,
                      title=f"Trial {t+1}: Success in {elapsed:.2f}s, {len(path)} states")
            plt.savefig(os.path.join(out_dir, f"trial_{t+1}_success.png"), dpi=150)
            print(f"[Trial {t+1}] Success in {elapsed:.3f}s")
        else:
            draw_grid(ax, None, obstacles, title=f"Trial {t+1}: Failed")
            plt.savefig(os.path.join(out_dir, f"trial_{t+1}_failed.png"), dpi=150)
            print(f"[Trial {t+1}] Failed in {elapsed:.3f}s")
        plt.close(fig)

    avg_time = sum(runtimes) / len(runtimes)
    print(f"\nSummary: {success}/{args.trials} successes, average runtime {avg_time:.3f}s")