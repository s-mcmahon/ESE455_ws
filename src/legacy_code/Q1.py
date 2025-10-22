import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import shutil
import time 
import os 

# ---------------- PARAMS----------------
# MAP 
map_dim = 10.0
map_res = 0.1
num_obs = 5
obs_size = 1.0
goal = (9.0, 9.0, 10.0, 10.0)
start = (1.0, 1.0)
# ROBOT
robot_size = 0.5
sensor_range = 1.0
step_size = 0.1
#AlGORITHM
max_nodes = 1000
goal_bias = 0.10
frontier_bias = 0.15
move_step = 10
# IMAGE + VIDEO 
video_path = "rrt_sensor.mp4"
img_path = "rrt_sensor.gif"
fps = 10
dpi = 150


# ----------------- CLASSES ------------------
class Node:
    def __init__(self, x, y, parent=None):
        self.x, self.y = float(x), float(y)
        self.parent = parent

class Obstacle:
    def __init__(self, cx, cy, size=1.0):
        self.cx, self.cy, self.size = float(cx), float(cy), float(size)
    def bounds(self):
        half = self.size/2.0
        return (self.cx-half, self.cy-half, self.cx+half, self.cy+half)

# ----------------- CLASSES ----------------------

# Chechk if node is in goal region
def in_goal(node, region=goal):
    xmin, ymin, xmax, ymax = region
    return (xmin <= node.x <= xmax) and (ymin <= node.y <= ymax)

# Euclidean distance between two nodes
def euclidean(n1, n2):
    return np.sqrt((n1.x - n2.x)**2 + (n1.y - n2.y)**2)

# Reconstruct path from last node to start
def reconstruct_path(last):
    path = []
    n = last
    while n is not None:
        path.append((n.x, n.y))
        n = n.parent
    return path[::-1]

# Create Grid Environment (-1: unknown, 0: free, 1: occupied) - Everything starts as unknown
def init_grid(size, res):
    N = int(np.ceil(size/res))
    return -np.ones((N, N), dtype=int)

# Convert from world (x,y) to grid (i,j)
def world_to_grid(x, y, res=map_res):
    i = int(np.floor(np.clip(x / res, 0, np.inf)))
    j = int(np.floor(np.clip(y / res, 0, np.inf)))
    return i, j

# Convert from grid (i,j) to world (x,y) - center of cell
def grid_to_world(i, j, res=map_res):
    return (i + 0.5) * res, (j + 0.5) * res

# Inflate occupied cell by a disk of given radius (in cells)
def inflate_obs(grid, i_c, j_c, radius_cells):
    H, W = grid.shape
    rr = radius_cells
    for dj in range(-rr, rr+1):
        for di in range(-rr, rr+1):
            ii, jj = i_c + di, j_c + dj
            if 0 <= ii < W and 0 <= jj < H and di*di + dj*dj <= rr*rr:
                grid[jj, ii] = 1

# Update grid based on robot position, and sesnor data (obstacles)
def update_grid(grid, robot, obstacles, res=map_res, R=sensor_range, inflate=robot_size):
    H, W = grid.shape
    i_c, j_c = world_to_grid(robot.x, robot.y, res)
    cells_R = int(np.ceil(R / res))
    infl_cells = int(np.ceil(inflate / res))
    for dj in range(-cells_R, cells_R + 1):
        for di in range(-cells_R, cells_R + 1):
            i, j = i_c + di, j_c + dj
            if not (0 <= i < W and 0 <= j < H):
                continue
            wx, wy = grid_to_world(i, j, res)
            if np.hypot(wx - robot.x, wy - robot.y) > R:
                continue
            occ = False
            for obs in obstacles:
                xmin, ymin, xmax, ymax = obs.bounds()
                if (xmin <= wx <= xmax) and (ymin <= wy <= ymax):
                    occ = True
                    break
            if occ:
                inflate_obs(grid, i, j, infl_cells)
            else:
                if grid[j, i] != 1:
                    grid[j, i] = 0
    return grid

# Verify if (x,y) is free space
def is_free_xy(grid, x, y, res=map_res):
    i, j = world_to_grid(x, y, res)
    H, W = grid.shape
    if 0 <= i < W and 0 <= j < H:
        return grid[j, i] == 0
    return False

# Ensure path between two points is collision-free
def collision_free_path(p1, p2, grid, res=map_res):
    length = np.hypot(p2[0]-p1[0], p2[1]-p1[1])
    steps = max(2, int(np.ceil(length / (0.5 * res))))
    xs = np.linspace(p1[0], p2[0], steps)
    ys = np.linspace(p1[1], p2[1], steps)
    H, W = grid.shape
    for x, y in zip(xs, ys):
        i, j = world_to_grid(x, y, res)
        if not (0 <= i < W and 0 <= j < H):
            return False
        if grid[j, i] != 0:
            return False
    return True

# Find free space cells adjacent to unknown cells
def find_frontiers(grid):
    H, W = grid.shape
    frontiers = []
    for j in range(1, H-1):
        for i in range(1, W-1):
            if grid[j, i] != 0:
                continue
            if (grid[j-1, i] == -1 or grid[j+1, i] == -1 or
                grid[j, i-1] == -1 or grid[j, i+1] == -1):
                frontiers.append(grid_to_world(i, j))
    return frontiers

# Robot Movement
def steer(from_node, to_node, step=step_size):
    dx, dy = to_node.x - from_node.x, to_node.y - from_node.y
    d = euclidean(from_node, to_node)
    if d <= 1e-9:
        return None
    if d <= step:
        return Node(to_node.x, to_node.y, parent=from_node)
    theta = np.arctan2(dy, dx)
    return Node(from_node.x + step*np.cos(theta),
                from_node.y + step*np.sin(theta),
                parent=from_node)

# Sample random free node in grid with goal bias and frontier bias
def sample_free(grid, goal_bias_pt=None, frontiers=None):
    H, W = grid.shape
    for _ in range(8000):
        r = np.random.rand()
        if goal_bias_pt is not None and r < goal_bias:
            x, y = goal_bias_pt
        elif frontiers and len(frontiers) > 0 and r < goal_bias + frontier_bias:
            x, y = frontiers[np.random.randint(len(frontiers))]
        else:
            i = np.random.randint(0, W)
            j = np.random.randint(0, H)
            x, y = grid_to_world(i, j)
        if is_free_xy(grid, x, y):
            return Node(x, y)
    return None

# Find nearest node in tree to random sample
def nearest(nodes, q_rand):
    return min(nodes, key=lambda n: euclidean(n, q_rand))

# Path plans in only the known space of the grid``
def rrt_known_space(start_xy, goal, grid, frontiers=None, max_nodes=max_nodes):
    # Initialize tree, start node, and goal bias point
    start = Node(*start_xy)
    if not is_free_xy(grid, start.x, start.y):
        return None
    nodes = [start]
    gx = 0.5 * (goal[0] + goal[2])
    gy = 0.5 * (goal[1] + goal[3])
    goal_bias_pt = (gx, gy) if is_free_xy(grid, gx, gy) else None
    # generate tree
    for _ in range(max_nodes):
        q_rand = sample_free(grid, goal_bias_pt=goal_bias_pt, frontiers=frontiers)
        if q_rand is None:
            break
        q_near = nearest(nodes, q_rand)
        q_new = steer(q_near, q_rand, step_size)
        if q_new is None:
            continue
        if not is_free_xy(grid, q_new.x, q_new.y):
            continue
        if collision_free_path((q_near.x, q_near.y), (q_new.x, q_new.y), grid):
            nodes.append(q_new)
            if in_goal(q_new, goal):
                return reconstruct_path(q_new)
    return None

# ------------------- PLOTTING ---------------------
def draw_grid(ax, grid, path, robot, obstacles, title=""):
    ax.clear()
    H, W = grid.shape
    rgb = np.zeros((H, W, 3))
    rgb[grid == -1] = [0.8, 0.8, 0.8]
    rgb[grid == 0]  = [1.0, 1.0, 1.0]
    rgb[grid == 1]  = [0.0, 0.0, 0.0]
    ax.imshow(rgb, origin="lower", extent=(0, map_dim, 0, map_dim))
    for obs in obstacles:
        xmin, ymin, xmax, ymax = obs.bounds()
        ax.add_patch(plt.Rectangle((xmin, ymin), xmax-xmin, ymax-ymin, fill=False, ls="--", ec="red"))
    xmin, ymin, xmax, ymax = goal
    ax.add_patch(plt.Rectangle((xmin, ymin), xmax-xmin, ymax-ymin, fill=False, ec="red", lw=2))
    if path and len(path) > 1:
        xs, ys = zip(*path)
        ax.plot(xs, ys, "-b", lw=2)
    ax.plot(robot.x, robot.y, "go", ms=6)
    ax.set_xlim(0, map_dim); ax.set_ylim(0, map_dim)
    ax.set_aspect("equal")
    ax.set_title(title)

# ------------------- ONLINE RRT ---------------------
def run_trial(obstacles , trial_idx=0, save_folder="trial_results"):
    # Initialize grid, robot, and path
    grid = init_grid(map_dim, map_res)
    robot = Node(*start)
    path_taken = [(robot.x, robot.y)]
    # Cap Cycle to prevent infinite exploration
    for cycle in range(240):
        # Update grid, find frontiers, and plan path
        update_grid(grid, robot, obstacles)
        frontiers = find_frontiers(grid)
        plan = rrt_known_space((robot.x, robot.y), goal, grid, frontiers=frontiers)
        # If no path to goal, try to plan to best frontier
        if plan is None and len(frontiers) > 0:
            gx, gy = 0.5*(goal[0]+goal[2]), 0.5*(goal[1]+goal[3])
            best_idx, best_cost = None, float("inf")
            for k, (fx, fy) in enumerate(frontiers):
                d_robot = np.hypot(fx - robot.x, fy - robot.y)
                d_goal  = np.hypot(fx - gx, fy - gy)
                cost = 0.6*d_robot + 0.4*d_goal
                if cost < best_cost:
                    best_cost, best_idx = cost, k
            fx, fy = frontiers[best_idx]
            tmp_goal_region = max(0.25, 2.0*map_res)
            tmp_goal = (fx - tmp_goal_region, fy - tmp_goal_region,
                        fx + tmp_goal_region, fy + tmp_goal_region)
            plan = rrt_known_space((robot.x, robot.y), tmp_goal, grid, frontiers=frontiers)
        if plan is None:
            continue
        # Move along planned path up to move_step
        advanced = 0
        for (px, py) in plan[1:]:
            dx, dy = px - robot.x, py - robot.y
            d = np.hypot(dx, dy)
            if d < 1e-9: continue
            s = min(step_size, d)
            robot = Node(robot.x + s*dx/d, robot.y + s*dy/d)
            path_taken.append((robot.x, robot.y))
            update_grid(grid, robot, obstacles)
            advanced += 1
            if in_goal(robot, goal): break
            if advanced >= move_step: break
        if in_goal(robot, goal):
            break
    # Save final map and path
    os.makedirs(save_folder, exist_ok=True)
    fig = plt.figure(figsize=(6,6))
    draw_grid(plt.gca(), grid, path_taken, robot, obstacles,
              title=f"Trial {trial_idx})")
    out_path = os.path.join(save_folder, f"trial_{trial_idx:02d}.png")
    plt.savefig(out_path, dpi=150)
    plt.close(fig)
    return in_goal(robot, goal)  

# Same as function above but saves video of the running
def run_video(obstacles, video_out="rrt_trial.mp4"):
    grid = init_grid(map_dim, map_res)
    robot = Node(*start)
    path_taken = [(robot.x, robot.y)]

    # Setup figure and writer
    fig, ax = plt.subplots(figsize=(6,6))
    have_ffmpeg = shutil.which("ffmpeg") is not None
    writer = (animation.FFMpegWriter(fps=fps, codec="libx264", bitrate=1800)
              if have_ffmpeg else animation.PillowWriter(fps=fps))
    out_path = video_out if have_ffmpeg else video_out.replace(".mp4",".gif")

    with writer.saving(fig, out_path, dpi):
        for cycle in range(240):
            update_grid(grid, robot, obstacles)
            frontiers = find_frontiers(grid)
            plan = rrt_known_space((robot.x, robot.y), goal, grid, frontiers=frontiers)
            if plan is None and len(frontiers) > 0:
                gx, gy = 0.5*(goal[0]+goal[2]), 0.5*(goal[1]+goal[3])
                best_idx, best_cost = None, float("inf")
                for k, (fx, fy) in enumerate(frontiers):
                    d_robot = np.hypot(fx - robot.x, fy - robot.y)
                    d_goal  = np.hypot(fx - gx, fy - gy)
                    cost = 0.6*d_robot + 0.4*d_goal
                    if cost < best_cost:
                        best_cost, best_idx = cost, k
                fx, fy = frontiers[best_idx]
                tmp_goal_region = max(0.25, 2.0*map_res)
                tmp_goal = (fx - tmp_goal_region, fy - tmp_goal_region,
                            fx + tmp_goal_region, fy + tmp_goal_region)
                plan = rrt_known_space((robot.x, robot.y), tmp_goal, grid, frontiers=frontiers)
            if plan is None:
                draw_grid(ax, grid, path_taken, robot, obstacles, title=f"No path (cycle {cycle})")
                writer.grab_frame()
                continue
            advanced = 0
            for (px, py) in plan[1:]:
                dx, dy = px - robot.x, py - robot.y
                d = np.hypot(dx, dy)
                if d < 1e-9: continue
                s = min(step_size, d)
                robot = Node(robot.x + s*dx/d, robot.y + s*dy/d)
                path_taken.append((robot.x, robot.y))
                update_grid(grid, robot, obstacles)
                draw_grid(ax, grid, path_taken, robot, obstacles, title=f"Cycle {cycle}")
                writer.grab_frame()
                advanced += 1
                if in_goal(robot, goal): break
                if advanced >= move_step: break
            if in_goal(robot, goal):
                draw_grid(ax, grid, path_taken, robot, obstacles, title="Reached goal!")
                writer.grab_frame()
                break
    plt.close(fig)
    print(f"Saved video to: {out_path}")

# ------------------- MAIN ---------------------
if __name__ == "__main__":
    num_trials = 1
    runtimes, successes = [], []
    np.random.seed(6)
    obstacles = [Obstacle(np.random.uniform(2, 8), np.random.uniform(2, 8), obs_size)
                 for _ in range(num_obs)]
    
    for t in range(num_trials):
        if num_trials == 1:
            run_video(obstacles, video_out=video_path)
        else:
            start_time = time.time()
            success = run_trial(obstacles = obstacles, trial_idx=t, save_folder="trial_results")
            end_time = time.time()

            runtimes.append(end_time - start_time)
            successes.append(success)
            print(f"Trial {t+1}: {runtimes[-1]:.3f} sec, Success={success}")

    avg_runtime = sum(runtimes) / num_trials
    success_rate = sum(successes) / num_trials * 100
    print(f"\nAverage runtime over {num_trials} trials: {avg_runtime:.3f} sec")
    print(f"Success rate: {success_rate:.1f}%")
    print("Final maps saved to: trial_results/")
