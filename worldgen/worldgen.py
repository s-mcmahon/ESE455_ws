#!/usr/bin/env python3
"""
worldgen.py — Generate Gazebo Classic worlds at ~10/20/30/40% obstacle coverage,
pick start/goal, and expose a CLI to print coordinates and world paths.

Usage examples:
  python3 worldgen.py --generate                  # (re)generate all .world files + manifest
  python3 worldgen.py --env env1_10 --print-start # prints "X Y" start (Gazebo frame)
  python3 worldgen.py --env env1_10 --print-world-path
  python3 worldgen.py --ensure --env env1_10      # generate if missing
"""

import math, random, os, json, argparse, sys

random.seed(42)

MAP_SIZE = 10.0
AREA_TOTAL = MAP_SIZE * MAP_SIZE

GOAL_RADIUS = 0.8  # ~2% of map
ROBOT_CLEARANCE = 0.25
MIN_BOX = 0.6
MAX_BOX = 2.0
MAX_TRIES = 4000

TARGETS = {
    "env1_10": 0.10,
    "env2_20": 0.20,
    "env3_30": 0.30,
    "env4_40": 0.40,
}

OUTDIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "worlds"))
MANIFEST = os.path.join(OUTDIR, "manifest.json")

def rects_overlap(a, b, pad=0.0):
    return not (a[2] + pad <= b[0] or b[2] + pad <= a[0] or
                a[3] + pad <= b[1] or b[3] + pad <= a[1])

def line_intersects_rect(p1, p2, rect):
    (x1,y1), (x2,y2) = p1, p2
    xmin, ymin, xmax, ymax = rect
    edges = [((xmin,ymin),(xmax,ymin)), ((xmax,ymin),(xmax,ymax)),
             ((xmax,ymax),(xmin,ymax)), ((xmin,ymax),(xmin,ymin))]

    def ccw(A,B,C):
        return (C[1]-A[1])*(B[0]-A[0]) > (B[1]-A[1])*(C[0]-A[0])
    def seg_intersect(A,B,C,D):
        return (ccw(A,C,D) != ccw(B,C,D)) and (ccw(A,B,C) != ccw(A,B,D))

    for e in edges:
        if seg_intersect((x1,y1),(x2,y2), e[0], e[1]):
            return True
    mx, my = (x1+x2)/2.0, (y1+y2)/2.0
    if xmin <= mx <= xmax and ymin <= my <= ymax:
        return True
    return False

def point_in_rect(pt, rect, pad=0.0):
    x,y = pt
    xmin,ymin,xmax,ymax = rect
    return (xmin - pad) <= x <= (xmax + pad) and (ymin - pad) <= y <= (ymax + pad)

def make_environment(target_fraction):
    rects, areas = [], []
    total_area = 0.0
    target_area = target_fraction * AREA_TOTAL
    tol = 0.02 * AREA_TOTAL  # absolute 2% of map

    tries = 0
    while total_area < target_area - tol and tries < MAX_TRIES:
        tries += 1
        w = random.uniform(MIN_BOX, MAX_BOX)
        h = random.uniform(MIN_BOX, MAX_BOX)
        x0 = random.uniform(0.1, MAP_SIZE - w - 0.1)
        y0 = random.uniform(0.1, MAP_SIZE - h - 0.1)
        new_rect = (x0, y0, x0 + w, y0 + h)
        if any(rects_overlap(new_rect, r, pad=0.05) for r in rects):
            continue
        rects.append(new_rect)
        areas.append(w*h)
        total_area += w*h

    while total_area > target_area + tol and rects:
        r = rects.pop()
        total_area -= areas.pop()

    return rects, total_area

def pick_start_goal(rects):
    def valid_pose(p):
        x,y = p
        if x < ROBOT_CLEARANCE or x > MAP_SIZE-ROBOT_CLEARANCE: return False
        if y < ROBOT_CLEARANCE or y > MAP_SIZE-ROBOT_CLEARANCE: return False
        for r in rects:
            if point_in_rect(p, r, pad=ROBOT_CLEARANCE):
                return False
        return True

    for _ in range(2000):
        start = (random.uniform(0.8, MAP_SIZE-0.8), random.uniform(0.8, MAP_SIZE-0.8))
        if not valid_pose(start): 
            continue
        goal_center = (random.uniform(0.8, MAP_SIZE-0.8), random.uniform(0.8, MAP_SIZE-0.8))
        if not valid_pose(goal_center): 
            continue
        if math.dist(start, goal_center) < 3.0:
            continue
        gx, gy = goal_center
        if gx - GOAL_RADIUS < 0 or gx + GOAL_RADIUS > MAP_SIZE: 
            continue
        if gy - GOAL_RADIUS < 0 or gy + GOAL_RADIUS > MAP_SIZE: 
            continue
        overlaps = any(rects_overlap((gx-GOAL_RADIUS, gy-GOAL_RADIUS, gx+GOAL_RADIUS, gy+GOAL_RADIUS), r, pad=0.0) for r in rects)
        if overlaps: 
            continue
        if any(line_intersects_rect(start, goal_center, r) for r in rects):
            return start, goal_center
    return None, None

def sdf_world(name, rects, start, goal_center):
    boxes_sdf = []
    for idx, (xmin, ymin, xmax, ymax) in enumerate(rects):
        w = xmax - xmin
        h = ymax - ymin
        cx = xmin + w/2.0
        cy = ymin + h/2.0
        boxes_sdf.append(f"""
      <model name="obstacle_{idx}">
        <static>true</static>
        <pose>{cx - MAP_SIZE/2:.3f} {cy - MAP_SIZE/2:.3f} 0.5 0 0 0</pose>
        <link name="link">
          <collision name="collision">
            <geometry><box><size>{w:.3f} {h:.3f} 1.0</size></box></geometry>
          </collision>
          <visual name="visual">
            <geometry><box><size>{w:.3f} {h:.3f} 1.0</size></box></geometry>
            <material><ambient>0.3 0.3 0.3 1</ambient></material>
          </visual>
        </link>
      </model>
        """)
    gx, gy = goal_center
    goal_sdf = f"""
      <model name="goal_region">
        <static>true</static>
        <pose>{gx - MAP_SIZE/2:.3f} {gy - MAP_SIZE/2:.3f} 0.025 0 0 0</pose>
        <link name="link">
          <visual name="visual">
            <geometry><cylinder><radius>{GOAL_RADIUS:.3f}</radius><length>0.05</length></cylinder></geometry>
            <material><ambient>0 1 0 0.6</ambient><diffuse>0 1 0 0.6</diffuse></material>
          </visual>
        </link>
      </model>
    """
    sx, sy = start
    start_marker = f"""
      <model name="start_marker">
        <static>true</static>
        <pose>{sx - MAP_SIZE/2:.3f} {sy - MAP_SIZE/2:.3f} 0.05 0 0 0</pose>
        <link name="link">
          <visual name="visual">
            <geometry><cylinder><radius>0.15</radius><length>0.1</length></cylinder></geometry>
            <material><ambient>0 0 1 0.8</ambient><diffuse>0 0 1 0.8</diffuse></material>
          </visual>
        </link>
      </model>
    """
    ground = f"""
    <model name="ground_plane_custom">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>{MAP_SIZE} {MAP_SIZE}</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>{MAP_SIZE} {MAP_SIZE}</size></plane></geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
      </link>
      <pose>0 0 0 0 0 0</pose>
    </model>
    """
    world = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="{name}">
    <gravity>0 0 -9.8</gravity>
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>
    {ground}
    {"".join(boxes_sdf)}
    {goal_sdf}
    {start_marker}
  </world>
</sdf>
"""
    return world

def generate_all():
    os.makedirs(OUTDIR, exist_ok=True)
    results = {}
    for env_name, frac in TARGETS.items():
        # try a few times to get valid start/goal
        for _ in range(10):
            rects, area = make_environment(frac)
            start, goal = pick_start_goal(rects)
            if start is not None:
                break
        if start is None:
            raise RuntimeError(f"Could not place start/goal for {env_name}")
        xml = sdf_world(env_name, rects, start, goal)
        path = os.path.join(OUTDIR, f"{env_name}.world")
        with open(path, "w") as f:
            f.write(xml)
        results[env_name] = {
            "world_file": path,
            "target_fraction": frac,
            "achieved_fraction": area/AREA_TOTAL,
            "start_design_xy": list(start),         # [0..10] frame
            "goal_design_xy": list(goal),           # [0..10] frame
            "start_world_xy": [start[0]-5.0, start[1]-5.0],  # Gazebo world frame
            "goal_world_xy": [goal[0]-5.0, goal[1]-5.0],
            "goal_area_pct": (math.pi*GOAL_RADIUS*GOAL_RADIUS)/AREA_TOTAL
        }
    with open(MANIFEST, "w") as f:
        json.dump(results, f, indent=2)
    return results

def load_manifest():
    if not os.path.exists(MANIFEST):
        return None
    with open(MANIFEST, "r") as f:
        return json.load(f)

def ensure_env(env):
    m = load_manifest()
    if m is None or env not in m or not os.path.exists(m[env]["world_file"]):
        m = generate_all()
    return m

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--generate", action="store_true", help="(re)generate all worlds and manifest")
    ap.add_argument("--ensure", action="store_true", help="generate if missing")
    ap.add_argument("--env", choices=list(TARGETS.keys()), help="environment key")
    ap.add_argument("--print-start", action="store_true", help="print start X Y (Gazebo world frame)")
    ap.add_argument("--print-world-path", action="store_true", help="print absolute world file path")
    ap.add_argument("--print-summary", action="store_true", help="print JSON summary for env")
    args = ap.parse_args()

    if args.generate:
        m = generate_all()
        print(json.dumps(m, indent=2))
        return

    if args.ensure and args.env:
        m = ensure_env(args.env)
    else:
        m = load_manifest()

    if args.env and m and args.env in m:
        env = m[args.env]
        if args.print_start:
            x, y = env["start_world_xy"]
            print(f"{x} {y}")
            return
        if args.print_world_path:
            print(env["world_file"])
            return
        if args.print_summary:
            print(json.dumps(env, indent=2))
            return

    if not any([args.generate, args.print_start, args.print_world_path, args.print_summary]):
        # Default behavior: generate if manifest missing, then print summary
        if m is None:
            m = generate_all()
        print(json.dumps(m, indent=2))

if __name__ == "__main__":
    main()
