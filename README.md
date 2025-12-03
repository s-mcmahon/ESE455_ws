# How to Run this Program: 

Open 3 terminal windows: 

1. Run the following (SIM): 
```bash
cd ESE455_ws/
colcon build --symlink-install
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch tb3_custom_launch  tb3_nav_offboard.launch.py use_slam:=True use_nav2:=True
#ros2 launch tb3_custom_launch turtlebot3_custom_world.launch.py \
    world:=/home/$USER/ESE455_ws/worlds/env1.world \
    x_pose:=-0 y_pose:=-0 \
    use_nav2:=true use_slam:=true

```
2. Run the following (Planner): 
```bash
source ~/ESE455_ws/install/setup.bash

ros2 run rrt_nav rrt_nav_node \
  --ros-args \
  -p use_sim_time:=true \
  -p unknown_is_free:=true \
  -p step_size:=0.3 \
  -p min_waypoint_separation:=0.5 \
  -p goal_x:=3.8 \  # -2.4 for world 1 
  -p goal_y:=3.8 \  # 3.7 for world 1
  -p goal_radius:=0.5

#or 
#ros2 run rrt_nav kino_rrt_nav_node \
  --ros-args \
  -p use_sim_time:=true \
  -p unknown_is_free:=true \
  -p step_time:=0.5 \
  -p dt:=0.2 \
  -p v_max:=0.25 \
  -p w_max:=1.82 \
  -p goal_x:=3.8 \ # -2.4 for world 1 
  -p goal_y:=3.8 \ # 3.7 for world 1
  -p goal_radius:=0.5 \
  -p min_waypoint_separation:=0.6

```

3. Run the following (Save Metrics): 
```bash
source ~/ESE455_ws/install/setup.bash
ros2 run rrt_nav rrt_metrics_node
```

# How to Run this Program (on robot): 

Open 3 terminal windows: 

1. Run the following (SIM): 
```bash
cd ESE455_ws/
colcon build --symlink-install
source install/setup.bash
ros2 run rrt_nav onboard_kino_rrt_node   --ros-args     -p use_sim_time:=false      -p v_max:=0.25     -p w_max:=1.82     -p goal_x:=5.5     -p goal_y:=5.0
```
2. Run the following (Planner): 
```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch nav2_bringup navigation_launch.py     use_sim_time:=False     autostart:=True      localization:=False
```

3. Run the following (Save Metrics): 
```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=False

```
