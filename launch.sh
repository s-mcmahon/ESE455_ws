#!/usr/bin/env bash

WS=~/ESE455_ws
WORLD=$WS/worlds/env1.world

# Terminal 1: Launch Simulation
gnome-terminal -- bash -c "
cd $WS;
colcon build --symlink-install;
source install/setup.bash;
export TURTLEBOT3_MODEL=waffle;
ros2 launch tb3_custom_launch turtlebot3_custom_world.launch.py \
    world:=$WORLD \
    x_pose:=0 y_pose:=0 \
    use_nav2:=true use_slam:=true;
exec bash
"

# Terminal 2: Launch RRT Planner
gnome-terminal -- bash -c "
source $WS/install/setup.bash;
ros2 run rrt_nav rrt_nav_node \
  --ros-args \
  -p use_sim_time:=true \
  -p unknown_is_free:=true \
  -p step_size:=0.3 \
  -p min_waypoint_separation:=0.5 \
  -p goal_x:=3.8 \
  -p goal_y:=3.8 \
  -p goal_radius:=0.5;
exec bash
"

# Terminal 3: Launch Metrics Node
gnome-terminal -- bash -c "
source $WS/install/setup.bash;
ros2 run rrt_nav rrt_metrics_node;
exec bash
"

# Uncomment below and comment out Terminal 2 above to use kinodynamic planner instead:
# gnome-terminal -- bash -c "
# source $WS/install/setup.bash;
# ros2 run rrt_nav kino_rrt_nav_node \
#   --ros-args \
#   -p use_sim_time:=true \
#   -p unknown_is_free:=true \
#   -p step_time:=0.5 \
#   -p dt:=0.2 \
#   -p v_max:=0.25 \
#   -p w_max:=1.82 \
#   -p goal_x:=3.8 \
#   -p goal_y:=3.8 \
#   -p goal_radius:=0.5 \
#   -p min_waypoint_separation:=0.6;
# exec bash
# "