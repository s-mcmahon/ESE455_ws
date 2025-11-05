#!/usr/bin/env bash

WS=~/ESE455_ws
WORLD=$WS/worldgen/worlds/env1_10.world

gnome-terminal -- bash -c "
cd $WS;
colcon build --symlink-install;
source install/setup.bash;
export TURTLEBOT3_MODEL=burger;
ros2 launch tb3_custom_launch turtlebot3_custom_world.launch.py world:=$WORLD;
exec bash
"

gnome-terminal -- bash -c "
source $WS/install/setup.bash;
ros2 run obstacle_detection obstacle_detection;
exec bash
"

gnome-terminal -- bash -c "
source /opt/ros/humble/setup.bash;
source $WS/install/setup.bash;
ros2 run unknown_rrt rrt_planner $WORLD;
exec bash
"
