#!/usr/bin/env bash

set -e
set +u
source /opt/ros/humble/setup.bash
set -u

ENV="${1:-env1_10}"
export TURTLEBOT3_MODEL=waffle_pi

WORLD="$(python3 worldgen.py --ensure --env "$ENV" --print-world-path)"
read START_X START_Y <<< "$(python3 worldgen.py --env "$ENV" --print-start)"

echo "Using $ENV"
echo "World: $WORLD"
echo "Start: X=$START_X  Y=$START_Y"

ros2 launch gazebo_ros gazebo.launch.py world:="$WORLD" &
GAZEBO_PID=$!
sleep 6
ros2 run gazebo_ros spawn_entity.py -entity tb3 -database turtlebot3_waffle_pi \
  -x "$START_X" -y "$START_Y" -z 0.01 -Y 0.0
wait $GAZEBO_PID
