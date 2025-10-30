# How to Run this Program: 

Open 3 terminal windows: 

1. Run the following (SIM): 
```bash
cd ESE455_ws/
colcon build --symlink-install
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch tb3_custom_launch turtlebot3_custom_world.launch.py world:=../ESE455_ws/worldgen/worlds/env1_10.world

```
2. Run the following (SLAM): 
```bash
 export TURTLEBOT3_MODEL=burger
 ros2 launch turtlebot3_bringup robot.launch.py
```
3. Run the following (RRT): currently this is a waypoint follower for testing 
```bash 
source /opt/ros/humble/setup.bash
source ~/ESE455_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_example turtlebot3_relative_move
```
