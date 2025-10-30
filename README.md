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
source ~/ESE455_ws/install/setup.bash
ros2 run obstacle_detection obstacle_detection
```

3. Run the following (RRT): 
```bash 
source /opt/ros/humble/setup.bash
source ~/ESE455_ws/install/setup.bash
ros2 run unknown_rrt rrt_planner ../ESE455_ws/worldgen/worlds/env1_10.world
```


Obstacles don't save on the occupency grid currently 