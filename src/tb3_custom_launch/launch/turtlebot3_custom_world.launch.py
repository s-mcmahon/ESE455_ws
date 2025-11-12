import os
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def launch_setup(context: LaunchContext, *args, **kwargs):
    world        = LaunchConfiguration('world').perform(context)
    use_nav2     = LaunchConfiguration('use_nav2').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    use_slam     = LaunchConfiguration('use_slam').perform(context)
    use_rviz     = LaunchConfiguration('use_rviz').perform(context)
    x_pose       = LaunchConfiguration('x_pose').perform(context)
    y_pose       = LaunchConfiguration('y_pose').perform(context)

    if not os.path.exists(world):
        raise FileNotFoundError(f"World file not found: {world}")

    def safe_pkg(pkg_name):
        path = FindPackageShare(pkg_name).perform(context)
        if not path or not os.path.exists(path):
            raise FileNotFoundError(f"Package not found: {pkg_name}")
        print(f"[INFO] Found package {pkg_name}: {path}")
        return path

    # Required packages
    gazebo_ros_path       = safe_pkg('gazebo_ros')
    tb3_gazebo_path       = safe_pkg('turtlebot3_gazebo')
    nav2_bringup_path     = safe_pkg('nav2_bringup')

    # Gazebo world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_path, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    # Robot state publisher
    robot_state_pub = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_path, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Spawn TurtleBot3 
    spawn_tb3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_path, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items()
    )

    # Scan Relay
    laser_relay = Node(
        package='topic_tools',
        executable='relay',
        name='scan_relay',
        arguments=['/turtlebot3_laserscan', '/scan'],
        output='screen'
    )

    # Map + Params
    default_map_path = os.path.join(os.environ['HOME'], 'map.yaml')
    if not os.path.exists(default_map_path):
        default_map_path = os.path.join(nav2_bringup_path, 'maps', 'turtlebot4_world.yaml')
        print(f"[INFO] Using bundled map: {default_map_path}")
    else:
        print(f"[INFO] Using user map: {default_map_path}")

    params_file = PathJoinSubstitution([
        FindPackageShare('nav2_bringup'),
        'params',
        'nav2_params.yaml'
    ])

    # Nav2 bringup
    nav2_args = {
        'use_sim_time': use_sim_time,
        'params_file': params_file,
    }

    if use_slam.lower() == 'true':
        print('[INFO] Running Nav2 in SLAM mode (slam_toolbox).')
        nav2_args['slam'] = 'True'
        nav2_args['map'] = ''  
    else:
        print('[INFO] Running Nav2 in localization mode (AMCL + map).')
        nav2_args['slam'] = 'False'
        nav2_args['map'] = default_map_path

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_path, 'launch', 'bringup_launch.py')
        ),
        condition=IfCondition(use_nav2),
        launch_arguments=nav2_args.items()
    )

    # RViz
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_path, 'launch', 'rviz_launch.py')
        ),
        condition=IfCondition(use_rviz),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Action order
    actions = [gazebo, robot_state_pub, spawn_tb3, laser_relay, nav2, rviz]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=os.path.expanduser('~/ESE455_ws/worlds/env4.world'),
            description='Full path to Gazebo world file'
        ),
        DeclareLaunchArgument('x_pose', default_value='0'),
        DeclareLaunchArgument('y_pose', default_value='0'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_nav2', default_value='true', description='Enable Navigation2'),
        DeclareLaunchArgument('use_slam', default_value='false', description='Use slam_toolbox (SLAM) instead of AMCL'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='Launch RViz'),
        OpaqueFunction(function=launch_setup)
    ])
