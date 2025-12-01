import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    use_nav2     = LaunchConfiguration('use_nav2')
    use_slam     = LaunchConfiguration('use_slam')
    use_rviz     = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Package paths
    nav2_bringup_path = FindPackageShare('nav2_bringup').find('nav2_bringup')
    cartographer_path = FindPackageShare('cartographer_ros').find('cartographer_ros')

    # Nav2 params for REAL robot
    params_file = PathJoinSubstitution([
        FindPackageShare('turtlebot3_navigation2'),
        'param',
        'nav2_params.yaml'
    ])

    # SLAM mode
    nav2_args = {
        'use_sim_time': use_sim_time,
        'params_file': params_file,
    }

    # Cartographer config
    cartographer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cartographer_path, 'launch', 'cartographer.launch.py')
        ),
        condition=IfCondition(use_slam),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'configuration_directory': PathJoinSubstitution([
                FindPackageShare('turtlebot3_cartographer'),
                'config'
            ]),
            'configuration_basename': 'turtlebot3_lds_2d.lua',
        }.items()
    )

    # Nav2 bringup (AMCL when NOT using SLAM)
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

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_nav2',     default_value='true'),
        DeclareLaunchArgument('use_slam',     default_value='true',  description='Run Cartographer'),
        DeclareLaunchArgument('use_rviz',     default_value='true'),

        cartographer_launch,
        nav2,
        rviz
    ])
