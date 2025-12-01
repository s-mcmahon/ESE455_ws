import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Launch args
    use_nav2     = LaunchConfiguration('use_nav2')
    use_slam     = LaunchConfiguration('use_slam')
    use_rviz     = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # This argument forces SLAM mode (disables AMCL + map_server)
    map_yaml = LaunchConfiguration('map')

    # ----------------------------------------------------------------------
    # PACKAGE PATHS
    # ----------------------------------------------------------------------
    nav2_bringup_path = FindPackageShare('nav2_bringup')
    slam_toolbox_path = FindPackageShare('slam_toolbox')
    tb3_nav_path      = FindPackageShare('turtlebot3_navigation2')

    # ----------------------------------------------------------------------
    # PARAM FILE (TB3 Humble Waffle Pi example)
    # ----------------------------------------------------------------------
    params_file = PathJoinSubstitution([
        tb3_nav_path,
        'param',
        'humble',
        'waffle_pi.yaml'
    ])

    # ----------------------------------------------------------------------
    # NAV2 BRINGUP (AMCL OFF since map is "")
    # ----------------------------------------------------------------------
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav2_bringup_path.find('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            )
        ),
        condition=IfCondition(use_nav2),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'map': map_yaml,       # ← THIS DISABLES AMCL WHEN map=""
        }.items()
    )

    # ----------------------------------------------------------------------
    # SLAM TOOLBOX
    # ----------------------------------------------------------------------
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                slam_toolbox_path.find('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )
        ),
        condition=IfCondition(use_slam),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # ----------------------------------------------------------------------
    # RVIZ2
    # ----------------------------------------------------------------------
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav2_bringup_path.find('nav2_bringup'),
                'launch',
                'rviz_launch.py'
            )
        ),
        condition=IfCondition(use_rviz),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # ----------------------------------------------------------------------
    # RETURN FULL LAUNCH DESCRIPTION
    # ----------------------------------------------------------------------
    return LaunchDescription([

        # PUBLIC ARGUMENTS
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_nav2',     default_value='true'),
        DeclareLaunchArgument('use_slam',     default_value='true'),
        DeclareLaunchArgument('use_rviz',     default_value='true'),

        # THE CRITICAL FIX: FORCE SLAM MODE
        DeclareLaunchArgument('map', default_value=''),  # ← disables AMCL + map_server

        # NODES TO START
        slam_launch,
        nav2_launch,
        rviz_launch,
    ])
