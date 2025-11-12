#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('goal_x', default_value='9.0'),
        DeclareLaunchArgument('goal_y', default_value='9.0'),
        DeclareLaunchArgument('goal_box', default_value='0.5'),
        DeclareLaunchArgument('robot_radius', default_value='0.18'),
        DeclareLaunchArgument('occ_threshold', default_value='65'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        Node(
            package='rrt_nav',
            executable='rrt_nav_node',
            name='rrt_navigator',
            output='screen',
            parameters=[{
                'goal_x': LaunchConfiguration('goal_x'),
                'goal_y': LaunchConfiguration('goal_y'),
                'goal_box': LaunchConfiguration('goal_box'),
                'robot_radius': LaunchConfiguration('robot_radius'),
                'occ_threshold': LaunchConfiguration('occ_threshold'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }]
        )
    ])
