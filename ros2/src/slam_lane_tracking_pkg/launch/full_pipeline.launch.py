#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Arguments
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='burger',
        description='TurtleBot3 model [burger, waffle, waffle_pi]'
    )

    # Set TURTLEBOT3_MODEL environment variable
    set_tb3_model = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL',
        value=LaunchConfiguration('model')
    )

    # === Gazebo + TurtleBot3 world ===
    # tb3_gazebo_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('turtlebot3_gazebo'),
    #             'launch',
    #             'turtlebot3_world.launch.py'
    #         )
    #     )
    # )

    # === SLAM Toolbox (online async) ===
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )
        )
    )

    # === Odom logger node (from slam_lane_tracking_pkg) ===
    odom_logger_node = Node(
        package='slam_lane_tracking_pkg',
        executable='odom_logger',   # from setup.py entry_points
        name='odom_logger',
        output='screen'
    )

    # === Centroid node (you must match this executable name) ===
    # centroid_node = Node(
    #     package='slam_lane_tracking_pkg',
    #     executable='centroid_node',  # change if your entry point is named differently
    #     name='centroid_node',
    #     output='screen'
    # )



    # You can run teleop in a separate terminal (recommended).
    # If you really want it inside launch, uncomment and make sure xterm is installed:
    #
    # teleop = Node(
    #     package='turtlebot3_teleop',
    #     executable='teleop_keyboard',
    #     name='teleop_keyboard',
    #     output='screen',
    #     prefix='xterm -e'  # opens teleop in its own terminal
    # )

    return LaunchDescription([
        model_arg,
        set_tb3_model,
        # tb3_gazebo_launch,
        slam_toolbox_launch,
        odom_logger_node,
        # centroid_node,
        # teleop,
    ])