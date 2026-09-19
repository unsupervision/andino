# The robot's LIDAR, namespaced. rplidar.launch.py already uses relative names throughout, so a
# pushed namespace is all it needs: /<ns>/rplidar_node publishing /<ns>/scan.
#
#   ros2 launch andino_bringup robot_lidar.launch.py namespace:=mouse
#
# Separate from robot_base.launch.py on purpose: the two talk to different devices and fail
# differently, so they are separate restart domains. An RPLIDAR A1 whose handshake hangs should be
# restartable without resetting the motor controller and zeroing the odometry.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    rplidar_launch = os.path.join(
        get_package_share_directory('andino_bringup'), 'launch', 'rplidar.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='',
                              description='ROS namespace for this robot, e.g. "mouse".'),
        GroupAction([
            PushRosNamespace(LaunchConfiguration('namespace')),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(rplidar_launch)),
        ]),
    ])
