# The robot's STATE ESTIMATE, namespaced: wheel odometry + IMU -> odom -> base_link.
#
#   ros2 launch andino_bringup robot_ekf.launch.py namespace:=mouse
#
# The third piece beside robot_base.launch.py (which publishes `odom` and `imu/data` and, in this
# deployment, no odom TF of its own) and robot_lidar.launch.py. It runs ON the robot, in its own
# container: it is light, and the transform every consumer hangs off should not depend on the
# network. Output: `odometry/filtered`, and odom -> base_link on the robot's TF topic.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Same per-robot TF topics as robot_base.launch.py; the frames themselves carry no prefix.
TF_REMAPPINGS = [('/tf', 'tf'), ('/tf_static', 'tf_static')]


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    config = os.path.join(get_package_share_directory('andino_bringup'), 'config', 'ekf.yaml')

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        namespace=namespace,
        output='both',
        parameters=[config],
        remappings=TF_REMAPPINGS,
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='',
                              description='ROS namespace for this robot, e.g. "mouse".'),
        ekf,
    ])
