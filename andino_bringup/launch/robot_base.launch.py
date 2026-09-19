# The robot's BASE, namespaced: robot_state_publisher + ros2_control + the two controllers + the
# cmd_vel relay. For running several robots in one ROS graph.
#
#   ros2 launch andino_bringup robot_base.launch.py namespace:=mouse
#
# It does what andino_description + andino_control do together, with two differences that a
# namespace makes unavoidable:
#   * every name is RELATIVE. andino_control.launch.py addresses `/controller_manager` and remaps
#     `/diff_controller/cmd_vel` absolutely, so pushing a namespace onto it silently disconnects
#     the controllers from their manager and cmd_vel from the controller.
#   * the URDF comes straight from xacro, not from `ros2 param get /robot_state_publisher ...`,
#     which is an absolute name and a start-order dependency besides.
#
# TF follows the Nav2 multi-robot convention: the TOPICS are namespaced (/<ns>/tf,
# /<ns>/tf_static) and the FRAMES are not (odom, base_link). A consumer then needs a namespace and
# these two remaps — not a renamed copy of every frame in every config.

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TF_REMAPPINGS = [('/tf', 'tf'), ('/tf_static', 'tf_static')]


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    pkg_description = get_package_share_directory('andino_description')
    doc = xacro.process_file(
        os.path.join(pkg_description, 'urdf', 'andino.urdf.xacro'),
        mappings={'yaml_config_dir': os.path.join(pkg_description, 'config', 'andino')},
    )
    robot_description = {'robot_description': doc.toprettyxml(indent='  ')}
    controllers = os.path.join(
        get_package_share_directory('andino_control'), 'config', 'andino_controllers.yaml')

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        output='both',
        parameters=[robot_description, {'publish_frequency': 30.0}],
        remappings=TF_REMAPPINGS,
    )

    # Relative remap rules are expanded against each node's namespace, so inside /<ns> the first
    # one reads /<ns>/diff_controller/cmd_vel -> /<ns>/cmd_vel_stamped. The TF remaps are needed
    # here too: diff_drive_controller publishes odom -> base_link on the absolute `/tf`.
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace=namespace,
        output='both',
        parameters=[robot_description, controllers],
        remappings=[
            ('diff_controller/cmd_vel', 'cmd_vel_stamped'),
            ('diff_controller/cmd_vel_out', 'cmd_vel_out'),
            ('diff_controller/odom', 'odom'),
        ] + TF_REMAPPINGS,
    )

    # A relative --controller-manager is resolved by the spawner against ITS OWN namespace.
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        arguments=['joint_state_broadcaster', '--controller-manager', 'controller_manager'],
    )
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        arguments=['diff_controller', '--controller-manager', 'controller_manager'],
    )
    diff_after_joint_state = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )

    # Twist -> TwistStamped, as in andino_control: teleop and Nav2 publish the former, Jazzy's
    # diff_drive_controller wants the latter.
    relay_node = Node(
        package='topic_tools',
        executable='relay_field',
        name='cmd_vel_relay',
        namespace=namespace,
        arguments=['cmd_vel', 'cmd_vel_stamped', 'geometry_msgs/TwistStamped',
                   "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, twist: m}",
                   '--wait-for-start'],
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='',
                              description='ROS namespace for this robot, e.g. "mouse".'),
        rsp,
        control_node,
        joint_state_broadcaster_spawner,
        diff_after_joint_state,
        relay_node,
    ])
