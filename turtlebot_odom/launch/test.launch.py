import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    node_cmd = Node(
        package='turtlebot_waypoint_controller',
        executable='turtlebot_waypoint_controller_node',
        name='node_name',
        output='screen'
    )

    ld = LaunchDescription()

    for i in range(10)
        ld.add_action(node_cmd)
    
    return ld