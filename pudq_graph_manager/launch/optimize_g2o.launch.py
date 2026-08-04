#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    robot_namespace = 'trajectory1'

    # rviz_cmd = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', os.path.join(get_package_share_directory('turtlebot_odom'), 'rviz', 'pgo.rviz')]
    # )

    graph_cmd = Node(
        package='pudq_graph_manager',
        executable='pudq_graph_manager_node',
        namespace=robot_namespace,
        parameters=[{
            'fixed_frame': 'world',
            'map_frame': 'map',
            'g2o_mode': True,
            'g2o_file': os.path.join(get_package_share_directory('pudq_graph_manager'), 'g2o', 'intel.g2o')
        }],
        output='screen'
    )

    ld = LaunchDescription()
    # ld.add_action(rviz_cmd)
    ld.add_action(graph_cmd)

    return ld