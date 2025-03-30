#!/usr/bin/env python3
# This launch file starts rviz2 with a configuration that works with the create3_mapper node.
# It remaps tf topics to work with namespaces just like the create3_lidar_slam RViz launch file.

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package share directory for create3_mapper
    mapper_pkg = get_package_share_directory('create3_mapper')
    
    # Build the path to the RViz configuration file
    rviz_config = PathJoinSubstitution(
        [mapper_pkg, 'rviz', 'create3_mapper.rviz']
    )

    # Allow namespace to be passed in at launch time.
    namespace = LaunchConfiguration('namespace')
    namespace_argument = DeclareLaunchArgument(
        'namespace', 
        default_value='',
        description='Robot namespace'
    )

    # Launch the rviz2 node with our configuration and remappings for tf topics.
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        remappings=[('/tf_static', 'tf_static'), ('/tf', 'tf')],
        namespace=namespace,
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(namespace_argument)
    ld.add_action(rviz_node)
    return ld

