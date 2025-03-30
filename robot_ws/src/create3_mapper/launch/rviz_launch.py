#!/usr/bin/env python3
# Copyright 2023 Your Name
#
# This launch file starts rviz2 with a configuration suitable for the create3_mapper package,
# remapping tf topics as needed so that namespaces work correctly.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package share directory of your create3_mapper package
    mapper_pkg = get_package_share_directory('create3_mapper')

    # Generate the path to the rviz configuration file (adjust the filename if necessary)
    rviz_config = PathJoinSubstitution(
        [mapper_pkg, 'rviz', 'create3_mapper.rviz']
    )

    # Get namespace launch configuration
    namespace = LaunchConfiguration('namespace')

    # Declare a launch argument to allow passing a namespace from the CLI
    namespace_argument = DeclareLaunchArgument(
        'namespace', 
        default_value='',
        description='Robot namespace'
    )

    # Launch the rviz2 node with the given configuration and remappings for tf topics
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        remappings=[
            ('/tf_static', 'tf_static'),
            ('/tf', 'tf')
        ],
        namespace=namespace,
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(namespace_argument)
    ld.add_action(rviz_node)

    return ld

