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
    """
    Generate a LaunchDescription for starting RViz2 with the create3_mapper configuration.
    
    This function creates a launch description that configures and starts RViz2 with
    the appropriate settings for visualizing the create3_mapper node output. It sets up
    topic remappings to ensure TF frames work correctly with robot namespaces and loads
    a predefined RViz configuration file.
    
    The function supports an optional namespace parameter that can be provided at launch
    time to match the robot's namespace if one is being used.
    
    @return: A LaunchDescription object containing all the necessary actions to launch RViz2
    @rtype: launch.LaunchDescription
    """
    # Get the package share directory for create3_mapper
    # This finds the installed location of our package to access config files
    mapper_pkg = get_package_share_directory('create3_mapper')
    
    # Build the path to the RViz configuration file
    # PathJoinSubstitution ensures proper path joining across different platforms
    rviz_config = PathJoinSubstitution(
        [mapper_pkg, 'rviz', 'create3_mapper.rviz']
    )

    # Allow namespace to be passed in at launch time.
    # This enables running multiple robots, each in their own namespace
    namespace = LaunchConfiguration('namespace')
    namespace_argument = DeclareLaunchArgument(
        'namespace', 
        default_value='',
        description='Robot namespace'
    )

    # Launch the rviz2 node with our configuration and remappings for tf topics.
    # The remappings ensure TF frames are properly received when using namespaces
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],  # -d flag loads the specified config file
        remappings=[('/tf_static', 'tf_static'), ('/tf', 'tf')],  # Map TF topics to work with namespaces
        namespace=namespace,  # Place RViz in the same namespace as the robot if specified
        output='screen'  # Show RViz output in the console for debugging
    )

    # Create and populate the LaunchDescription
    # This is the return object that the ROS 2 launch system will execute
    ld = LaunchDescription()
    ld.add_action(namespace_argument)  # Register the namespace argument
    ld.add_action(rviz_node)  # Add the RViz node to the launch description
    return ld

