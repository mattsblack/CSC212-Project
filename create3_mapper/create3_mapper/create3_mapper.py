#!/usr/bin/env python3
"""
 * @file create3_mapper.py
 * @author Matthew Black
 * @date 2025-04-05
 * 
 * @copyright Copyright (c) 2025
"""

#TODO: fix the bug where the map doesn't update properly in RViz
#also begin implementing path coverage. try getting a screenshot of the map and then computing the free space in the blank map and then comparing it to the map with the path on top. 
#or find when an algorithm finishes running and after that compute the percentage of the room covered.

# patch numpy to restore np.float and np.maximum_sctype if missing.
import numpy as np
if not hasattr(np, "float"):
    np.float = float
if not hasattr(np, "maximum_sctype"):
    def maximum_sctype(t):
        if t is np.float:
            return np.float64
        elif t is np.int:
            return np.int64
        else:
            return np.dtype(t).type
    np.maximum_sctype = maximum_sctype

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import TransformStamped, Point, PolygonStamped, Point32
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Time
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
import tf_transformations
import numpy as np

class Create3Mapper(Node):
    """
    A ROS2 node that tracks and visualizes a robot's path, computes map coverage,
    and manages coordinate transformations between frames.
    
    This node subscribes to odometry and map data to track the robot's position
    and calculate coverage statistics. It publishes path visualization markers
    and necessary transforms for visualization in RViz.
    """
    
    def __init__(self):
        """
        Initialize the Create3Mapper node.
        
        Sets up subscribers, publishers, transform handlers, and timers needed for
        path tracking and map coverage calculation.
        """
        super().__init__('create3_mapper')
        # Store start time (nanoseconds)
        self.start_time = self.get_clock().now().nanoseconds

        # TF: set up buffer and listener to get map->odom (published by SLAM Toolbox)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribe to odometry (best effort to match the robot's publisher)
        odom_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            odom_qos
        )
        # Subscribe to /map occupancy grid to compute coverage.
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        self.latest_map = None

        # Publisher for a Marker that draws the robot's path as a line strip.
        self.marker_pub = self.create_publisher(Marker, 'robot_path_marker', 10)
        self.path_points = []

        # Transform broadcaster for publishing computed transforms.
        self.br = TransformBroadcaster(self)

        # Timer to output coverage stats every 5 seconds.
        """ self.timer_output = self.create_timer(5.0, self.print_coverage)"""

    def odom_callback(self, msg: Odometry):
        """
        Process incoming odometry messages to track the robot's path.
        
        This callback extracts position data from odometry messages, adds points
        to the path trajectory, detects jumps in position, and triggers path
        visualization and transform publishing.
        
        @param msg: The odometry message containing robot pose information
        @type msg: nav_msgs.msg.Odometry
        """
        # Extract position (assumed to be odom->base_footprint) from odometry.
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # If a large jump is detected (e.g. when resetting), clear the path.
        if self.path_points:
            last_x, last_y = self.path_points[-1]
            if math.hypot(x - last_x, y - last_y) > 1.0:  # 1.0m threshold for jump detection
                self.get_logger().info("Large jump detected; resetting path.")
                self.path_points = []
        self.path_points.append((x, y))
        
        # Limit path history to prevent memory issues with long runs
        # removed for now
        """
        if len(self.path_points) > 2000:
            self.path_points.pop(0)
        """
        # Publish the line strip marker (robot path) using the latest odom header stamp.
        self.publish_path_marker(msg.header.stamp)
        # Compute and broadcast transforms: map->base_footprint and map->laser_frame.
        self.publish_dynamic_transforms(msg)

    def map_callback(self, msg: OccupancyGrid):
        """
        Store the latest map data for coverage calculations.
        
        This callback simply saves the most recent occupancy grid message
        to be used later for map coverage calculations.
        
        @param msg: The occupancy grid message containing map data
        @type msg: nav_msgs.msg.OccupancyGrid
        """
        self.latest_map = msg

    def publish_path_marker(self, stamp):
        """
        Create and publish a visualization marker showing the robot's path.
        
        Uses the collected path points to generate a LINE_STRIP marker that
        visualizes the robot's trajectory in RViz.
        
        @param stamp: Timestamp to use for the marker message
        @type stamp: builtin_interfaces.msg.Time
        """
        if not self.path_points:
            return  # Don't publish empty markers

        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = 'map'  # Path will be shown in the map frame.
        marker.ns = 'robot_path'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.03  # Line width.
        # Set color to blue.
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        # initialize marker point list explicitly
        marker.points = []

        # Convert stored path points into marker points
        for (px, py) in self.path_points:
            p = Point()
            p.x = px
            p.y = py
            p.z = 0.0  # Keep path flat in 2D plane
            marker.points.append(p)

        self.marker_pub.publish(marker)

    def publish_dynamic_transforms(self, odom_msg: Odometry):
        """
        Compute and publish coordinate transformations between frames.
        
        Calculates and broadcasts the transforms from map to base_footprint and
        from map to laser_frame using the latest odometry and map->odom transform.
        
        @param odom_msg: The odometry message containing robot pose information
        @type odom_msg: nav_msgs.msg.Odometry
        """
        try:
            # Get the transform from map to odom (published by SLAM toolbox)
            now = rclpy.time.Time()
            t_map_odom = self.tf_buffer.lookup_transform(
                'map', 'odom', now, timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except Exception as e:
            self.get_logger().warn("Could not lookup transform from map to odom: %s" % str(e))
            return

        # Build transformation matrix for map->odom.
        # Extract translation component from transform
        map_to_odom_translation = [
            t_map_odom.transform.translation.x,
            t_map_odom.transform.translation.y,
            t_map_odom.transform.translation.z
        ]
        # Extract rotation component as quaternion
        map_to_odom_quat = [
            t_map_odom.transform.rotation.x,
            t_map_odom.transform.rotation.y,
            t_map_odom.transform.rotation.z,
            t_map_odom.transform.rotation.w
        ]
        # Create 4x4 homogeneous transformation matrix from quaternion
        T_map_odom = tf_transformations.quaternion_matrix(map_to_odom_quat)
        # Insert translation into the matrix
        T_map_odom[0:3, 3] = map_to_odom_translation

        # Get odom->base_footprint transform from odometry message.
        # Similar process as above but for a different transform
        odom_to_base_translation = [
            odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y,
            odom_msg.pose.pose.position.z
        ]
        odom_to_base_quat = [
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w
        ]
        T_odom_base = tf_transformations.quaternion_matrix(odom_to_base_quat)
        T_odom_base[0:3, 3] = odom_to_base_translation

        # Compose: T_map_base = T_map_odom * T_odom_base.
        # Matrix multiplication gives us the combined transform
        T_map_base = np.dot(T_map_odom, T_odom_base)
        map_base_translation = T_map_base[0:3, 3]
        map_base_quat = tf_transformations.quaternion_from_matrix(T_map_base)

        # Broadcast transform: map -> base_footprint.
        t_mb = TransformStamped()
        t_mb.header.stamp = odom_msg.header.stamp
        t_mb.header.frame_id = 'map'
        t_mb.child_frame_id = 'base_footprint'
        t_mb.transform.translation.x = map_base_translation[0]
        t_mb.transform.translation.y = map_base_translation[1]
        t_mb.transform.translation.z = map_base_translation[2]
        t_mb.transform.rotation.x = map_base_quat[0]
        t_mb.transform.rotation.y = map_base_quat[1]
        t_mb.transform.rotation.z = map_base_quat[2]
        t_mb.transform.rotation.w = map_base_quat[3]
        self.br.sendTransform(t_mb)

        # Compute map->laser_frame using a static transform from base_footprint.
        # This defines the relative position of the laser sensor to the robot base
        T_base_laser = np.eye(4)  # Identity matrix = no offset by default
        # Example: if the laser is 0.1 m forward relative to base_footprint, uncomment:
        # T_base_laser[0, 3] = 0.1
        
        # Chain transformations to get map->laser transformation
        T_map_laser = np.dot(T_map_base, T_base_laser)
        map_laser_translation = T_map_laser[0:3, 3]
        map_laser_quat = tf_transformations.quaternion_from_matrix(T_map_laser)

        # Create and broadcast the map->laser_frame transform
        t_ml = TransformStamped()
        t_ml.header.stamp = odom_msg.header.stamp
        t_ml.header.frame_id = 'map'
        t_ml.child_frame_id = 'laser_frame'
        t_ml.transform.translation.x = map_laser_translation[0]
        t_ml.transform.translation.y = map_laser_translation[1]
        t_ml.transform.translation.z = map_laser_translation[2]
        t_ml.transform.rotation.x = map_laser_quat[0]
        t_ml.transform.rotation.y = map_laser_quat[1]
        t_ml.transform.rotation.z = map_laser_quat[2]
        t_ml.transform.rotation.w = map_laser_quat[3]
        self.br.sendTransform(t_ml)
        
        
    '''
    Commented out because of issues with the coverage statistics
    
    
    def print_coverage(self):
        """
        Calculate and log map coverage statistics.
        
        Computes and outputs the duration of operation, map coverage percentage,
        and total distance traveled by the robot. This method is called periodically
        by a timer.
        
        @return: None
        """
        # Compute elapsed duration (in seconds).
        current_time_ns = self.get_clock().now().nanoseconds
        duration = (current_time_ns - self.start_time) / 1e9  # Convert nanoseconds to seconds

        # Compute distance covered along the path.
        distance = 0.0
        if len(self.path_points) >= 2:
            for i in range(1, len(self.path_points)):
                # Calculate Euclidean distance between consecutive points
                dx = self.path_points[i][0] - self.path_points[i-1][0]
                dy = self.path_points[i][1] - self.path_points[i-1][1]
                distance += math.hypot(dx, dy)  # hypot = sqrt(dx² + dy²)

        # Compute map coverage percentage using the latest occupancy grid.
        coverage = 0.0
        if self.latest_map is not None:
            data = self.latest_map.data
            # Count free cells (value 0) and known cells (not -1 which means unknown)
            free = sum(1 for v in data if v == 0)
            known = sum(1 for v in data if v != -1)
            if known > 0:
                coverage = (free / known) * 100.0  # Percentage of known space that is free

        self.get_logger().info(
            "Duration: %.1f s, Map Coverage: %.1f%%, Distance Covered: %.2f m" %
            (duration, coverage, distance)
        )
        '''

def main(args=None):
    """
    Main entry point for the Create3Mapper node.
    
    Initializes the ROS2 client library, creates the node instance,
    and handles the main execution loop and shutdown. Catches keyboard
    interrupts (Ctrl+C) to ensure clean shutdown without stack traces.
    
    @param args: Command line arguments
    @type args: list
    @return: None
    """
    rclpy.init(args=args)
    node = Create3Mapper()
    
    try:
        # Start the node's execution
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        node.get_logger().info('Keyboard interrupt detected. Shutting down...')
    except Exception as e:
        # Handle other exceptions but still clean up
        node.get_logger().error(f'Error occurred: {str(e)}')
    finally:
        # Clean up resources in all cases
        node.destroy_node()
        rclpy.shutdown()
    
    # Log successful shutdown completion
    print('Create3Mapper node shutdown successfully.')

if __name__ == '__main__':
    main()

