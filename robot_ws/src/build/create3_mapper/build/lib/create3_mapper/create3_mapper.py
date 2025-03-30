#!/usr/bin/env python3

# patch NumPy for transforms3d compatibility (gets rid of deprecated function errors)
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
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Point
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Time
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from rclpy.qos import QoSProfile, ReliabilityPolicy
import tf_transformations

class Create3Mapper(Node):
    def __init__(self):
        super().__init__('create3_mapper')

        # Listen for map->odom from SLAM, so we can compose transforms.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribe to odometry with a best-effort QoS.
        odom_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            odom_qos
        )

        # Publisher for a Marker that draws the robot's path as a line strip.
        self.marker_pub = self.create_publisher(Marker, 'robot_path_marker', 10)

        # Keep track of path points (x,y).
        self.path_points = []

        # Create a TransformBroadcaster for dynamic transforms.
        self.br = TransformBroadcaster(self)

    def odom_callback(self, msg: Odometry):
        """
        Handle incoming odometry:
          - Append (x,y) to self.path_points.
          - Clear path if there's a large jump.
          - Publish the path as a line strip Marker (in the map frame).
          - Compute and broadcast transforms map->base_footprint and map->laser_frame.
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # If there's a big jump, reset the path to avoid weird connecting lines.
        if self.path_points:
            last_x, last_y = self.path_points[-1]
            dist = math.hypot(x - last_x, y - last_y)
            if dist > 1.0:
                self.get_logger().info("Large jump detected (%.2f m); resetting path." % dist)
                self.path_points = []
        self.path_points.append((x, y))

        # Limit path length if desired
        if len(self.path_points) > 2000:
            self.path_points.pop(0)

        # Publish a line strip marker in the map frame
        self.publish_path_marker(msg.header.stamp)

        # Compute and broadcast transforms from map->base_footprint and map->laser_frame
        self.publish_dynamic_transforms(msg)

    def publish_path_marker(self, stamp: Time):
        if not self.path_points:
            return

        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = 'map'  # we want the path in the map frame
        marker.ns = 'robot_path'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.03  # line width
        # Color the line green; set alpha=1 for fully opaque
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Convert stored points to geometry_msgs/Point
        for (px, py) in self.path_points:
            p = Point()
            p.x = px
            p.y = py
            p.z = 0.0
            marker.points.append(p)

        # Publish the marker
        self.marker_pub.publish(marker)

    def publish_dynamic_transforms(self, odom_msg: Odometry):
        """
        Look up map->odom, combine with odom->base_footprint from odom_msg to get map->base_footprint,
        then compute map->laser_frame.
        """
        try:
            now = rclpy.time.Time()
            t_map_odom = self.tf_buffer.lookup_transform(
                'map', 'odom', now,
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except Exception as e:
            self.get_logger().warn('Could not lookup transform map->odom: %s' % str(e))
            return

        # Build T_map_odom
        map_to_odom_translation = [
            t_map_odom.transform.translation.x,
            t_map_odom.transform.translation.y,
            t_map_odom.transform.translation.z
        ]
        map_to_odom_quat = [
            t_map_odom.transform.rotation.x,
            t_map_odom.transform.rotation.y,
            t_map_odom.transform.rotation.z,
            t_map_odom.transform.rotation.w
        ]
        T_map_odom = tf_transformations.quaternion_matrix(map_to_odom_quat)
        T_map_odom[0:3, 3] = map_to_odom_translation

        # Extract odom->base_footprint from the odometry message
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

        # Compose: T_map_base = T_map_odom * T_odom_base
        T_map_base = np.dot(T_map_odom, T_odom_base)
        map_base_translation = T_map_base[0:3, 3]
        map_base_quat = tf_transformations.quaternion_from_matrix(T_map_base)

        # Broadcast map->base_footprint
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

        # Compute map->laser_frame (assuming a static offset from base_footprint)
        T_base_laser = np.eye(4)
        # Example: if laser is 0.1 m forward:
        # T_base_laser[0, 3] = 0.1
        T_map_laser = np.dot(T_map_base, T_base_laser)
        map_laser_translation = T_map_laser[0:3, 3]
        map_laser_quat = tf_transformations.quaternion_from_matrix(T_map_laser)

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

def main(args=None):
    rclpy.init(args=args)
    node = Create3Mapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
