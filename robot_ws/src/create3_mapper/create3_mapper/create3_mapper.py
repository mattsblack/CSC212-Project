#!/usr/bin/env python3
# Monkey-patch numpy to restore np.float if missing, before any other imports.
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
from geometry_msgs.msg import PolygonStamped, Point32, TransformStamped
from builtin_interfaces.msg import Time
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from rclpy.qos import QoSProfile, ReliabilityPolicy
import tf_transformations
import numpy as np  # re-import if needed

class Create3Mapper(Node):
    def __init__(self):
        super().__init__('create3_mapper')

        # Create a tf buffer and listener to get transforms (e.g., map->odom) from SLAM Toolbox.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Use a best-effort QoS for the /odom subscription.
        odom_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            odom_qos
        )

        # Publisher for a PolygonStamped that traces the robot’s path.
        self.path_pub = self.create_publisher(PolygonStamped, 'robot_path_polygon', 10)

        # Storage for path points.
        self.path_points = []

        # Create a TransformBroadcaster for publishing our computed transforms.
        self.br = TransformBroadcaster(self)

    def odom_callback(self, msg: Odometry):
        """
        Process incoming odometry:
          - Append the (x,y) from odom->base_footprint to the path.
          - Optionally clear the path if a large jump is detected.
          - Publish the path as a PolygonStamped.
          - Compute and broadcast transforms from map->base_footprint and map->laser_frame.
        """
        # Extract the odom->base_footprint pose from odometry.
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Detect large jumps to avoid unwanted connecting lines.
        if self.path_points:
            last_x, last_y = self.path_points[-1]
            dist = math.hypot(x - last_x, y - last_y)
            if dist > 1.0:
                self.get_logger().info("Large jump detected (%.2f m); resetting path." % dist)
                self.path_points = []
        self.path_points.append((x, y))
        if len(self.path_points) > 2000:
            self.path_points.pop(0)

        # Publish the path as a PolygonStamped in the 'map' frame.
        self.publish_path_polygon(msg.header.stamp)

        # Compute and broadcast transforms from map->base_footprint and map->laser_frame.
        self.publish_dynamic_transforms(msg)

    def publish_path_polygon(self, stamp: Time):
        if not self.path_points:
            return

        poly = PolygonStamped()
        poly.header.stamp = stamp
        poly.header.frame_id = 'map'  # Publish the polygon in the map frame.

        for (px, py) in self.path_points:
            pt = Point32()
            pt.x = float(px)
            pt.y = float(py)
            pt.z = 0.0
            poly.polygon.points.append(pt)

        self.path_pub.publish(poly)

    def publish_dynamic_transforms(self, odom_msg: Odometry):
        """
        Look up the transform from map->odom (from SLAM Toolbox) and combine it with odom->base_footprint
        from the odometry message to compute map->base_footprint. Then, publish that transform and compute map->laser_frame.
        """
        try:
            now = rclpy.time.Time()
            t_map_odom = self.tf_buffer.lookup_transform('map', 'odom', now, timeout=rclpy.duration.Duration(seconds=0.1))
        except Exception as e:
            self.get_logger().warn('Could not lookup transform from map to odom: %s' % str(e))
            return

        # Build T_map_odom from the lookup.
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

        # Get the odom->base_footprint transform from the odometry message.
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
        T_map_base = np.dot(T_map_odom, T_odom_base)

        # Extract translation and rotation.
        map_base_translation = T_map_base[0:3, 3]
        map_base_quat = tf_transformations.quaternion_from_matrix(T_map_base)

        # Broadcast transform from map to base_footprint.
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

        # Compute map->laser_frame using a static transform from base_footprint to laser_frame.
        # Adjust T_base_laser if the laser is offset from base_footprint.
        T_base_laser = np.eye(4)
        # Example: if the laser is 0.1 m forward relative to base_footprint, uncomment the next line:
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

