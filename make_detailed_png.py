#!/usr/bin/env python3
"""
Generate a colored occupancy grid map image with sensor overlays.
Optionally, if --only-path is provided, only the robot's path is overlaid,
skipping the LIDAR and point cloud data.
"""

import argparse
import math
import numpy as np
from pathlib import Path
from PIL import Image
import matplotlib.pyplot as plt

from rosbags.highlevel import AnyReader
from rosbags.typesys import get_typestore, get_types_from_msg, Stores

# ---------------------------------------------------------------------
# Register custom message types from iRobot Create messages
# ---------------------------------------------------------------------
def setup_custom_types():
    msg_folder = Path("/opt/ros/humble/share/irobot_create_msgs/msg")
    if not msg_folder.exists():
        raise FileNotFoundError(f"Message folder not found: {msg_folder}")
    msg_files = list(msg_folder.glob("*.msg"))
    if not msg_files:
        raise FileNotFoundError(f"No .msg files found in {msg_folder}")
    add_types = {}
    for msg_file in msg_files:
        msg_text = msg_file.read_text()
        msg_path = f"irobot_create_msgs/msg/{msg_file.stem}"
        add_types.update(get_types_from_msg(msg_text, msg_path))
    store = get_typestore(Stores.ROS2_HUMBLE)
    store.register(add_types)
    return store

# ---------------------------------------------------------------------
# A simple 2D transform class for rotation+translation.
# ---------------------------------------------------------------------
class Transform2D:
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta

    def apply(self, points):
        """
        Apply a 2D rotation and translation to an Nx2 numpy array.
        """
        c = math.cos(self.theta)
        s = math.sin(self.theta)
        x_new = points[:, 0]*c - points[:, 1]*s + self.x
        y_new = points[:, 0]*s + points[:, 1]*c + self.y
        return np.column_stack((x_new, y_new))

def quaternion_to_yaw(q):
    """
    Convert a quaternion (x, y, z, w) to a yaw angle (2D rotation).
    """
    siny_cosp = 2.0 * (q.w*q.z + q.x*q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    return math.atan2(siny_cosp, cosy_cosp)

# ---------------------------------------------------------------------
# Main visualization function.
# ---------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="Generate map visualization with sensor overlays. "
                    "Use --only-path to only overlay the robot's path."
    )
    parser.add_argument(
        "--only-path",
        action="store_true",
        help="Only include the robot's path, skipping LIDAR and point cloud overlays."
    )
    args = parser.parse_args()

    custom_store = setup_custom_types()
    bag_path = Path('/home/qcc/bag_files/bag_test')
    output_file = 'map_overview.png'

    with AnyReader([bag_path]) as reader:
        # 1. Retrieve the Occupancy Grid from /map
        map_conns = [c for c in reader.connections if c.topic == '/map']
        map_msg = None
        for conn, t, rawdata in reader.messages(map_conns):
            map_msg = reader.deserialize(rawdata, conn.msgtype)
        if not map_msg:
            print("No /map messages found.")
            return

        width = map_msg.info.width
        height = map_msg.info.height
        resolution = map_msg.info.resolution
        origin_pose = map_msg.info.origin  # geometry_msgs/Pose

        # Reshape the occupancy data (flat int8 array)
        map_data = np.array(map_msg.data, dtype=np.int8).reshape(height, width)

        # Create a color image: white = free, black = occupied, gray = unknown.
        color_map = np.zeros((height, width, 3), dtype=np.uint8)
        for r in range(height):
            for c in range(width):
                val = map_data[r, c]
                if val < 0 or val == 255:
                    color_map[r, c] = (127, 127, 127)  # Unknown: gray
                elif val == 0:
                    color_map[r, c] = (255, 255, 255)  # Free: white
                else:
                    if val >= 50:
                        color_map[r, c] = (0, 0, 0)      # Occupied: black
                    else:
                        shade = 255 - int(val * 2.55)
                        color_map[r, c] = (shade, shade, shade)

        # 2. Retrieve the robot's path from /odom
        odom_conns = [c for c in reader.connections if c.topic == '/odom']
        odom_positions = []
        # For simplicity, assume the map-to-odom transform is identity.
        map_to_odom_tf = Transform2D(0.0, 0.0, 0.0)
        for conn, t, rawdata in reader.messages(odom_conns):
            odom_msg = reader.deserialize(rawdata, conn.msgtype)
            x = odom_msg.pose.pose.position.x
            y = odom_msg.pose.pose.position.y
            p = np.array([[x, y]])
            p_map = map_to_odom_tf.apply(p)
            odom_positions.append(p_map[0])
        odom_positions = np.array(odom_positions)

        # Initialize empty arrays for sensor overlays.
        scan_points_map = np.zeros((0, 2))
        pc_points_map = np.zeros((0, 2))

        if not args.only_path:
            # 3. Retrieve and overlay LIDAR scans from /scan
            scan_conns = [c for c in reader.connections if c.topic == '/scan']
            scan_list = []
            # Assume laser is at the origin of the odom frame.
            laser_to_odom_tf = Transform2D(0.0, 0.0, 0.0)
            for conn, t, rawdata in reader.messages(scan_conns):
                scan_msg = reader.deserialize(rawdata, conn.msgtype)
                angle = scan_msg.angle_min
                local_pts = []
                for r in scan_msg.ranges:
                    if r < scan_msg.range_min or r > scan_msg.range_max:
                        angle += scan_msg.angle_increment
                        continue
                    lx = r * math.cos(angle)
                    ly = r * math.sin(angle)
                    local_pts.append((lx, ly))
                    angle += scan_msg.angle_increment
                if local_pts:
                    local_pts = np.array(local_pts)
                    odom_pts = laser_to_odom_tf.apply(local_pts)
                    map_pts = map_to_odom_tf.apply(odom_pts)
                    scan_list.append(map_pts)
            if scan_list:
                scan_points_map = np.vstack(scan_list)

            # 4. Retrieve and overlay point cloud data from /pointcloud (if available)
            pc_conns = [c for c in reader.connections if c.topic == '/pointcloud']
            pc_list = []
            for conn, t, rawdata in reader.messages(pc_conns):
                try:
                    pc_msg = custom_store.deserialize_cdr(rawdata, conn.msgtype)
                except KeyError as e:
                    print("Skipping point cloud message of type", conn.msgtype, ":", e)
                    continue
                if hasattr(pc_msg, 'points'):
                    pts = []
                    for pt in pc_msg.points:
                        pts.append((pt.x, pt.y))
                    if pts:
                        pts = np.array(pts)
                        pts_map = map_to_odom_tf.apply(pts)
                        pc_list.append(pts_map)
            if pc_list:
                pc_points_map = np.vstack(pc_list)

    # ------------------------------------------------------------------
    # Helper: Convert world coordinates to image pixel coordinates.
    # ------------------------------------------------------------------
    map_origin_x = origin_pose.position.x
    map_origin_y = origin_pose.position.y
    q = origin_pose.orientation
    map_yaw = quaternion_to_yaw(q)

    def world_to_map_coords(wx, wy):
        # Translate relative to map origin.
        rel_x = wx - map_origin_x
        rel_y = wy - map_origin_y
        # Rotate by -map_yaw.
        c = math.cos(-map_yaw)
        s = math.sin(-map_yaw)
        rot_x = rel_x*c - rel_y*s
        rot_y = rel_x*s + rel_y*c
        px = int(rot_x / resolution)
        py = int(rot_y / resolution)
        # Convert to image row, col.
        row = height - py - 1
        col = px
        return row, col

    # ------------------------------------------------------------------
    # Plotting the occupancy map with overlays.
    # ------------------------------------------------------------------
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.imshow(np.flipud(color_map), origin='lower')

    # Always plot the robot path.
    if odom_positions.shape[0] > 1:
        path_pixels = []
        for (ox, oy) in odom_positions:
            row, col = world_to_map_coords(ox, oy)
            path_pixels.append((col, row))
        path_pixels = np.array(path_pixels)
        ax.plot(path_pixels[:, 0], path_pixels[:, 1], color='black', linewidth=1, label='Robot Path')

    if not args.only_path:
        # Plot LIDAR scan points.
        if scan_points_map.shape[0] > 0:
            scan_pixels = []
            for (sx, sy) in scan_points_map:
                row, col = world_to_map_coords(sx, sy)
                scan_pixels.append((col, row))
            scan_pixels = np.array(scan_pixels)
            ax.scatter(scan_pixels[:, 0], scan_pixels[:, 1], s=1, c='red', label='LIDAR Points', alpha=0.5)
        # Plot point cloud points.
        if pc_points_map.shape[0] > 0:
            pc_pixels = []
            for (px_val, py_val) in pc_points_map:
                row, col = world_to_map_coords(px_val, py_val)
                pc_pixels.append((col, row))
            pc_pixels = np.array(pc_pixels)
            ax.scatter(pc_pixels[:, 0], pc_pixels[:, 1], s=1, c='orange', label='Point Cloud', alpha=0.5)

    ax.set_title("Occupancy Map with Sensor Overlays")
    ax.set_xlabel("Map X (pixels)")
    ax.set_ylabel("Map Y (pixels)")
    ax.legend(loc='upper right')
    plt.savefig(output_file, bbox_inches='tight', dpi=150)
    print(f"Saved combined map visualization to {output_file}")

if __name__ == "__main__":
    main()
