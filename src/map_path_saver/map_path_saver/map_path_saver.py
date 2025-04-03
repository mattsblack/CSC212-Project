import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import yaml
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray

class MapPathSaver(Node):
    def __init__(self):
        super().__init__('map_path_saver')
        
        # Subscribers
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.path_sub = self.create_subscription(MarkerArray, '/slam_toolbox/graph_visualization', self.path_callback, 10)
        
        # Storage
        self.map_data = None
        self.map_info = None
        self.path_points = []
        self.lidar_points = []
        
        self.get_logger().info("MapPathSaver node started. Waiting for map, lidar, and path data...")
    
    def map_callback(self, msg):
        self.map_info = msg.info
        width, height = msg.info.width, msg.info.height
        
        # Convert occupancy grid to numpy array
        map_array = np.array(msg.data, dtype=np.int8).reshape((height, width))
        
        # Normalize to grayscale image with RViz-like colors
        map_image = np.zeros((height, width, 3), dtype=np.uint8)
        map_image[map_array == 0] = [255, 255, 255]  # Free space (white)
        map_image[map_array == 100] = [0, 0, 0]      # Occupied (black)
        map_image[map_array == -1] = [127, 127, 127] # Unknown (gray)
        
        self.map_data = map_image
        self.get_logger().info("Map data received.")
    
    def lidar_callback(self, msg):
        self.lidar_points = []
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                self.lidar_points.append((x, y))
            angle += msg.angle_increment
        
        self.get_logger().info(f"Received {len(self.lidar_points)} Lidar points.")
    
    def path_callback(self, msg):
        self.path_points = []
        for marker in msg.markers:
            for point in marker.points:
                self.path_points.append((point.x, point.y))
        
        self.get_logger().info(f"Received {len(self.path_points)} path points.")
    
    def save_map(self, filename='saved_map'):
        if self.map_data is None or self.map_info is None:
            self.get_logger().warn("No map data received yet.")
            return
        
        # Convert path coordinates to pixel coordinates
        map_image = self.map_data.copy()
        origin_x, origin_y = self.map_info.origin.position.x, self.map_info.origin.position.y
        resolution = self.map_info.resolution
        
        for x, y in self.path_points:
            px = int((x - origin_x) / resolution)
            py = int((y - origin_y) / resolution)
            if 0 <= px < map_image.shape[1] and 0 <= py < map_image.shape[0]:
                cv2.circle(map_image, (px, map_image.shape[0] - py), 3, (0, 0, 255), -1)  # Red path points
        
        for x, y in self.lidar_points:
            px = int((x - origin_x) / resolution)
            py = int((y - origin_y) / resolution)
            if 0 <= px < map_image.shape[1] and 0 <= py < map_image.shape[0]:
                cv2.circle(map_image, (px, map_image.shape[0] - py), 1, (255, 0, 0), -1)  # Blue Lidar points
        
        # Save map as PNG
        png_filename = filename + '.png'
        cv2.imwrite(png_filename, map_image)
        
        # Save metadata as YAML
        yaml_filename = filename + '.yaml'
        map_metadata = {
            'image': png_filename,
            'resolution': self.map_info.resolution,
            'origin': [self.map_info.origin.position.x, self.map_info.origin.position.y, 0.0],
            'occupied_thresh': 0.65,
            'free_thresh': 0.196,
            'negate': 0
        }
        with open(yaml_filename, 'w') as yaml_file:
            yaml.dump(map_metadata, yaml_file, default_flow_style=False)
        
        self.get_logger().info(f"Map, path, and Lidar data saved as {png_filename} and {yaml_filename}.")


def main(args=None):
    rclpy.init(args=args)
    node = MapPathSaver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down MapPathSaver.")
    finally:
        node.save_map()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
