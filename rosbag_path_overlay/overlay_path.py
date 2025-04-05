import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from PIL import Image
from pathlib import Path
from rosbags.highlevel import AnyReader

"""
@file overlay_path.py
@brief Overlay robot path data on an occupancy grid map

This script combines occupancy grid map data (as a PGM image) with robot 
odometry data (from a CSV file) to create a visualization of the robot's
path overlaid on the map. The script can either use hardcoded map metadata
or automatically extract it from a ROS2 bag file.
"""

# ----- CONFIGURATION -----
# Set this flag to True to automatically extract metadata from rosbag.
use_auto_metadata = True

# Default parameters (used if auto extraction fails or is disabled)
default_map_resolution = 0.05   # meters per pixel
default_origin_x = -8.0         # world x coordinate corresponding to the left side of the image
default_origin_y = -5.0         # world y coordinate corresponding to the bottom of the image

# Path to the map image (exported as .pgm)
map_filename = 'map_data.pgm'
# Path to the odometry CSV file
csv_filename = 'odometry_data.csv'

# If using automatic metadata extraction, set the bag path here.
# (Ensure that the bag contains a map message on the '/map' topic.)
bag_path = Path('path/to/bag')  # adjust as needed

# ----- AUTOMATIC EXTRACTION OF MAP METADATA -----
"""
@brief Extract map metadata from ROS2 bag file
@details Attempts to read the first map message from the bag to get map resolution and origin

This section extracts the map resolution and origin coordinates from the first map message
found in the ROS2 bag file. If extraction fails or is disabled, it falls back to default values.

Parameters extracted:
- map_resolution: meters per pixel
- origin_x, origin_y: Map origin in world coordinates
"""
if use_auto_metadata:
    try:
        with AnyReader([bag_path]) as reader:
            # Find connections that publish to the '/map' topic
            connections = [conn for conn in reader.connections if conn.topic == '/map']
            if not connections:
                raise RuntimeError("No '/map' topic found in the rosbag. Using default parameters.")
            
            # Process the first map message to extract metadata
            for connection, timestamp, rawdata in reader.messages(connections):
                msg = reader.deserialize(rawdata, connection.msgtype)
                # Extract map metadata from msg.info (OccupancyGrid message)
                map_resolution = msg.info.resolution
                width = msg.info.width
                height = msg.info.height
                origin = msg.info.origin  # geometry_msgs/Pose
                origin_x = origin.position.x # add offset if needed
                origin_y = origin.position.y # add offset if needed
                print("Automatically extracted map metadata:")
                print("  Resolution (m/pixel):", map_resolution)
                print("  Dimensions (pixels):", width, "x", height)
                print("  Origin (x, y):", origin_x, origin_y)
                break  # Use the first map message found
    except Exception as e:
        print("Automatic extraction of map metadata failed:", e)
        print("Falling back to default parameters.")
        map_resolution = default_map_resolution
        origin_x = default_origin_x
        origin_y = default_origin_y
else:
    # Use the default parameters if automatic extraction is disabled
    map_resolution = default_map_resolution
    origin_x = default_origin_x
    origin_y = default_origin_y

# ----- LOAD THE MAP IMAGE -----
"""
@brief Load the occupancy grid map from PGM file
@details Loads the map image and converts it to a numpy array for processing
"""
try:
    map_img = Image.open(map_filename)
    map_data = np.array(map_img)
except Exception as e:
    raise RuntimeError(f"Error loading map image '{map_filename}': {e}")

# Get map dimensions in pixels from the image
map_height, map_width = map_data.shape

# Calculate the extent of the map in world coordinates.
# extent = [left, right, bottom, top]
# This is used by matplotlib to properly scale and position the image
extent = [
    origin_x,
    origin_x + map_width * map_resolution,
    origin_y,
    origin_y + map_height * map_resolution
]

# ----- LOAD THE ODOMETRY DATA -----
"""
@brief Load robot odometry data from CSV file
@details Reads the position data from a CSV file exported by extract_map.py
"""
try:
    df = pd.read_csv(csv_filename)
except Exception as e:
    raise RuntimeError(f"Error loading odometry CSV '{csv_filename}': {e}")

# Validate that the CSV contains the required columns
if 'pos_x' not in df.columns or 'pos_y' not in df.columns:
    raise ValueError("CSV file must contain 'pos_x' and 'pos_y' columns.")

# Extract the coordinates from the CSV.
x_coords = df['pos_x'].values
y_coords = df['pos_y'].values

# ----- DEBUG: PRINT RANGES -----
# These values help to verify that the coordinate systems align properly
print("Odometry X range:", x_coords.min(), x_coords.max())
print("Odometry Y range:", y_coords.min(), y_coords.max())
print("Map extent:", extent)

# ----- PLOTTING: OVERLAY PATH ON MAP -----
"""
@brief Create and display the visualization
@details Creates a matplotlib figure with the map image and robot path overlay
"""
# Create a figure with appropriate size
plt.figure(figsize=(10, 8))

# Display the map image.
# The 'extent' parameter ensures the image is properly scaled to world coordinates
# 'origin=lower' ensures the origin is at the bottom left, as ROS uses a right-handed coordinate system
plt.imshow(map_data, cmap='gray', extent=extent, origin='lower')

# Overlay the odometry path as a red line with markers
# This visualizes the robot's trajectory through the environment
plt.plot(x_coords, y_coords, marker='o', linestyle='-', color='red',
         linewidth=2, markersize=3, alpha=0.5, label='Odometry Path')

# Add labels and formatting
plt.xlabel("X (meters)")
plt.ylabel("Y (meters)")
plt.title("Robot Path Overlaid on Map")
plt.legend()
plt.grid(True)

# Set the plot limits to match the map extent
plt.xlim(extent[0], extent[1])
plt.ylim(extent[2], extent[3])

# Display the visualization
plt.show()

# Optionally, save the overlay to a file.
# plt.savefig('automatic_overlay_path_on_map.png', dpi=300)
