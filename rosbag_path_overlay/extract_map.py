from pathlib import Path
import pandas as pd
import numpy as np
from PIL import Image
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg
import matplotlib.pyplot as plt

# Path to the ROS bag file (adjust as needed for your setup)
bag_path = Path('path/to/bag')  # adjust as needed

"""
@file extract_map.py
@brief Extract map and odometry data from ROS2 bag files

This script extracts occupancy grid map data and robot odometry data from ROS2 
bag files and saves them as PGM image and CSV files respectively. The extracted
data can be used for analyzing robot trajectories and visualizing paths overlaid
on maps.
"""

def export_odometry_data():
    """
    @brief Extract odometry data from a ROS2 bag and save to CSV
    
    Reads messages from the '/odom' topic in the bag file, extracting position,
    orientation, and velocity information. The data is saved to a CSV file named
    'odometry_data.csv' with appropriate column headers.
    
    @return None
    """
    output_file = 'odometry_data.csv'
    with AnyReader([bag_path]) as reader:
        # Filter connections to only include odometry messages
        connections = [conn for conn in reader.connections if conn.topic == '/odom']
        data = []
        
        # Iterate through all odometry messages in the bag
        for connection, timestamp, rawdata in reader.messages(connections):
            # Deserialize the raw message data into a nav_msgs/Odometry object
            msg = reader.deserialize(rawdata, connection.msgtype)
            
            # Extract position, orientation, and velocity components
            position = msg.pose.pose.position
            orientation = msg.pose.pose.orientation
            velocity = msg.twist.twist.linear
            
            # Append extracted data to our collection
            data.append([timestamp, position.x, position.y, position.z,
                         orientation.x, orientation.y, orientation.z,
                         velocity.x, velocity.y, velocity.z])
    
    # Create DataFrame with appropriate column labels
    df = pd.DataFrame(data, columns=["timestamp", "pos_x", "pos_y", "pos_z",
                                     "orient_x", "orient_y", "orient_z",
                                     "vel_x", "vel_y", "vel_z"])
    
    # Save to CSV file
    df.to_csv(output_file, index=False)
    print(f"Exported odometry data to {output_file}")

def export_map_data():
    """
    @brief Extract map data from a ROS2 bag and export as PGM image
    
    Reads messages from the '/map' topic in the bag file, extracting the occupancy
    grid data. The occupancy grid is converted to a grayscale image and saved as
    a PGM file named 'map_data.pgm'.
    
    The function processes the first map message found in the bag file, converting
    the occupancy grid values to appropriate grayscale values for visualization:
    - Free space (0): Black
    - Unknown (-1): Gray (205)
    - Occupied (100): White
    
    @return None
    """
    output_file = "map_data.pgm"
    with AnyReader([bag_path]) as reader:
        # Filter connections to only include map messages
        connections = [conn for conn in reader.connections if conn.topic == '/map']
        
        # Process the first map message found
        for connection, timestamp, rawdata in reader.messages(connections):
            # Deserialize the raw message data into a nav_msgs/OccupancyGrid object
            msg = reader.deserialize(rawdata, connection.msgtype)
            
            # Extract map dimensions
            width = msg.info.width
            height = msg.info.height
            
            # Reshape the flat array into a 2D grid
            data = np.array(msg.data, dtype=np.int8).reshape((height, width))
            
            # Convert to uint8 format for image creation
            # In ROS, occupancy grid values are:
            # - 0: Free space
            # - 100: Occupied
            # - -1: Unknown
            # For visualization, we convert -1 (unknown) to gray (205)
            data = data.astype(np.uint8)
            data[data == 255] = 205  # Convert ROS unknown cells (-1→255) to gray (205)
            
            # Create and save the image
            img = Image.fromarray(data, mode='L')  # 'L' mode for grayscale
            img.save(output_file)
            
            # Only process the first map message
            break
    
    print(f"Exported map data to {output_file}")

def main():
    """
    @brief Main entry point for the script
    
    Calls the export functions in sequence, handling exceptions for each
    operation separately to ensure maximum data extraction even if one
    part fails.
    
    @return None
    """
    print("Starting data export from ROS2 bag...")

    try:
        export_map_data()
    except Exception as e:
        print("Error exporting map data:", e)

    try:
        export_odometry_data()
    except Exception as e:
        print("Error exporting odometry data:", e)

    print("All data exported successfully!")

if __name__ == "__main__":
    main()
