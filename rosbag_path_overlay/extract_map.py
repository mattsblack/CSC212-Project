from pathlib import Path
import pandas as pd
import numpy as np
from PIL import Image
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg
import matplotlib.pyplot as plt

bag_path = Path('path/to/bag')  # adjust as needed

def export_odometry_data():
    """Extract odometry data and save to CSV."""
    output_file = 'odometry_data.csv'
    with AnyReader([bag_path]) as reader:
        connections = [conn for conn in reader.connections if conn.topic == '/odom']
        data = []
        for connection, timestamp, rawdata in reader.messages(connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            position = msg.pose.pose.position
            orientation = msg.pose.pose.orientation
            velocity = msg.twist.twist.linear
            data.append([timestamp, position.x, position.y, position.z,
                            orientation.x, orientation.y, orientation.z,
                            velocity.x, velocity.y, velocity.z])
    df = pd.DataFrame(data, columns=["timestamp", "pos_x", "pos_y", "pos_z",
                                        "orient_x", "orient_y", "orient_z",
                                        "vel_x", "vel_y", "vel_z"])
    df.to_csv(output_file, index=False)
    print(f"Exported odometry data to {output_file}")

def export_map_data():
    """Extract map data and export as a PGM image."""
    output_file = "map_data.pgm"
    with AnyReader([bag_path]) as reader:
        connections = [conn for conn in reader.connections if conn.topic == '/map']
        for connection, timestamp, rawdata in reader.messages(connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            width = msg.info.width
            height = msg.info.height
            data = np.array(msg.data, dtype=np.int8).reshape((height, width))
            # Convert -1 (unknown) to 205 and cast to uint8
            data = data.astype(np.uint8)
            data[data == 255] = 205
            img = Image.fromarray(data, mode='L')
            img.save(output_file)
    print(f"Exported map data to {output_file}")

def main():
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
