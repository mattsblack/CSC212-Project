#!/usr/bin/env python3
import csv
from pathlib import Path
import pandas as pd
import numpy as np
from PIL import Image
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

# Global bag path (adjust if needed)
bag_path = Path('/home/qcc/bag_files/bag_test')

def export_battery_data():
    """Extract battery state data and save to CSV."""
    output_file = 'battery_data.csv'
    with AnyReader([bag_path]) as reader:
        connections = [conn for conn in reader.connections if conn.topic == '/battery_state']
        data = []
        for connection, timestamp, rawdata in reader.messages(connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            data.append([timestamp, msg.voltage, msg.percentage])
    with open(output_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp", "voltage", "percentage"])
        writer.writerows(data)
    print(f"Exported battery data to {output_file}")

def setup_custom_store():
    """
    Set up and return a custom type store for iRobot messages.
    It parses the .msg files from the iRobot Create messages directory.
    """
    msg_folder = Path("/opt/ros/humble/share/irobot_create_msgs/msg")
    if not msg_folder.exists():
        raise FileNotFoundError(f"Message folder not found: {msg_folder}")
    msg_files = list(msg_folder.glob("*.msg"))
    if not msg_files:
        raise FileNotFoundError(f"No MSG files found in {msg_folder}")
    
    add_types = {}
    for msg_file in msg_files:
        msg_text = msg_file.read_text()
        msg_path = f"irobot_create_msgs/msg/{msg_file.stem}"
        add_types.update(get_types_from_msg(msg_text, msg_path))
    
    store = get_typestore(Stores.ROS2_HUMBLE)
    store.register(add_types)
    return store

def extract_data(topic, output_file, columns, process_msg, store=None):
    """
    Generic extractor for a given topic.
    If a 'store' is provided, it uses store.deserialize_cdr; otherwise it uses the default reader.deserialize.
    """
    data = []
    with AnyReader([bag_path]) as reader:
        connections = [conn for conn in reader.connections if conn.topic == topic]
        for connection, timestamp, rawdata in reader.messages(connections):
            if store:
                msg = store.deserialize_cdr(rawdata, connection.msgtype)
            else:
                msg = reader.deserialize(rawdata, connection.msgtype)
            processed_data = process_msg(timestamp, msg)
            # Ensure that if the processed data is a list of lists, we flatten it correctly
            if isinstance(processed_data, list) and processed_data and isinstance(processed_data[0], list):
                data.extend(processed_data)
            else:
                data.append(processed_data)
    if not data:
        print(f"No data found for topic: {topic}")
        return
    df = pd.DataFrame(data, columns=columns)
    df.to_csv(output_file, index=False)
    print(f"Exported {topic} data to {output_file}")

def export_custom_data(store):
    """Extract custom topics using the iRobot message definitions."""
    # Extract detailed wheel status
    extract_data(
        "/wheel_status",
        "wheel_status_custom.csv",
        ["timestamp", "current_ma_left", "current_ma_right", "pwm_left", "pwm_right", "wheels_enabled"],
        lambda timestamp, msg: [
            timestamp,
            getattr(msg, "current_ma_left", None),
            getattr(msg, "current_ma_right", None),
            getattr(msg, "pwm_left", None),
            getattr(msg, "pwm_right", None),
            getattr(msg, "wheels_enabled", None),
        ],
        store=store,
    )
    # Extract interface buttons status
    extract_data(
        "/interface_buttons",
        "interface_buttons.csv",
        ["timestamp", "button_1_pressed", "button_power_pressed", "button_2_pressed"],
        lambda timestamp, msg: [
            timestamp,
            getattr(msg.button_1, "is_pressed", None),
            getattr(msg.button_power, "is_pressed", None),
            getattr(msg.button_2, "is_pressed", None),
        ],
        store=store,
    )
    # Extract hazard detections (each detection becomes a row)
    extract_data(
        "/hazard_detection",
        "hazard_detection.csv",
        ["timestamp", "hazard_type", "frame_id", "detection_time_sec", "detection_time_nsec"],
        lambda timestamp, msg: (
            [
                [
                    timestamp,
                    hazard.type,
                    hazard.header.frame_id,
                    hazard.header.stamp.sec,
                    hazard.header.stamp.nanosec,
                ]
                for hazard in getattr(msg, "detections", [])
            ]
            if hasattr(msg, "detections") else []
        ),
        store=store,
    )

def export_lidar_scan():
    """Extract LIDAR scan data and save to CSV."""
    output_file = 'lidar_scan.csv'
    with AnyReader([bag_path]) as reader:
        connections = [conn for conn in reader.connections if conn.topic == '/scan']
        data = []
        for connection, timestamp, rawdata in reader.messages(connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            data.append([timestamp, msg.angle_min, msg.angle_max, msg.range_min, msg.range_max, msg.ranges])
    df = pd.DataFrame(data, columns=["timestamp", "angle_min", "angle_max", "range_min", "range_max", "ranges"])
    df.to_csv(output_file, index=False)
    print(f"Exported LIDAR scan data to {output_file}")

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

def export_wheel_status_simple(store):
    """
    Extract wheel status with simple fields (velocity and torque) and save to CSV.
    Instead of passing 'store' to AnyReader, we open the bag normally and then use the store to deserialize.
    """
    output_file = 'wheel_status_simple.csv'
    data = []
    with AnyReader([bag_path]) as reader:
        connections = [conn for conn in reader.connections if conn.topic == '/wheel_status']
        for connection, timestamp, rawdata in reader.messages(connections):
            # Use the provided store to deserialize
            msg = store.deserialize_cdr(rawdata, connection.msgtype)
            data.append([timestamp, getattr(msg, "velocity", None), getattr(msg, "torque", None)])
    df = pd.DataFrame(data, columns=["timestamp", "velocity", "torque"])
    df.to_csv(output_file, index=False)
    print(f"Exported wheel status (simple) data to {output_file}")

def main():
    print("Starting data export from ROS2 bag...")
    
    try:
        export_battery_data()
    except Exception as e:
        print("Error exporting battery data:", e)
    
    try:
        custom_store = setup_custom_store()
    except Exception as e:
        print("Error setting up custom store:", e)
        custom_store = None
    
    try:
        if custom_store is not None:
            export_custom_data(custom_store)
        else:
            print("Skipping custom data export due to store setup error.")
    except Exception as e:
        print("Error exporting custom data:", e)
    
    try:
        export_lidar_scan()
    except Exception as e:
        print("Error exporting LIDAR scan data:", e)
    
    try:
        export_map_data()
    except Exception as e:
        print("Error exporting map data:", e)
    
    try:
        export_odometry_data()
    except Exception as e:
        print("Error exporting odometry data:", e)
    
    try:
        if custom_store is not None:
            export_wheel_status_simple(custom_store)
        else:
            print("Skipping simple wheel status export due to store setup error.")
    except Exception as e:
        print("Error exporting wheel status simple data:", e)
    
    print("All data exported successfully!")

if __name__ == "__main__":
    main()
