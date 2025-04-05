# Rosbag Path Overlay for iRobot Create3

Rosbag Path Overlay for iRobot Create3 plots the path of a robot over a map image using data extracted from a ROS2 rosbag.
This approach can be used to compare the coverage of pathfinding algorithms. As an example, two different algorithms are provided in this repository.

## Features

- Extracts data from `/map`, `/odom` topics.
- Overlays the robot's path on a map image.
- Allows (and often requires) manual adjustment of the origin coordinates for a precise overlay.

## Usage

1. [**Record a rosbag**](#record-a-rosbag) with topics: `/map`, `/odom`
2. [**Install dependencies**](#install-dependencies)
3. [**Extract data**](#extract-data)
4. [**Generate the overlay**](#generate-the-overlay)

## Record a rosbag

To record a rosbag with all the topics we need, we have to first launch the necessary sensors.

Source the setup shell script:

```bash
source ~/create3_examples_ws/install/setup.bash
```

Run the sensors launch script:

```bash
ros2 launch create3_lidar_slam sensors_launch.py
```

Right before launching your algorithm, start recording the bag:

```bash
ros2 bag record /map /odom
```

More on rosbag2 here: [rosbag2](https://github.com/ros2/rosbag2)

## Install dependencies

Make sure you have the following Python packages installed:

```bash
numpy
pandas
matplotlib
Pillow
rosbags
```

You can install them via pip:

```bash
pip install numpy pandas matplotlib Pillow rosbags
```

## Extract data

After changing the path to your rosbag folder in `extract_map.py`, run the script. This will create two files: `odometry_data.csv`, `map_data.pgm`.

## Generate the overlay

If needed, change the paths to your map pgm odometry csv files and your rosbag folder in `overlay_path.py`. Run the script to see the path overlayed over the map image.

NOTE: The path is most likely going to be off its actual origin. Unfortunately this requires your manual adjustment in the script.
