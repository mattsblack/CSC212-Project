# Path Publisher

**NOTE:** This is an early version of the create3_mapper package and doesn't need to be run.

## Overview

Path Publisher is a ROS2 package designed to publish predefined paths for robot navigation. It provides a simple interface to generate and publish path messages that can be used by navigation or visualization tools.

## Prerequisites

- ROS2 Humble
- Ubuntu 22.04
- Basic knowledge of ROS2 concepts

## Installation

1. Navigate to your ROS2 workspace source directory:

    ```bash
    cd ~/your_ros_workspace/src
    ```

2. Clone the repository:

    ```bash
    git clone https://github.com/mattsblack/CSC212-Project.git
    ```

3. Build the package:

    ```bash
    cd ..
    colcon build
    ```

4. Source the workspace:

    ```bash
    source install/setup.bash
    ```

## Setup

1. Configure path parameters in the config file (if applicable):

    ```bash
    nano path_publisher/config/paths.yaml
    ```

2. Adjust any parameters according to your specific requirements.

## Usage

To run the path publisher node:

```bash
ros2 run path_publisher path_publisher_node
```

Alternatively, you can use the provided launch file:

```bash
ros2 launch path_publisher path_publisher.launch.py
```

## Topics

The package publishes to the following topics:

- `/path` - The generated path (nav_msgs/Path)

## Parameters

- `update_rate`: Frequency at which to publish paths (default: 1.0 Hz)
- `frame_id`: Frame ID for the path messages (default: "map")

## License

This project is licensed under the MIT License - see the LICENSE file for details.
