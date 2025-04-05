# create3_mapper

ROS2 package for mapping capabilities with the iRobot Create 3 robot. This package publishes the robot's path using the `Marker` message type. This code is loosely based on the [ros2_explorer](https://github.com/DaniGarciaLopez/ros2_explorer/tree/main) package.

## Building the Package

1. Create a workspace (if you don't already have one):
    ```bash
    mkdir -p ~/ros2_ws/src
    ```

2. Clone this repository into your workspace's src folder:
    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/mattsblack/CSC212-Project/
    ```

3. Install dependencies:
    ```bash
    cd ~/ros2_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```

4. Build the package:
    ```bash
    colcon build --packages-select create3_mapper
    ```

## Sourcing the Setup File

After building the package, you need to source the setup file to use it:

```bash
source ~/ros2_ws/install/setup.bash
```

To automatically source this file every time you open a new terminal, add this line to your `~/.bashrc` file:

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

## Usage
1. Launch the `create3_mapper` node:
    ```bash
    ros2 run create3_mapper create3_mapper.py
    ```
2. Launch RViz to visualize the robot's path:
    ```bash
    ros2 launch create3_mapper rviz_launch.py
    ```