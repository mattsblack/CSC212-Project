# create_3_spiral_coverage

The two algorithms are simply two split behaviours of the non-systematic coverage algorithm in [create3_examples](https://github.com/iRobotEducation/create3_examples/tree/humble).

## Build the package

After you build `create3_examples` and `create3_examples_msgs`, add `create3_spiral_coverage` and `create3_straight_coverage` to your `create3_examples_ws`.

In your `create3_spiral_coverage` workspace folder, run:

```bash
colcon build
```

Source the workspace:

```bash
source install/setup.bash
```

In your `src` folder create the ROS2 package:

```bash
ros2 pkg create --build-type ament_cmake create3_straight_coverage
```

Back in the root directory of your `create3_straight_coverage` workspace run:

```bash
colcon build --packages-select create3_straight_coverage
```

Source the workspace before you run your algorithm:

```bash
source install/setup.bash
```

## Launch

Start the coverage action server:

```bash
ros2 run create3_straight_coverage create3_straight_coverage
```

Then launch the algorithm in a separate terminal (after you start recording the bag):

```bash
ros2 action send_goal /coverage create3_examples_msgs/action/Coverage "{explore_duration:{sec: 500, nanosec: 0}, max_runtime:{sec: 1000,nanosec: 0}}"
```
