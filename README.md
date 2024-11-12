# EOHJackalBoundry: Jackal Robot Boundary Control

This repository contains a Python script for controlling the Jackal robot's movement within a defined boundary using ROS (Robot Operating System). The script subscribes to the robot's odometry and joystick messages, performs coordinate transformations, and publishes velocity commands to ensure the robot stays within the specified area.

## Installation

To set up and run this project, follow these steps:

1. Clone the repository:

```bash
git clone https://github.com/AUVSL/EOHJackalBoundry.git
cd EOHJackalBoundry
```

2. Install the required dependencies:

```bash
pip install -r requirements.txt
```

3. Build and source the ROS workspace:

```bash
cd catkin_ws
catkin_make
source devel/setup.bash
```

**Note:** Make sure you have ROS Melodic installed on your system. You can find installation instructions on the [ROS Kinetic website](http://wiki.ros.org/melodic/Installation).

5. Launch the Jackal simulation with the required nodes:

```bash
roslaunch jackal_gazebo jackal_world.launch
roslaunch jackal_control jackal_base.launch
roslaunch jackal_viz view_robot.launch
```

## Usage

1. Run the launch file:

```bash
roslaunch EOHJackalBoundry boundry.launch
```

2. Use the joystick to control the robot. Pressing all four buttons (indices 4, 5, 9, and 10) will reset the boundary transformation.

3. The script will continuously read the robot's odometry and joystick input, perform coordinate transformations, and publish velocity commands to keep the robot within the defined boundary.

## Parameters

- `width`: The width of the allowed area in meters (default: 10).
- `height`: The height of the allowed area in meters (default: 10).

These parameters can be set in the launch file or using `rosparam set` command.

## Contributing

Contributions are welcome! If you find any issues or have suggestions for improvements, please open an issue or submit a pull request.

This project is licensed under the [MIT License](LICENSE).
