# README

### Dependencies for the package :

All dependencies are encouraged to be installed by source.
1. Moveit2: [instructions](https://moveit.ai/install-moveit2/source/)
2. Universal_Robots_ROS2_Driver : [instructions](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble)
3. Realsense Driver for ROS2 : [instructions](https://github.com/realsenseai/realsense-ros)
4. Moveit Calibration : [instructions](https://github.com/AndrejOrsula/moveit2_calibration)
---
### Usage of the package : 

Bringup command for the system `ros2 launch dual_arm_workcell_bringup dual_arm_workcell_bringup.launch.py`

To simulate the system with fake hardware, use the `use_fake_hardware` flag. This will not launch any depth camera drivers.

Configure the ip and camera s.no parameters in the launch file when the system is changed.

The calibration of the system is done using the moveit_calibration package. The the relative orientation of the robots are 0.
