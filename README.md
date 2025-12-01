# README

### Dependencies for the package :

All dependencies are encouraged to be installed by source.
1. Moveit2: [instructions](https://moveit.ai/install-moveit2/source/)
2. Universal_Robots_ROS2_Driver : [instructions](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble)
3. Realsense Driver for ROS2 : [instructions](https://github.com/realsenseai/realsense-ros)
4. Moveit Calibration : [instructions](https://github.com/AndrejOrsula/moveit2_calibration)
5. GroundingDino : [instructions](https://github.com/IDEA-Research/GroundingDINO) This is more challenging, the environment is setup in the workstation in the virtual environment ~/venv-gdino/
---
### Usage of the package : 

Bringup command for the system `ros2 launch dual_arm_workcell_bringup dual_arm_workcell_bringup.launch.py`
To simulate the system with fake hardware, use the `use_fake_hardware` flag. This will not launch any depth camera drivers.
Configure the ip and camera s.no parameters in the launch file when the system is changed.
The calibration of the system is done using the moveit_calibration package. The the relative orientation of the robots are 0.

---
### Usage of open_set_object_detection

This is the ros2 abstraction over GroundingDino object detection, the final executable is the script `./open_set_object_detection/script/get_object_locations.py`, note that this is setup in the workstation with the grounding dino python environment described in the dependencies part. The script is run in the python venv setup in the `~/venv-gdino` folder of the workstation. This **cannot be simulated**, for this to be used in another PC, the groundingdino folder generated while building the GroundingDino package must be copied into the foler `open_set_object_detection/scripts/` folder. Along with that make empty directories `open_set_object_detection/scripts/inference_images` and `open_set_object_detection/scripts/assets`. The node automatically subscribes to left and right camera image streams with the topics and namespaces in sync with the current bringup for the cameras.

---
### Usage of motion_planning_abstractions

This is the collection of general purpose motion planning abstraction nodes, each host one or more services(or actions) that perform a generic set of movements. Eg move to a configurable joint state, pick and place with cartesian motion, pick and place with ft feedback for touch etc. The launchfile `motion_planning_abstractions/launch/launch_servers.launch.py` is used to launch the configured nodes.

