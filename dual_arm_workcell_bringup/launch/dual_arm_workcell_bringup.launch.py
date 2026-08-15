from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import os
import math


def launch_setup():
    left_robot_ip = LaunchConfiguration('left_robot_ip')
    left_translation = [-0.46,0.529,0.00]
    left_rotation = [0.0,0.0,math.pi/2]
    right_robot_ip = LaunchConfiguration('right_robot_ip')
    right_translation = [0.46,0.529,0.00]
    right_rotation = [0.0,0.0,math.pi/2]
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_cameras = LaunchConfiguration('launch_cameras')
    
    dual_arm_workcell_driver_pkg = FindPackageShare('dual_arm_workcell_driver').find('dual_arm_workcell_driver')
    dual_arm_workcell_moveit_pkg = FindPackageShare('dual_arm_workcell_moveit_config').find('dual_arm_workcell_moveit_config')
    realsense2_camera_pkg = FindPackageShare('realsense2_camera').find('realsense2_camera')
    bringup_pkg = FindPackageShare('dual_arm_workcell_bringup').find('dual_arm_workcell_bringup')

    dual_arm_workcell_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dual_arm_workcell_driver_pkg, 'launch', 'dual_arm_workcell_control.launch.py')
        ),
        launch_arguments={
            'left_robot_ip': left_robot_ip,
            'left_translation_x': str(left_translation[0]),
            'left_translation_y': str(left_translation[1]),
            'left_translation_z': str(left_translation[2]),
            'left_rotation_r': str(left_rotation[0]),
            'left_rotation_p': str(left_rotation[1]),
            'left_rotation_y': str(left_rotation[2]),
            'right_robot_ip': right_robot_ip,
            'right_translation_x': str(right_translation[0]),
            'right_translation_y': str(right_translation[1]),
            'right_translation_z': str(right_translation[2]),
            'right_rotation_r': str(right_rotation[0]),
            'right_rotation_p': str(right_rotation[1]),
            'right_rotation_y': str(right_rotation[2]),
            'use_fake_hardware': use_fake_hardware,
            'use_sim_time': use_sim_time,
        }.items()
    )

    dual_arm_workcell_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dual_arm_workcell_moveit_pkg, 'launch', 'dual_arm_workcell_moveit.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'left_translation_x': str(left_translation[0]),
            'left_translation_y': str(left_translation[1]),
            'left_translation_z': str(left_translation[2]),
            'left_rotation_r': str(left_rotation[0]),
            'left_rotation_p': str(left_rotation[1]),
            'left_rotation_y': str(left_rotation[2]),
            'right_translation_x': str(right_translation[0]),
            'right_translation_y': str(right_translation[1]),
            'right_translation_z': str(right_translation[2]),
            'right_rotation_r': str(right_rotation[0]),
            'right_rotation_p': str(right_rotation[1]),
            'right_rotation_y': str(right_rotation[2]),
        }.items()
    )

    left_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense2_camera_pkg, 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'camera_name': 'left_camera',
            'align_depth.enable': 'true',
            'serial_no': '_135122075246',
            'pointcloud.enable': 'false',
            'spatial_filter.enable': 'false',
            'temporal_filter.enable': 'false',
            'hole_filling_filter.enable': 'false',
            'rgb_camera.color_profile':'640,480,30',
            'enable_depth':'false',
        }.items(),
        condition=UnlessCondition(use_fake_hardware) and IfCondition(launch_cameras),
    )

    left_camera_calibration_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'left_camera_left_wrist_3_link_calibration.launch.py')
        ),
    )

    right_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense2_camera_pkg, 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'camera_name': 'right_camera',
            'align_depth.enable': 'true',
            'serial_no': '_135122075246',
            'pointcloud.enable': 'false',
            'spatial_filter.enable': 'false',
            'temporal_filter.enable': 'false',
            'hole_filling_filter.enable': 'false',
            'rgb_camera.color_profile':'640,480,30',
            'depth_module.color_profile':'640,480,30',
        }.items(),
        condition=UnlessCondition(use_fake_hardware) and IfCondition(launch_cameras),
    )

    right_camera_calibration_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'right_camera_right_wrist_3_link_calibration.launch.py')
        ),
    )

    return [
        dual_arm_workcell_control_launch,
        dual_arm_workcell_moveit_launch,
        left_camera_launch,
        # right_camera_launch,
        left_camera_calibration_launch,
        # right_camera_calibration_launch,
    ]


def generate_launch_description():
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            name="left_robot_ip",
            default_value="192.168.1.3",
            description="Left ur16e ip address",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            name="right_robot_ip",
            default_value="192.168.1.6",
            description="Right ur16e ip address",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            name="use_fake_hardware",
            default_value="false",
            description="Use fake hardware?",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            name="use_sim_time",
            default_value="false",
            description="Use sim time?",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            name="launch_camera",
            default_value="true",
            description="Launch Cameras?",
        )
    )

    nodes = launch_setup()
    return LaunchDescription(declared_arguments + nodes)
