from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import os


def launch_setup():
    left_robot_ip = LaunchConfiguration('left_robot_ip')
    right_robot_ip = LaunchConfiguration('right_robot_ip')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    
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
            'right_robot_ip': right_robot_ip,
            'use_fake_hardware': use_fake_hardware,
        }.items()
    )

    dual_arm_workcell_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dual_arm_workcell_moveit_pkg, 'launch', 'dual_arm_workcell_moveit.launch.py')
        )
    )

    left_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense2_camera_pkg, 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'camera_name': 'left_camera',
            'align_depth.enable': 'true',
            'serial_no': '_210622060509',
            'pointcloud.enable': 'true',
            'spatial_filter.enable': 'true',
            'temporal_filter.enable': 'true',
            'hole_filling_filter.enable': 'true',
        }.items(),
        condition=UnlessCondition(use_fake_hardware),
    )

    left_camera_calibration_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'left_camera_left_wrist_3_link_calibration.launch.py')
        ),
        condition=UnlessCondition(use_fake_hardware),
    )

    right_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense2_camera_pkg, 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'camera_name': 'right_camera',
            'align_depth.enable': 'true',
            'serial_no': '_211122061649',
            'pointcloud.enable': 'true',
            'spatial_filter.enable': 'true',
            'temporal_filter.enable': 'true',
            'hole_filling_filter.enable': 'true',
        }.items(),
        condition=UnlessCondition(use_fake_hardware),
    )

    right_camera_calibration_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'right_camera_right_wrist_3_link_calibration.launch.py')
        ),
        condition=UnlessCondition(use_fake_hardware),
    )

    return [
        dual_arm_workcell_control_launch,
        dual_arm_workcell_moveit_launch,
        left_camera_launch,
        right_camera_launch,
        left_camera_calibration_launch,
        right_camera_calibration_launch,
    ]


def generate_launch_description():
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            name="left_robot_ip",
            default_value="192.168.1.4",
            description="Left ur16e ip address",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            name="right_robot_ip",
            default_value="192.168.1.3",
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

    nodes = launch_setup()
    return LaunchDescription(declared_arguments + nodes)
