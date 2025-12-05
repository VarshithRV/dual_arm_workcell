from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from ur_moveit_config.launch_common import load_yaml
from launch_param_builder import ParameterBuilder
from moveit_configs_utils import MoveItConfigsBuilder

import os
import yaml
import xacro


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .to_moveit_configs()
    )

    robot_description_kinematics = {
    "robot_description_kinematics": {
        "ur_manipulator": {
            "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
            "kinematics_solver_attempts": 3,
            "kinematics_solver_search_resolution": 0.005,
            "kinematics_solver_timeout": 0.005,
        },
    }}

    # left_pose_tracking_node = Node(
    #     package="motion_planning_abstractions",
    #     executable="pose_tracker",
    #     name="left_pose_tracker",
    #     output="screen",
    #     parameters=[
    #         robot_description_kinematics,
    #         {"use_sim_time": use_sim_time}, # setting use_sim_time = True screws up the planning scene monitor, but setting it up False just reads at time 0
    #         left_servo_params,
    #     ]
    # )

    servo_params = {
        "servo_node": ParameterBuilder("motion_planning_abstractions")
        .yaml("config/pose_tracker.yaml")
        .yaml("config/ur_servo.yaml")
        .to_dict()
    }

    preaction_server = Node(
        package="motion_planning_abstractions",
        executable="predefined_state_server",
        name="preaction_server",
        output="screen",
        parameters=[
            robot_description_kinematics,
            {
                "planning_group": "ur_manipulator",
                "shoulder_pan": 1.6606996059417725,
                "shoulder_lift": -1.4407718938640137,
                "elbow": -1.1456663608551025,
                "wrist_1": -2.125268121758932,
                "wrist_2": 1.45247220993042,
                "wrist_3": 1.677489995956421,
            },
            {"use_sim_time":use_sim_time},
        ],
    )

    rest_server = Node(
        package="motion_planning_abstractions",
        executable="predefined_state_server",
        name="rest_server",
        output="screen",
        parameters=[
            robot_description_kinematics,
            {
                "planning_group": "ur_manipulator",
                "shoulder_pan": -0.1496956984149378,
                "shoulder_lift": -1.468573884373047,
                "elbow": -1.9538769721984863,
                "wrist_1": -1.2700193685344239,
                "wrist_2": 1.9041476249694824,
                "wrist_3": -1.182901684437887,
            },
            {"use_sim_time":use_sim_time},
        ],
    )

    pose_tracking_node = Node(
        package="motion_planning_abstractions",
        executable="pose_tracker",
        output="screen",
        parameters=[
            
            robot_description_kinematics,
            {"use_sim_time": use_sim_time}, # setting use_sim_time = True screws up the planning scene monitor, but setting it up False just reads at time 0
            servo_params,
        ]
    )
    
    nodes_to_start = [
        # left_pose_tracking_node,
        pose_tracking_node,
        # preaction_server,
        # rest_server,
    ]
    
    return nodes_to_start


def generate_launch_description():
    
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])