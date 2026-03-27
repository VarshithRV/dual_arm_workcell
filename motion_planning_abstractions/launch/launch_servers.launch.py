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

import os
import yaml
import xacro
import math


def launch_setup(context, *args, **kwargs):
    left_robot_ip = LaunchConfiguration('left_robot_ip')
    left_translation = [-0.331,0.529,0.006]
    left_rotation = [0.0,0.0,math.pi/2] 
    left_tool0 = [0.0,0.0,0.189] # left tool0
    right_robot_ip = LaunchConfiguration('right_robot_ip')
    right_translation = [0.587,0.542,0.001]
    right_rotation = [0.0,0.0,math.pi/2]
    right_tool0 = [0.0,0.0,0.125] # right tool0
    
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    _publish_robot_description_semantic = LaunchConfiguration("publish_robot_description_semantic")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_joint_limits_file = LaunchConfiguration("moveit_joint_limits_file")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    left_ur_type = LaunchConfiguration("left_ur_type")
    left_safety_limits = LaunchConfiguration("left_safety_limits")
    left_safety_pos_margin = LaunchConfiguration("left_safety_pos_margin")
    left_safety_k_position = LaunchConfiguration("left_safety_k_position")
    left_kinematics_params_file = LaunchConfiguration("left_kinematics_params_file")
    
    left_joint_limit_params = PathJoinSubstitution([FindPackageShare(description_package), "config", left_ur_type, "joint_limits.yaml"])
    left_physical_params = PathJoinSubstitution([FindPackageShare(description_package), "config", left_ur_type, "physical_parameters.yaml"])
    left_visual_params = PathJoinSubstitution([FindPackageShare(description_package), "config", left_ur_type, "visual_parameters.yaml"])

    right_ur_type = LaunchConfiguration("right_ur_type")
    right_safety_limits = LaunchConfiguration("right_safety_limits")
    right_safety_pos_margin = LaunchConfiguration("right_safety_pos_margin")
    right_safety_k_position = LaunchConfiguration("right_safety_k_position")
    right_kinematics_params_file = LaunchConfiguration("right_kinematics_params_file")
    
    right_joint_limit_params = PathJoinSubstitution([FindPackageShare(description_package), "config", right_ur_type, "joint_limits.yaml"])
    right_physical_params = PathJoinSubstitution([FindPackageShare(description_package), "config", right_ur_type, "physical_parameters.yaml"])
    right_visual_params = PathJoinSubstitution([FindPackageShare(description_package), "config", right_ur_type, "visual_parameters.yaml"])

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "name:=dual_arm_workcell",
            " ",
            "left_robot_ip:=xxx.yyy.zzz.www",
            " ",
            "left_tf_prefix:=left_",
            " ",
            "left_joint_limit_params:=",
            left_joint_limit_params,
            " ",
            "left_kinematics_params:=",
            left_kinematics_params_file,
            " ",
            "left_physical_params:=",
            left_physical_params,
            " ",
            "left_visual_params:=",
            left_visual_params,
            " ",
            "left_safety_limits:=",
            left_safety_limits,
            " ",
            "left_safety_pos_margin:=",
            left_safety_pos_margin,
            " ",
            "left_safety_k_position:=",
            left_safety_k_position,
            " ",
            "left_name:=",
            "left_ur16e",
            " ",
            "left_script_filename:=",
            "ros_control.urscript",
            " ",
            "left_input_recipe_filename:=",
            "rtde_input_recipe.txt",
            " ",
            "left_output_recipe_filename:=",
            "rtde_output_recipe.txt",
            " ",
            "left_translation_x:=",
            str(left_translation[0]),
            " ",
            "left_translation_y:=",
            str(left_translation[1]),
            " ",
            "left_translation_z:=",
            str(left_translation[2]),
            " ",
            "left_rotation_r:=",
            str(left_rotation[0]),
            " ",
            "left_rotation_p:=",
            str(left_rotation[1]),
            " ",
            "left_rotation_y:=",
            str(left_rotation[2]),
            " ",
            "left_tool0_x:=",
            str(left_tool0[0]),
            " ",
            "left_tool0_y:=",
            str(left_tool0[1]),
            " ",
            "left_tool0_z:=",
            str(left_tool0[2]),
            " ",
            "right_robot_ip:=xxx.yyy.zzz.www",
            " ",
            "right_tf_prefix:=right_",
            " ",
            "right_joint_limit_params:=",
            right_joint_limit_params,
            " ",
            "right_kinematics_params:=",
            right_kinematics_params_file,
            " ",
            "right_physical_params:=",
            right_physical_params,
            " ",
            "right_visual_params:=",
            right_visual_params,
            " ",
            "right_safety_limits:=",
            right_safety_limits,
            " ",
            "right_safety_pos_margin:=",
            right_safety_pos_margin,
            " ",
            "right_safety_k_position:=",
            right_safety_k_position,
            " ",
            "right_name:=",
            "right_ur16e",
            " ",
            "right_script_filename:=",
            "ros_control.urscript",
            " ",
            "right_input_recipe_filename:=",
            "rtde_input_recipe.txt",
            " ",
            "right_output_recipe_filename:=",
            "rtde_output_recipe.txt",
            " ",
            "right_translation_x:=",
            str(right_translation[0]),
            " ",
            "right_translation_y:=",
            str(right_translation[1]),
            " ",
            "right_translation_z:=",
            str(right_translation[2]),
            " ",
            "right_rotation_r:=",
            str(right_rotation[0]),
            " ",
            "right_rotation_p:=",
            str(right_rotation[1]),
            " ",
            "right_rotation_y:=",
            str(right_rotation[2]),
            " ",
            "right_tool0_x:=",
            str(right_tool0[0]),
            " ",
            "right_tool0_y:=",
            str(right_tool0[1]),
            " ",
            "right_tool0_z:=",
            str(right_tool0[2]),
            " ",
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(value=robot_description_content, value_type=str)
    }

    # robot_description and param moveit param initialization is finished, now moveit config preparation needs to be done
    # moveit config stuff
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(moveit_config_package), "config", moveit_config_file]
            )
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(value=robot_description_semantic_content,value_type=str)
    }

    robot_description_kinematics = {
    "robot_description_kinematics": {
        "left_ur16e": {
            "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
            "kinematics_solver_attempts": 3,
            "kinematics_solver_search_resolution": 0.005,
            "kinematics_solver_timeout": 0.005,
        },
        "right_ur16e": {
            "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
            "kinematics_solver_attempts": 3,
            "kinematics_solver_search_resolution": 0.005,
            "kinematics_solver_timeout": 0.005,
        }
    }}

    #### nodes

    bare_bones_moveit_node = Node(
        package="motion_planning_abstractions",
        executable="bare_bones_moveit_example",
        name="bare_bones_moveit_example",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {"use_sim_time": use_sim_time},
            {
                "planning_group": "left_ur16e",
                "endeffector_link": "left_tool0",
                "tscubic_gen_traj_ns":"/left_task_space_cubic_polynomial_trajectory_server/generate_trajectory",
                "tscubic_exec_traj_ns":"/left_task_space_cubic_polynomial_trajectory_server/execute_trajectory",
                "set_io_ns":"/left_io_and_status_controller/set_io",
            },
        ]
    )

    ee_servo_example = Node(
        package="motion_planning_abstractions",
        executable="ee_servo_example",
        name="ee_servo_example",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {
                "servo_node_ns": "/left_servo_node_main",
                "joint_traj_controller": "left_scaled_joint_trajectory_controller",
                "joint_vel_controller": "left_forward_velocity_controller",
                "alpha":0.8
            },
        ]
    )

    ee_pose_tracker_example = Node(
        package="motion_planning_abstractions",
        executable="ee_pose_tracker_example",
        name="ee_pose_tracker_example",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {
                "planning_group": "left_ur16e",
                "endeffector_link": "left_tool0",
                "tscubic_gen_traj_ns":"/left_task_space_cubic_polynomial_trajectory_server/generate_trajectory",
                "tscubic_exec_traj_ns":"/left_task_space_cubic_polynomial_trajectory_server/execute_trajectory",
                "set_io_ns":"/left_io_and_status_controller/set_io",
                "servo_node_ns": "/left_servo_node_main",
                "joint_traj_controller": "left_scaled_joint_trajectory_controller",
                "joint_vel_controller": "left_forward_velocity_controller",
                "alpha":0.8,
                "linear_P":1.0,
                "linear_D":0.0,
                "angular_P":1.0,
                "angular_D":0.0,
            }
        ]
    )

    dual_arm_control_template_node = Node(
        package="motion_planning_abstractions",
        executable="dual_arm_control_template",
        name="dual_arm_control_template",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {"use_sim_time": use_sim_time},
            {
            },
        ]
    )

    left_pose_tracking_node = Node(
        package="motion_planning_abstractions",
        executable="pose_tracker_node",
        name="left_pose_tracker",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {"use_sim_time": use_sim_time},
            {
                "planning_group": "left_ur16e",
                "endeffector_link": "left_tool0",
                "servo_controller": "left_forward_velocity_controller",
                "non_servo_controller": "left_scaled_joint_trajectory_controller",
                "servo_node_namespace": "left_servo_node_main",
                "P_GAIN": 1.0,
                "I_GAIN": 0.0,
                "D_GAIN": 0.0,
                "K_GAIN": 1.0,
                "terminate":False, # should the tracking terminate?
                "linear_stop_threshold": 0.01, #m
                "angular_stop_threshold": 0.01, #rad
                "planning_frame":"world",
                "linear_iir_alpha":0.9, # range [0.0,1.0], more implies filter more
                "angular_iir_alpha":0.5, # range [0.0,1.0], more implies filter more
            },
        ]
    )

    rws_pick_and_place_server = Node(
    package="motion_planning_abstractions",
    executable="pick_and_place_local_perception_server",
    name="rws_pick_and_place_server",
    output="screen",
    parameters=[
        robot_description,
        robot_description_semantic,
        robot_description_kinematics,
        {
            "planning_group": "right_ur16e",
            "place_x":0.062,
            "place_y":0.608,
            "place_z":0.014,
            "orientation_w":0.002,
            "orientation_x":0.396,
            "orientation_y":0.918,
            "orientation_z":0.000,
            "pick_offset_x":0.0,
            "pick_offset_y":0.0,
            "pick_offset_z":-0.02,
            "look_offset_x":0.0,
            "look_offset_y":0.1,
            "look_offset_z":0.32,
            "place_step_x":0.05,
            "place_step_y":0.05,
            "height_of_movement":0.25,
            "endeffector_link": "right_tool0",
            "pin_out1":14,
            "pin_out2":0,
            "arm_side":"right"
        },
        {"use_sim_time":use_sim_time},
        ],
    )

    left_task_space_cubic_polynomial_trajectory_server = Node(
        package="motion_planning_abstractions",
        executable="task_space_cubic_polynomial_trajectory_server",
        name="left_task_space_cubic_polynomial_trajectory_server",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {
                "planning_group" : "left_ur16e",
                "maximum_task_space_velocity" : 1.0,
                "maximum_task_space_acceleration" : 3.0,
                "maximum_joint_space_velocity" : 3.15,
                "maximum_joint_space_acceleration" : 3.14,
                "arm_side" : "left",
                "joint_trajectory_controller" : "left_scaled_joint_trajectory_controller",
                "endeffector_link" : "left_tool0",
            },
            {"use_sim_time":use_sim_time},
        ],
    )
    
    right_task_space_cubic_polynomial_trajectory_server = Node(
        package="motion_planning_abstractions",
        executable="task_space_cubic_polynomial_trajectory_server",
        name="right_task_space_cubic_polynomial_trajectory_server",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {
                "planning_group_" : "right_ur16e",
                "maximum_task_space_velocity_" : 1.0,
                "maximum_task_space_acceleration_" : 3.0,
                "maximum_joint_space_velocity_" : 3.15,
                "maximum_joint_space_acceleration_" : 3.14,
                "arm_side" : "right",
                "joint_trajectory_controller_" : "right_scaled_joint_trajectory_controller",
                "endeffector_link_" : "right_tool0",
            },
            {"use_sim_time":use_sim_time},
        ],
    )

    suction_pick_and_place_server = Node(
    package="motion_planning_abstractions",
    executable="pick_and_place_ft_feedback_server",
    name="suction_pick_and_place_server",
    output="screen",
    parameters=[
        robot_description,
        robot_description_semantic,
        robot_description_kinematics,
        {
            "planning_group": "left_ur16e",
            "place_x":0.062,
            "place_y":0.608,
            "place_z":0.014,
            "orientation_w":0.001,
            "orientation_x":0.020,
            "orientation_y":1.000,
            "orientation_z":0.002,
            "pick_offset_x":0.0,
            "pick_offset_y":0.0,
            "pick_offset_z":-0.005,
            "look_offset_x":0.0,
            "look_offset_y":0.1,
            "look_offset_z":0.25,
            "place_step_x":0.05,
            "place_step_y":0.05,
            "height_of_movement":0.25,
            "endeffector_link": "left_tool0",
            "pin_out1":12,
            "pin_out2":0,
            "arm_side":"left",
            "servo_controller":"left_forward_velocity_controller",
            "joint_trajectory_controller":"left_scaled_joint_trajectory_controller",
            "ft_threshold": 17.0,
            "speed":0.13,
            "pretouch_distance":0.04,
        },
        {"use_sim_time":use_sim_time},
        ],
    )

    left_preaction_server = Node(
        package="motion_planning_abstractions",
        executable="predefined_state_server",
        name="left_preaction_server",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {
                "planning_group": "left_ur16e",
                "shoulder_pan": 1.6606996059417725,
                "shoulder_lift": -1.4407718938640137,
                "elbow": -1.1456663608551025,
                "wrist_1": -2.125268121758932,
                "wrist_2": 1.45247220993042,
                "wrist_3": 1.677489995956421,
                "side":"left",
            },
            {"use_sim_time":use_sim_time},
        ],
    )

    right_preaction_server = Node(
        package="motion_planning_abstractions",
        executable="predefined_state_server",
        name="right_preaction_server",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {
                "planning_group": "right_ur16e",
                "shoulder_pan": -4.906626049672262,
                "shoulder_lift": -1.626050134698385,
                "elbow": 1.2541254202472132,
                "wrist_1": -1.1249484878829499,
                "wrist_2": -1.3437789122210901,
                "wrist_3": -1.0042908827411097,
                "side":"right",
            },
            {"use_sim_time":use_sim_time},
        ],
    )

    left_rest_server = Node(
        package="motion_planning_abstractions",
        executable="predefined_state_server",
        name="left_rest_server",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {
                "planning_group": "left_ur16e",
                "shoulder_pan": -0.1496956984149378,
                "shoulder_lift": -1.468573884373047,
                "elbow": -1.9538769721984863,
                "wrist_1": -1.2700193685344239,
                "wrist_2": 1.9041476249694824,
                "wrist_3": -1.182901684437887,
                "side":"left",
            },
            {"use_sim_time":use_sim_time},
        ],
    )
    
    right_rest_server = Node(
        package="motion_planning_abstractions",
        executable="predefined_state_server",
        name="right_rest_server",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {
                "planning_group": "right_ur16e",
                "shoulder_pan": -3.13219124475588,
                "shoulder_lift": -1.8146683178343714,
                "elbow": 1.9976828734027308,
                "wrist_1": -1.8108145199217738,
                "wrist_2": -2.130192581807272,
                "wrist_3": 0.8935091495513916,
                "side":"right",
            },
            {"use_sim_time":use_sim_time},
        ],
    )

    nodes_to_start = [
        # left_pose_tracking_node,
        # rws_pick_and_place_server,
        # left_task_space_cubic_polynomial_trajectory_server,
        # right_task_space_cubic_polynomial_trajectory_server,
        # bare_bones_moveit_node,
        # ee_servo_example,
        ee_pose_tracker_example,
        # dual_arm_control_template_node,
        # suction_pick_and_place_server,
        # left_preaction_server,
        # right_preaction_server,
        left_rest_server,
        right_rest_server,
    ]
    
    return nodes_to_start


def generate_launch_description():
    
    declared_arguments = []

    # general arguments (used in launch_setup)
    declared_arguments.append(
        DeclareLaunchArgument(
            "publish_robot_description_semantic",
            default_value="true",
            description="publish robot description semantic?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="dual_arm_workcell_moveit_config",
            description="dual_arm_workcell_moveit_config",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_joint_limits_file",
            default_value="joint_limits.yaml",
            description="MoveIt joint limits configuration file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_file",
            default_value="dual_arm_workcell.srdf",
            description="MoveIt SRDF file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_servo",
            default_value="true",
            description="Launch Servo?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="dual_arm_workcell_description",
            description="Description package with robot URDF/XACRO files. Usually the argument "
            "is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="dual_arm_workcell.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )

    # left robot arguments (used in launch_setup)
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_ur_type",
            description="Type/series of used UR robot.",
            choices=[
                "ur3",
                "ur5",
                "ur10",
                "ur3e",
                "ur5e",
                "ur7e",
                "ur10e",
                "ur12e",
                "ur16e",
                "ur8long",
                "ur15",
                "ur18",
                "ur20",
                "ur30",
            ],
            default_value="ur16e",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_kinematics_params_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare(LaunchConfiguration("description_package")),
                    "config",
                    LaunchConfiguration("left_ur_type"),
                    "left_default_kinematics.yaml",
                ]
            ),
            description="The calibration configuration of the actual left robot used.",
        )
    )

    # right robot arguments (used in launch_setup)
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_ur_type",
            description="Type/series of used UR robot.",
            choices=[
                "ur3",
                "ur5",
                "ur10",
                "ur3e",
                "ur5e",
                "ur7e",
                "ur10e",
                "ur12e",
                "ur16e",
                "ur8long",
                "ur15",
                "ur18",
                "ur20",
                "ur30",
            ],
            default_value="ur16e",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_kinematics_params_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare(LaunchConfiguration("description_package")),
                    "config",
                    LaunchConfiguration("right_ur_type"),
                    "right_default_kinematics.yaml",
                ]
            ),
            description="The calibration configuration of the actual right robot used.",
        )
    )
    
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])