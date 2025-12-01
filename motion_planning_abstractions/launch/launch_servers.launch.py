from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml
import xacro


def generate_launch_description():

    robot_description_kinematics = {
    "robot_description_kinematics": {
        "left_ur16e": {
            "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
            "kinematics_solver_attempts": 3,
            "kinematics_solver_search_resolution": 0.005,
            "kinematics_solver_timeout": 0.005,},
        "right_ur16e": {
            "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
            "kinematics_solver_attempts": 3,
            "kinematics_solver_search_resolution": 0.005,
            "kinematics_solver_timeout": 0.005,}
    }}
    rws_pick_and_place_server = Node(
    package="motion_planning_abstractions",
    executable="pick_and_place_local_perception_server",
    name="rws_pick_and_place_server",
    output="screen",
    parameters=[
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
        {"use_sim_time":True},
        ],
    )

    suction_pick_and_place_server = Node(
    package="motion_planning_abstractions",
    executable="pick_and_place_ft_feedback_server",
    name="suction_pick_and_place_server",
    output="screen",
    parameters=[
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
            "ft_threshold": 17.0,
            "speed":0.13,
            "pretouch_distance":0.04,
        },
        # {"use_sim_time":True},
        ],
    )

    left_preaction_server = Node(
        package="motion_planning_abstractions",
        executable="predefined_state_server",
        name="left_preaction_server",
        output="screen",
        parameters=[
            robot_description_kinematics,
            {
                "planning_group": "left_ur16e",
                "shoulder_pan": 1.6606996059417725,
                "shoulder_lift": -1.4407718938640137,
                "elbow": -1.1456663608551025,
                "wrist_1": -2.125268121758932,
                "wrist_2": 1.45247220993042,
                "wrist_3": 1.677489995956421,
            },
        ],
    )

    right_preaction_server = Node(
        package="motion_planning_abstractions",
        executable="predefined_state_server",
        name="right_preaction_server",
        output="screen",
        parameters=[
            robot_description_kinematics,
            {
                "planning_group": "right_ur16e",
                "shoulder_pan": -4.906626049672262,
                "shoulder_lift": -1.626050134698385,
                "elbow": 1.2541254202472132,
                "wrist_1": -1.1249484878829499,
                "wrist_2": -1.3437789122210901,
                "wrist_3": -1.0042908827411097,
            },
        ],
    )

    left_rest_server = Node(
        package="motion_planning_abstractions",
        executable="predefined_state_server",
        name="left_rest_server",
        output="screen",
        parameters=[
            robot_description_kinematics,
            {
                "planning_group": "left_ur16e",
                "shoulder_pan": -0.1496956984149378,
                "shoulder_lift": -1.468573884373047,
                "elbow": -1.9538769721984863,
                "wrist_1": -1.2700193685344239,
                "wrist_2": 1.9041476249694824,
                "wrist_3": -1.182901684437887,
            },
        ],
    )
    
    right_rest_server = Node(
        package="motion_planning_abstractions",
        executable="predefined_state_server",
        name="right_rest_server",
        output="screen",
        parameters=[
            robot_description_kinematics,
            {
                "planning_group": "right_ur16e",
                "shoulder_pan": -3.13219124475588,
                "shoulder_lift": -1.8146683178343714,
                "elbow": 1.9976828734027308,
                "wrist_1": -1.8108145199217738,
                "wrist_2": -2.130192581807272,
                "wrist_3": 0.8935091495513916,
            },
        ],
    )

    return LaunchDescription([
        rws_pick_and_place_server,
        suction_pick_and_place_server,
        left_preaction_server,
        right_preaction_server,
        left_rest_server,
        right_rest_server,
        ])