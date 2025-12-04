# Copyright (c) 2021 PickNik, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Denis Stogl

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    AndSubstitution,
    Command,
    FindExecutable,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare, FindPackage
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue


def launch_setup(context, *args, **kwargs):
    
    use_sim_time = LaunchConfiguration("use_sim_time")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    launch_rviz = LaunchConfiguration("launch_rviz")
    headless_mode = LaunchConfiguration("headless_mode")
    controllers_file = LaunchConfiguration("controllers_file")
    controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")

    left_ur_type = LaunchConfiguration("left_ur_type")
    left_robot_ip = LaunchConfiguration("left_robot_ip")
    left_safety_limits = LaunchConfiguration("left_safety_limits")
    left_safety_pos_margin = LaunchConfiguration("left_safety_pos_margin")
    left_safety_k_position = LaunchConfiguration("left_safety_k_position")
    left_tf_prefix = LaunchConfiguration("left_tf_prefix")
    left_use_tool_communication = LaunchConfiguration("left_use_tool_communication")
    left_kinematics_params_file = LaunchConfiguration("left_kinematics_params_file")
    left_tool_parity = LaunchConfiguration("left_tool_parity")
    left_tool_baud_rate = LaunchConfiguration("left_tool_baud_rate")
    left_tool_stop_bits = LaunchConfiguration("left_tool_stop_bits")
    left_tool_rx_idle_chars = LaunchConfiguration("left_tool_rx_idle_chars")
    left_tool_tx_idle_chars = LaunchConfiguration("left_tool_tx_idle_chars")
    left_tool_device_name = LaunchConfiguration("left_tool_device_name")
    left_tool_tcp_port = LaunchConfiguration("left_tool_tcp_port")
    left_tool_voltage = LaunchConfiguration("left_tool_voltage")
    left_reverse_ip = LaunchConfiguration("left_reverse_ip")
    left_script_command_port = LaunchConfiguration("left_script_command_port")
    left_reverse_port = LaunchConfiguration("left_reverse_port")
    left_script_sender_port = LaunchConfiguration("left_script_sender_port")
    left_trajectory_port = LaunchConfiguration("left_trajectory_port")
    left_joint_limit_params = PathJoinSubstitution([FindPackageShare(description_package), "config", left_ur_type, "joint_limits.yaml"])
    left_physical_params = PathJoinSubstitution([FindPackageShare(description_package), "config", left_ur_type, "physical_parameters.yaml"])
    left_visual_params = PathJoinSubstitution([FindPackageShare(description_package), "config", left_ur_type, "visual_parameters.yaml"])
    left_script_filename = PathJoinSubstitution([FindPackageShare("ur_client_library"), "resources", "external_control.urscript"])
    left_input_recipe_filename = PathJoinSubstitution([FindPackageShare("ur_robot_driver"), "resources", "rtde_input_recipe.txt"])
    left_output_recipe_filename = PathJoinSubstitution([FindPackageShare("ur_robot_driver"), "resources", "rtde_output_recipe.txt"])

    right_ur_type = LaunchConfiguration("right_ur_type")
    right_robot_ip = LaunchConfiguration("right_robot_ip")
    right_safety_limits = LaunchConfiguration("right_safety_limits")
    right_safety_pos_margin = LaunchConfiguration("right_safety_pos_margin")
    right_safety_k_position = LaunchConfiguration("right_safety_k_position")
    right_tf_prefix = LaunchConfiguration("right_tf_prefix")
    right_use_tool_communication = LaunchConfiguration("right_use_tool_communication")
    right_kinematics_params_file = LaunchConfiguration("right_kinematics_params_file")
    right_tool_parity = LaunchConfiguration("right_tool_parity")
    right_tool_baud_rate = LaunchConfiguration("right_tool_baud_rate")
    right_tool_stop_bits = LaunchConfiguration("right_tool_stop_bits")
    right_tool_rx_idle_chars = LaunchConfiguration("right_tool_rx_idle_chars")
    right_tool_tx_idle_chars = LaunchConfiguration("right_tool_tx_idle_chars")
    right_tool_device_name = LaunchConfiguration("right_tool_device_name")
    right_tool_tcp_port = LaunchConfiguration("right_tool_tcp_port")
    right_tool_voltage = LaunchConfiguration("right_tool_voltage")
    right_reverse_ip = LaunchConfiguration("right_reverse_ip")
    right_script_command_port = LaunchConfiguration("right_script_command_port")
    right_reverse_port = LaunchConfiguration("right_reverse_port")
    right_script_sender_port = LaunchConfiguration("right_script_sender_port")
    right_trajectory_port = LaunchConfiguration("right_trajectory_port")
    right_joint_limit_params = PathJoinSubstitution([FindPackageShare(description_package), "config", right_ur_type, "joint_limits.yaml"])
    right_physical_params = PathJoinSubstitution([FindPackageShare(description_package), "config", right_ur_type, "physical_parameters.yaml"])
    right_visual_params = PathJoinSubstitution([FindPackageShare(description_package), "config", right_ur_type, "visual_parameters.yaml"])
    right_script_filename = PathJoinSubstitution([FindPackageShare("ur_client_library"), "resources", "external_control.urscript"])
    right_input_recipe_filename = PathJoinSubstitution([FindPackageShare("ur_robot_driver"), "resources", "rtde_input_recipe.txt"])
    right_output_recipe_filename = PathJoinSubstitution([FindPackageShare("ur_robot_driver"), "resources", "rtde_output_recipe.txt"])

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "name:=dual_arm_workcell",
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "fake_sensor_commands:=",
            fake_sensor_commands,
            " ",
            "left_robot_ip:=",
            left_robot_ip,
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
            left_script_filename,
            " ",
            "left_input_recipe_filename:=",
            left_input_recipe_filename,
            " ",
            "left_output_recipe_filename:=",
            left_output_recipe_filename,
            " ",
            "left_tf_prefix:=",
            left_tf_prefix,
            " ",
            "left_headless_mode:=",
            headless_mode,
            " ",
            "left_use_tool_communication:=",
            left_use_tool_communication,
            " ",
            "left_tool_parity:=",
            left_tool_parity,
            " ",
            "left_tool_baud_rate:=",
            left_tool_baud_rate,
            " ",
            "left_tool_stop_bits:=",
            left_tool_stop_bits,
            " ",
            "left_tool_rx_idle_chars:=",
            left_tool_rx_idle_chars,
            " ",
            "left_tool_tx_idle_chars:=",
            left_tool_tx_idle_chars,
            " ",
            "left_tool_device_name:=",
            left_tool_device_name,
            " ",
            "left_tool_tcp_port:=",
            left_tool_tcp_port,
            " ",
            "left_tool_voltage:=",
            left_tool_voltage,
            " ",
            "left_reverse_ip:=",
            left_reverse_ip,
            " ",
            "left_script_command_port:=",
            left_script_command_port,
            " ",
            "left_reverse_port:=",
            left_reverse_port,
            " ",
            "left_script_sender_port:=",
            left_script_sender_port,
            " ",
            "left_trajectory_port:=",
            left_trajectory_port,
            " ",
            "right_robot_ip:=",
            right_robot_ip,
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
            right_script_filename,
            " ",
            "right_input_recipe_filename:=",
            right_input_recipe_filename,
            " ",
            "right_output_recipe_filename:=",
            right_output_recipe_filename,
            " ",
            "right_tf_prefix:=",
            right_tf_prefix,
            " ",
            "right_headless_mode:=",
            headless_mode,
            " ",
            "right_use_tool_communication:=",
            right_use_tool_communication,
            " ",
            "right_tool_parity:=",
            right_tool_parity,
            " ",
            "right_tool_baud_rate:=",
            right_tool_baud_rate,
            " ",
            "right_tool_stop_bits:=",
            right_tool_stop_bits,
            " ",
            "right_tool_rx_idle_chars:=",
            right_tool_rx_idle_chars,
            " ",
            "right_tool_tx_idle_chars:=",
            right_tool_tx_idle_chars,
            " ",
            "right_tool_device_name:=",
            right_tool_device_name,
            " ",
            "right_tool_tcp_port:=",
            right_tool_tcp_port,
            " ",
            "right_tool_voltage:=",
            right_tool_voltage,
            " ",
            "right_reverse_ip:=",
            right_reverse_ip,
            " ",
            "right_script_command_port:=",
            right_script_command_port,
            " ",
            "right_reverse_port:=",
            right_reverse_port,
            " ",
            "right_script_sender_port:=",
            right_script_sender_port,
            " ",
            "right_trajectory_port:=",
            right_trajectory_port,
            " ",
        ]
    )

    robot_description = {"robot_description": ParameterValue(value=robot_description_content, value_type=str)}

    rviz_config_file = PathJoinSubstitution([FindPackageShare(description_package), "rviz", "view_robot.rviz"])

    initial_joint_controllers = PathJoinSubstitution([FindPackageShare(runtime_config_package), "config", controllers_file])

    update_rate_config_file = PathJoinSubstitution(
        [
            FindPackageShare(runtime_config_package),
            "config",
            "ur16e" + "_update_rate.yaml",
        ]
    )
    
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            update_rate_config_file,
            ParameterFile(initial_joint_controllers, allow_substs=True),
            {"use_sim_time":use_sim_time},
        ],
        output="screen",
        condition=IfCondition(use_fake_hardware),
    )

    ur_control_node = Node(
        package="ur_robot_driver",
        executable="ur_ros2_control_node",
        parameters=[
            robot_description,
            update_rate_config_file,
            ParameterFile(initial_joint_controllers, allow_substs=True),
            {"use_sim_time":use_sim_time},
        ],
        output="screen",
        condition=UnlessCondition(use_fake_hardware),
    )

    def controller_spawner(controllers, active=True):
        inactive_flags = ["--inactive"] if not active else []
        return Node(
            package="controller_manager",
            executable="spawner",
            name="spawner",
            arguments=[
                "--controller-manager",
                "controller_manager",
                "--controller-manager-timeout",
                controller_spawner_timeout,
            ]
            + inactive_flags
            + controllers,
            parameters=[
                {"use_sim_time":use_sim_time},
            ]
        )
    
    controllers_active = [
        "joint_state_broadcaster",
        "left_scaled_joint_trajectory_controller",
        "left_io_and_status_controller",
        "left_speed_scaling_state_broadcaster",
        "left_force_torque_sensor_broadcaster",
        "left_tcp_pose_broadcaster",
        "left_ur_configuration_controller",
        "right_scaled_joint_trajectory_controller",
        "right_io_and_status_controller",
        "right_speed_scaling_state_broadcaster",
        "right_force_torque_sensor_broadcaster",
        "right_tcp_pose_broadcaster",
        "right_ur_configuration_controller",
    ]
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
    )

    controllers_inactive = [
        "left_joint_trajectory_controller",
        "left_forward_velocity_controller",
        "left_forward_position_controller",
        "left_forward_effort_controller",
        "left_force_mode_controller",
        "left_passthrough_trajectory_controller",
        "left_freedrive_mode_controller",
        "left_tool_contact_controller",
        "right_joint_trajectory_controller",
        "right_forward_velocity_controller",
        "right_forward_position_controller",
        "right_forward_effort_controller",
        "right_force_mode_controller",
        "right_passthrough_trajectory_controller",
        "right_freedrive_mode_controller",
        "right_tool_contact_controller",
    ]

    if use_fake_hardware.perform(context) == "true":
        controllers_active.remove("left_tcp_pose_broadcaster")
        controllers_active.remove("right_tcp_pose_broadcaster")

    controller_spawners = [
        controller_spawner(controllers_active),
        controller_spawner(controllers_inactive, active=False),
    ]

    nodes_to_start = [
        ur_control_node,
        control_node,
        robot_state_publisher_node,
        rviz_node,
    ] + controller_spawners

    return nodes_to_start 


def generate_launch_description():
    declared_arguments = []
    
    # general arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time?",
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
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="dual_arm_workcell_driver",
            description='Package with the controller\'s configuration in "config" folder. '
            "Usually the argument is not set, it enables use of a custom setup.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description="Enable fake command interfaces for sensors used for simple simulations. "
            "Used only if 'use_fake_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "headless_mode",
            default_value="false",
            description="Enable headless mode for robot control",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_spawner_timeout",
            default_value="10",
            description="Timeout used when spawning controllers for the left arm.",
        )
    )

    # left robot arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_robot_ip",
            description="Ip address of the left robot",
        )
    )
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
            "left_tf_prefix",
            default_value='"left_"',
            description="Prefix of the joint names, useful for "
            "multi-robot setup. If changed than also joint names in the controllers' configuration "
            "have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="combined_ur_controllers.yaml",
            description="YAML file with the controllers configuration for the left robot.",
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
    declared_arguments.append(
        DeclareLaunchArgument(
            "activate_joint_controller",
            default_value="true",
            description="Activate loaded joint controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_use_tool_communication",
            default_value="false",
            description="Only available for e series!",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_tool_parity",
            default_value="0",
            description="Parity configuration for serial communication. Only effective, if "
            "use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_tool_baud_rate",
            default_value="115200",
            description="Baud rate configuration for serial communication. Only effective, if "
            "use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_tool_stop_bits",
            default_value="1",
            description="Stop bits configuration for serial communication. Only effective, if "
            "use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_tool_rx_idle_chars",
            default_value="1.5",
            description="RX idle chars configuration for serial communication. Only effective, "
            "if use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_tool_tx_idle_chars",
            default_value="3.5",
            description="TX idle chars configuration for serial communication. Only effective, "
            "if use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_tool_device_name",
            default_value="/tmp/ttyleftUR", #### might cause problems
            description="File descriptor that will be generated for the tool communication device. "
            "The user has be be allowed to write to this location. "
            "Only effective, if use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_tool_tcp_port",
            default_value="54321",
            description="Remote port that will be used for bridging the tool's serial device. "
            "Only effective, if use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_tool_voltage",
            default_value="0",  # 0 being a conservative value that won't destroy anything
            description="Tool voltage that will be setup.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_reverse_ip",
            default_value="0.0.0.0",
            description="IP that will be used for the robot controller to communicate back to the driver.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_script_command_port",
            default_value="50004",
            description="Port that will be opened to forward URScript commands to the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_reverse_port",
            default_value="50001",
            description="Port that will be opened to send cyclic instructions from the driver to the robot controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_script_sender_port",
            default_value="50002",
            description="The driver will offer an interface to query the external_control URScript on this port.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_trajectory_port",
            default_value="50003",
            description="Port that will be opened for trajectory control.",
        )
    )

    # right robot arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_robot_ip",
            description="Ip address of the right robot",
        )
    )
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
            "right_tf_prefix",
            default_value='"right_"',
            description="Prefix of the joint names, useful for "
            "multi-robot setup. If changed than also joint names in the controllers' configuration "
            "have to be updated.",
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
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_use_tool_communication",
            default_value="false",
            description="Only available for e series!",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_tool_parity",
            default_value="0",
            description="Parity configuration for serial communication. Only effective, if "
            "use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_tool_baud_rate",
            default_value="115200",
            description="Baud rate configuration for serial communication. Only effective, if "
            "use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_tool_stop_bits",
            default_value="1",
            description="Stop bits configuration for serial communication. Only effective, if "
            "use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_tool_rx_idle_chars",
            default_value="1.5",
            description="RX idle chars configuration for serial communication. Only effective, "
            "if use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_tool_tx_idle_chars",
            default_value="3.5",
            description="TX idle chars configuration for serial communication. Only effective, "
            "if use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_tool_device_name",
            default_value="/tmp/ttyrightUR", #### might cause problems
            description="File descriptor that will be generated for the tool communication device. "
            "The user has be be allowed to write to this location. "
            "Only effective, if use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_tool_tcp_port",
            default_value="64321",
            description="Remote port that will be used for bridging the tool's serial device. "
            "Only effective, if use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_tool_voltage",
            default_value="0",  # 0 being a conservative value that won't destroy anything
            description="Tool voltage that will be setup.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_reverse_ip",
            default_value="0.0.0.0",
            description="IP that will be used for the robot controller to communicate back to the driver.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_script_command_port",
            default_value="60004",
            description="Port that will be opened to forward URScript commands to the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_reverse_port",
            default_value="60001",
            description="Port that will be opened to send cyclic instructions from the driver to the robot controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_script_sender_port",
            default_value="60002",
            description="The driver will offer an interface to query the external_control URScript on this port.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_trajectory_port",
            default_value="60003",
            description="Port that will be opened for trajectory control.",
        )
    )
    
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])