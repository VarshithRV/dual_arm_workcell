""" Static transform publisher acquired via MoveIt 2 hand-eye calibration """
""" EYE-IN-HAND: right_wrist_3_link -> right_camera_link """
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    nodes = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="log",
            arguments=[
                "--frame-id",
                "right_wrist_3_link",
                "--child-frame-id",
                "right_camera_link",
                "--x",
                "-0.0799634",
                "--y",
                "-0.0605154",
                "--z",
                "0.0233057",
                "--qx",
                "0.280474",
                "--qy",
                "-0.662612",
                "--qz",
                "0.271475",
                "--qw",
                "0.639203",
                # "--roll",
                # "1.6201",
                # "--pitch",
                # "-0.768148",
                # "--yaw",
                # "1.60627",
            ],
        ),
    ]
    return LaunchDescription(nodes)
