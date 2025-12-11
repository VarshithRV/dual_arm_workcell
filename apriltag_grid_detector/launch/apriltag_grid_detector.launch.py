from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="your_package_name",
                executable="apriltag_grid_detector_node",
                name="apriltag_grid_detector",
                output="screen",
                parameters=[
                    {
                        "alpha": 0.25,
                        "marker_separation": 15.0,
                        "marker_size": 40.0,
                        "object.name": "object0",
                        "object.grid": [
                            [0, 1, 2],
                            [3, 4, 5],
                            [6, 7, 8],
                        ],
                        "color_image_topic": "/camera/color/image_raw",
                        "camera_info_topic": "/camera/color/camera_info",
                        "depth_image_topic": "/camera/depth/image_rect_raw",
                        "detection_rate": 30.0,
                    }
                ],
            )
        ]
    )
