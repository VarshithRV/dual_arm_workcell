from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="apriltag_grid_detector",
                executable="apriltag_grid_detector",
                name="apriltag_grid_detector",
                output="screen",
                parameters=[
                    {
                        "alpha": 0.25,
                        "marker_separation": 4.0,  # mm
                        "marker_size": 40.0,        # mm

                        "object.name": "object0",

                        # 2 rows x 2 cols, flattened [row0, row1, ...]
                        "grid.rows": 2,
                        "grid.cols": 2,
                        "grid.ids": [9, 10, 11, 12],

                        "color_image_topic": "/camera/right_camera/color/image_raw",
                        "camera_info_topic": "/camera/right_camera/color/camera_info",
                        "depth_image_topic": "/camera/right_camera/depth/image_rect_raw",
                        "detection_rate": 30.0,
                    }
                ],
            )
        ]
    )