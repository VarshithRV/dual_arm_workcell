#!/usr/bin/env python3

import copy
import sys
import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseStamped
from image_geometry import PinholeCameraModel
import cv_bridge
import apriltag
from scipy.spatial.transform import Rotation as R


class Deprojection(Node):
    def __init__(self) -> None:
        super().__init__("apriltag_grid_detector")

        # --- Bridge and camera model ---
        self.depth_image = None
        self.camera_info = None
        self.grayscale_image = None
        self.cv_bridge = cv_bridge.CvBridge()
        self.camera_model = PinholeCameraModel()

        # --- AprilTag + ArUco detectors ---
        self.apriltag_detector_options = apriltag.DetectorOptions()
        self.apriltag_detector = apriltag.Detector(self.apriltag_detector_options)

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_APRILTAG_36h11
        )

        # --- Parameters ---
        self.declare_parameter("alpha", 0.25)
        self.declare_parameter("marker_separation", 15.0)  # mm
        self.declare_parameter("marker_size", 40.0)        # mm

        # Object name
        self.declare_parameter("object.name", "object0")

        # Grid description: rows, cols, and flattened IDs (row-major)
        self.declare_parameter("grid.rows", 2)
        self.declare_parameter("grid.cols", 2)
        self.declare_parameter("grid.ids", [0, 1, 2, 3])

        # Camera topics
        self.declare_parameter("color_image_topic", "/camera/right_camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/right_camera/color/camera_info")
        self.declare_parameter("depth_image_topic", "/camera/right_camera/depth/image_rect_raw")
        self.declare_parameter("detection_rate", 30.0)

        # --- Read parameters ---
        self.alpha = float(self.get_parameter("alpha").value)
        self.marker_separation = float(self.get_parameter("marker_separation").value) / 1000.0
        self.marker_size = float(self.get_parameter("marker_size").value) / 1000.0

        self.object_name = str(self.get_parameter("object.name").value)

        grid_rows = int(self.get_parameter("grid.rows").value)
        grid_cols = int(self.get_parameter("grid.cols").value)
        grid_ids = self.get_parameter("grid.ids").value  # list of ints

        if len(grid_ids) != grid_rows * grid_cols:
            self.get_logger().error(
                f"grid.ids length ({len(grid_ids)}) != grid.rows*grid.cols ({grid_rows}*{grid_cols})"
            )
            raise RuntimeError("Invalid grid parameter sizes")

        self.grid = np.array(grid_ids, dtype=int).reshape((grid_rows, grid_cols))

        camera_color_topic = str(self.get_parameter("color_image_topic").value)
        camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        camera_depth_topic = str(self.get_parameter("depth_image_topic").value)
        self.detection_rate = float(self.get_parameter("detection_rate").value)


        self.get_logger().info("Apriltag grid detector node started with params:")
        self.get_logger().info(f"  Alpha: {self.alpha}")
        self.get_logger().info(f"  Marker Separation (m): {self.marker_separation}")
        self.get_logger().info(f"  Marker Size (m): {self.marker_size}")
        self.get_logger().info(f"  Object Name: {self.object_name}")
        self.get_logger().info(f"  Grid:\n{self.grid}")
        self.get_logger().info(f"  Color image topic: {camera_color_topic}")
        self.get_logger().info(f"  Camera info topic: {camera_info_topic}")
        self.get_logger().info(f"  Depth image topic: {camera_depth_topic}")
        self.get_logger().info(f"  Detection rate: {self.detection_rate} Hz")

        # Pose-related state
        self.estimated_pose = None
        self.filtered_pose = None
        self.rvec = None
        self.tvec = None
        self.tvec_impulse = None
        self.filtered_tvec = np.zeros(shape=(3,), dtype=float)
        self.qvec = None
        self.qvec_impulse = None
        self.filtered_qvec = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        self.mutex = threading.Lock()
        self.detected_image = None

        # Topics
        detected_image_topic = "~/detected_image_raw"
        pose_topic = f"~/{self.object_name}_pose"
        filtered_pose_topic = f"~/{self.object_name}_filtered_pose"

        # --- Publishers ---
        self.detected_image_pub = self.create_publisher(Image, detected_image_topic, 10)
        self.pose_pub = self.create_publisher(PoseStamped, pose_topic, 10)
        self.filtered_pose_pub = self.create_publisher(
            PoseStamped, filtered_pose_topic, 10
        )

        # --- Subscribers ---
        self.depth_image_sub = self.create_subscription(
            Image, camera_depth_topic, self.depth_image_callback, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback, 10
        )
        self.color_image_sub = self.create_subscription(
            Image, camera_color_topic, self.color_image_callback, 10
        )

        # --- Timers ---
        period = 1.0 / self.detection_rate
        self.apriltag_detector_timer = self.create_timer(
            period, self.apriltag_detector_cb
        )
        self.filtered_pose_publisher_timer = self.create_timer(
            period, self.filtered_pose_publisher_cb
        )
        self.pose_publisher_timer = self.create_timer(
            period, self.pose_publisher_cb
        )

    def draw_axes(self, image, rvec, tvec, intrinsics, distortion_coeffs, axis_length=0.05):
        axis_3d = np.float32(
            [
                [0, 0, 0],               # origin
                [axis_length, 0, 0],     # X axis
                [0, axis_length, 0],     # Y axis
                [0, 0, axis_length],     # Z axis
            ]
        ).reshape(-1, 3)

        imgpts, _ = cv2.projectPoints(axis_3d, rvec, tvec, intrinsics, distortion_coeffs)
        imgpts = imgpts.astype(int)

        origin = tuple(imgpts[0].ravel())
        x_axis = tuple(imgpts[1].ravel())
        y_axis = tuple(imgpts[2].ravel())
        z_axis = tuple(imgpts[3].ravel())

        cv2.line(image, origin, x_axis, (0, 0, 255), 2)
        cv2.line(image, origin, y_axis, (0, 255, 0), 2)
        cv2.line(image, origin, z_axis, (255, 0, 0), 2)
        cv2.circle(image, origin, 3, (0, 0, 0), -1)

        return image

    def apriltag_detector_cb(self):
        if self.grayscale_image is None or self.camera_info is None:
            return

        image_detected, rvec, tvec = self.detect_apriltags(self.grayscale_image)
        if image_detected is not None:
            image_msg = self.cv_bridge.cv2_to_imgmsg(image_detected, encoding="bgr8")
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = self.camera_info.header.frame_id
            self.detected_image_pub.publish(image_msg)

        if tvec is not None and rvec is not None:
            self.tvec = tvec
            r = R.from_rotvec(rvec)
            self.qvec = r.as_quat()  # [x, y, z, w]
        else:
            self.tvec = None
            self.qvec = None

    def pose_publisher_cb(self):
        if self.qvec is None or self.tvec is None or self.camera_info is None:
            return

        msg = PoseStamped()
        msg.header.frame_id = self.camera_info.header.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(self.tvec[0])
        msg.pose.position.y = float(self.tvec[1])
        msg.pose.position.z = float(self.tvec[2])
        msg.pose.orientation.x = float(self.qvec[0])
        msg.pose.orientation.y = float(self.qvec[1])
        msg.pose.orientation.z = float(self.qvec[2])
        msg.pose.orientation.w = float(self.qvec[3])

        self.pose_pub.publish(msg)

    def filter_pose(self):
        # SLERP for quaternion, LERP for translation
        q1 = self.filtered_qvec
        q2 = self.qvec

        dot = float(np.dot(q1, q2))
        if dot < 0.0:
            q2 = -q2
            dot = -dot
        if dot > 0.9995:
            filtered_quat = self.alpha * q1 + (1.0 - self.alpha) * q2
        else:
            theta_0 = np.arccos(dot)
            sin_theta_0 = np.sin(theta_0)
            theta = theta_0 * (1.0 - self.alpha)
            sin_theta = np.sin(theta)

            s1 = np.sin(theta_0 - theta) / sin_theta_0
            s2 = sin_theta / sin_theta_0
            filtered_quat = s1 * q1 + s2 * q2

        filtered_quat = filtered_quat / np.linalg.norm(filtered_quat)
        self.filtered_qvec = filtered_quat
        self.filtered_tvec = self.alpha * self.filtered_tvec + (1.0 - self.alpha) * self.tvec

    def filtered_pose_publisher_cb(self):
        if self.tvec is None or self.qvec is None or self.camera_info is None:
            return

        self.filter_pose()

        msg = PoseStamped()
        msg.header.frame_id = self.camera_info.header.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(self.filtered_tvec[0])
        msg.pose.position.y = float(self.filtered_tvec[1])
        msg.pose.position.z = float(self.filtered_tvec[2])
        msg.pose.orientation.x = float(self.filtered_qvec[0])
        msg.pose.orientation.y = float(self.filtered_qvec[1])
        msg.pose.orientation.z = float(self.filtered_qvec[2])
        msg.pose.orientation.w = float(self.filtered_qvec[3])

        self.filtered_pose_pub.publish(msg)

    def detect_apriltags(self, grayscale_image):
        detections = self.apriltag_detector.detect(grayscale_image)
        detected_image = cv2.cvtColor(copy.deepcopy(grayscale_image), cv2.COLOR_GRAY2RGB)

        if not len(detections):
            return detected_image, None, None

        image_points_grid_list = []
        object_points_grid_list = []

        for det in detections:
            tag_id = det.tag_id
            x1y1 = det.corners[0]  # top left
            x1y2 = det.corners[3]  # bottom left
            x2y1 = det.corners[1]  # top right
            x2y2 = det.corners[2]  # bottom right

            for pt in [x1y1, x1y2, x2y2, x2y1]:
                detected_image = cv2.circle(
                    detected_image,
                    center=(int(pt[0]), int(pt[1])),
                    radius=2,
                    color=(0, 255, 0),
                    thickness=2,
                )

            image_points = np.stack(
                (
                    x1y1.reshape((1, 2)),  # top left
                    x2y1.reshape((1, 2)),  # top right
                    x1y2.reshape((1, 2)),  # bottom left
                    x2y2.reshape((1, 2)),  # bottom right
                ),
                axis=1,
            )[0]

            if_tag_exists_in_grid = np.any(self.grid == tag_id)
            if if_tag_exists_in_grid:
                grid_positionx, grid_positiony = np.where(self.grid == tag_id)
                grid_position = np.concatenate((grid_positionx, grid_positiony), axis=0)
                grid_origin = np.array([self.grid.shape[0] - 1, 0])  # bottom-left marker

                # compute object points in grid frame (meters)
                dx = (grid_position[1] - grid_origin[1]) * (self.marker_size + self.marker_separation)
                dy = (grid_origin[0] - grid_position[0]) * (self.marker_size + self.marker_separation)

                object_points = np.array(
                    [
                        [dx, dy + self.marker_size, 0.0],                       # top left
                        [dx + self.marker_size, dy + self.marker_size, 0.0],   # top right
                        [dx, dy, 0.0],                                          # bottom left
                        [dx + self.marker_size, dy, 0.0],                      # bottom right
                    ],
                    dtype=float,
                )

                image_points_grid_list.append(image_points)
                object_points_grid_list.append(object_points)

        if not len(image_points_grid_list) or not len(object_points_grid_list):
            return detected_image, None, None

        image_points_grid = np.concatenate(image_points_grid_list, axis=0)
        object_points_grid = np.concatenate(object_points_grid_list, axis=0)

        intrinsics = self.camera_model.K
        distortion_coefficients = self.camera_model.D

        success, rvec, tvec = cv2.solvePnP(
            objectPoints=object_points_grid,
            imagePoints=image_points_grid,
            cameraMatrix=intrinsics,
            distCoeffs=distortion_coefficients,
            flags=cv2.SOLVEPNP_IPPE,
        )

        if success:
            detected_image = self.draw_axes(
                detected_image, rvec, tvec, intrinsics, distortion_coefficients
            )
            return detected_image, rvec.reshape((3,)), tvec.reshape((3,))
        else:
            return detected_image, None, None

    def color_image_callback(self, msg: Image):
        self.grayscale_image = self.cv_bridge.imgmsg_to_cv2(
            msg, desired_encoding="mono8"
        )

    def depth_image_callback(self, msg: Image):
        self.depth_image = self.cv_bridge.imgmsg_to_cv2(
            msg, desired_encoding="passthrough"
        )

    def camera_info_callback(self, msg: CameraInfo):
        self.camera_info = msg
        self.camera_model.fromCameraInfo(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Deprojection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
