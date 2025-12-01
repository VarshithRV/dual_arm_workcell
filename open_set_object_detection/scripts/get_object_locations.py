#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseStamped, Pose
from typing import List

import cv_bridge
import cv2
import image_geometry
import numpy as np
from groundingdino.util.inference import load_model, load_image, predict
from open_set_object_detection_msgs.msg import ObjectPosition, ObjectPositions
from open_set_object_detection_msgs.srv import GetObjectLocations
import supervision as sv
import torch
from torchvision.ops import box_convert
import tf2_ros
import tf2_geometry_msgs

from rclpy.time import Time
from rclpy.duration import Duration


# Model parameters
BOX_THRESHOLD = 0.35
TEXT_THRESHOLD = 0.25
TEXT_PROMPT = "blue_ball.yellow_ball.pink_ball"

# Camera topic base namespaces (ROS 2)
LEFT_CAMERA_NS = "/camera/left_camera"
RIGHT_CAMERA_NS = "/camera/right_camera"

# Local crop region (in full RGB image coordinates)
CROP_X_MIN = 280
CROP_Y_MIN = 174
CROP_X_MAX = 1025
CROP_Y_MAX = 569


class Deprojection(Node):
    def __init__(self) -> None:
        super().__init__("deprojection_node")

        # Initialize camera-related variables
        self.left_depth_image = None
        self.left_camera_info = None
        self.left_color_image = None
        self.right_depth_image = None
        self.right_camera_info = None
        self.right_color_image = None

        self.left_camera_model = image_geometry.PinholeCameraModel()
        self.right_camera_model = image_geometry.PinholeCameraModel()

        self.cv_bridge = cv_bridge.CvBridge()
        self.latest_result = None
        self.recursion = 0
        self.source_image_path = "assets/generated_image.jpeg"

        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Declare parameters (can be overridden via YAML / CLI)
        self.declare_parameter("left_depth_ws_threshold", 1000)
        self.declare_parameter("right_depth_ws_threshold", 1000)

        self.left_depth_threshold = (
            self.get_parameter("left_depth_ws_threshold").value
        )
        self.right_depth_threshold = (
            self.get_parameter("right_depth_ws_threshold").value
        )

        # Load the GroundingDINO model
        self.model = load_model(
            "groundingdino/config/GroundingDINO_SwinT_OGC.py",
            "weights/groundingdino_swint_ogc.pth",
        )
        self.get_logger().info("Loaded the Grounding Dino model")

        # Camera topics (ROS 2)
        left_camera_color_topic = f"{LEFT_CAMERA_NS}/color/image_raw"
        left_camera_info_topic = f"{LEFT_CAMERA_NS}/aligned_depth_to_color/camera_info"
        left_camera_depth_topic = f"{LEFT_CAMERA_NS}/aligned_depth_to_color/image_raw"

        right_camera_color_topic = f"{RIGHT_CAMERA_NS}/color/image_raw"
        right_camera_info_topic = f"{RIGHT_CAMERA_NS}/aligned_depth_to_color/camera_info"
        right_camera_depth_topic = f"{RIGHT_CAMERA_NS}/aligned_depth_to_color/image_raw"

        # Subscribers
        self.left_depth_image_sub = self.create_subscription(
            Image,
            left_camera_depth_topic,
            self.left_depth_image_callback,
            10,
        )
        self.left_camera_info_sub = self.create_subscription(
            CameraInfo,
            left_camera_info_topic,
            self.left_camera_info_callback,
            10,
        )
        self.left_color_image_sub = self.create_subscription(
            Image,
            left_camera_color_topic,
            self.left_color_image_callback,
            10,
        )
        self.right_depth_image_sub = self.create_subscription(
            Image,
            right_camera_depth_topic,
            self.right_depth_image_callback,
            10,
        )
        self.right_camera_info_sub = self.create_subscription(
            CameraInfo,
            right_camera_info_topic,
            self.right_camera_info_callback,
            10,
        )
        self.right_color_image_sub = self.create_subscription(
            Image,
            right_camera_color_topic,
            self.right_color_image_callback,
            10,
        )

        # Services
        self.left_service = self.create_service(
            GetObjectLocations,
            "left_get_object_locations",
            self.left_get_object_locations,
        )
        self.right_service = self.create_service(
            GetObjectLocations,
            "right_get_object_locations",
            self.right_get_object_locations,
        )

        # Publisher for streamed position
        self.stream_pub = self.create_publisher(
            PoseStamped, "/orange_position", 10
        )

        # Timer for streaming
        self.timer = self.create_timer(0.5, self.publish_stream)

    def __del__(self):
        # This is largely unnecessary in ROS 2 Python but kept for parity
        if hasattr(self, "model"):
            del self.model

    # === Annotation and TF utilities ===

    def annotate(
        self,
        image_source: np.ndarray,
        boxes: torch.Tensor,
        logits: torch.Tensor,
        phrases: List[str],
    ) -> np.ndarray:
        h, w, _ = image_source.shape
        boxes = boxes * torch.Tensor([w, h, w, h])
        xyxy = box_convert(
            boxes=boxes, in_fmt="cxcywh", out_fmt="xyxy"
        ).numpy()
        detections = sv.Detections(xyxy=xyxy)
        labels = [
            f"{i}: {phrase} {logit:.2f}"
            for i, (phrase, logit) in enumerate(zip(phrases, logits))
        ]
        bbox_annotator = sv.BoxAnnotator(color_lookup=sv.ColorLookup.INDEX)
        label_annotator = sv.LabelAnnotator(color_lookup=sv.ColorLookup.INDEX)
        annotated_frame = cv2.cvtColor(image_source, cv2.COLOR_RGB2BGR)
        annotated_frame = bbox_annotator.annotate(
            scene=annotated_frame, detections=detections
        )
        annotated_frame = label_annotator.annotate(
            scene=annotated_frame, detections=detections, labels=labels
        )
        return annotated_frame

    def transform_pose(self, pose: PoseStamped, target_frame: str):
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                pose.header.frame_id,
                Time(),
                timeout=Duration(seconds=3.0),
            )

            # do_transform_pose in your TF2 version expects a Pose, not PoseStamped
            transformed_pose = tf2_geometry_msgs.do_transform_pose(
                pose.pose, transform
            )  # -> Pose

            pose_stamped_out = PoseStamped()
            pose_stamped_out.header.frame_id = target_frame
            pose_stamped_out.header.stamp = self.get_clock().now().to_msg()
            pose_stamped_out.pose = transformed_pose
            return pose_stamped_out

        except Exception as e:
            self.get_logger().error(f"Transform error: {e}")
            return None

    # === Timer callback ===

    def publish_stream(self):
        if self.latest_result is None:
            return

        if not self.latest_result.object_position:
            return

        position = PoseStamped()
        position.header.frame_id = "world"
        position.header.stamp = self.get_clock().now().to_msg()
        position.pose = self.latest_result.object_position[0].pose.pose
        self.stream_pub.publish(position)

    # === Core 3D position computation ===

    def get_3d_position(
        self,
        x,
        y,
        depth_image,
        camera_info,
        camera_model,
        depth_threshold,
    ):
        if depth_image is None or camera_info is None:
            return None

        # Safety on indices
        h, w = depth_image.shape[:2]
        if x < 0 or x >= w or y < 0 or y >= h:
            self.get_logger().warn(
                f"Pixel ({x}, {y}) outside depth image bounds."
            )
            return None

        depth = depth_image[y, x] / 1000.0  # Convert to meters

        if (
            np.isnan(depth)
            or depth == 0
            or depth > depth_threshold / 1000.0
        ):
            self.get_logger().warn(
                f"Invalid depth at pixel ({x}, {y}) or above threshold."
            )
            self.recursion += 1
            if self.recursion <= 10:
                return self.get_3d_position(
                    x,
                    y,
                    depth_image=depth_image,
                    camera_info=camera_info,
                    camera_model=camera_model,
                    depth_threshold=depth_threshold,
                )
            else:
                self.recursion = 0
                return None

        point_3d = np.array(
            camera_model.projectPixelTo3dRay((x, y))
        ) * depth

        pose = PoseStamped()
        pose.header.frame_id = camera_model.tf_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(point_3d[0])
        pose.pose.position.y = float(point_3d[1])
        pose.pose.position.z = float(point_3d[2])
        pose.pose.orientation.w = 1.0

        return self.transform_pose(pose, "world")

    # === Service callbacks (with optional local crop) ===

    def left_get_object_locations(self, request, response):
        # Only require that we have a color image
        if self.left_color_image is None:
            self.get_logger().warn(
                "[LEFT] No color image yet, returning empty response"
            )
            response.result = ObjectPositions()
            return response

        # Decide whether to crop or use full image
        color_image_bgr_full = self.left_color_image.copy()
        h_full, w_full, _ = color_image_bgr_full.shape

        if getattr(request, "is_local", False):
            # Clamp crop to image bounds
            x_min = max(0, min(CROP_X_MIN, w_full - 1))
            x_max = max(x_min + 1, min(CROP_X_MAX, w_full))
            y_min = max(0, min(CROP_Y_MIN, h_full - 1))
            y_max = max(y_min + 1, min(CROP_Y_MAX, h_full))

            color_image_bgr = color_image_bgr_full[y_min:y_max, x_min:x_max]
            x_offset = x_min
            y_offset = y_min
            self.get_logger().info(
                f"[LEFT] Using local crop: x[{x_min},{x_max}), y[{y_min},{y_max})"
            )
        else:
            color_image_bgr = color_image_bgr_full
            x_offset = 0
            y_offset = 0
            self.get_logger().info("[LEFT] Using full image for detection")

        # Convert to RGB and run detector
        color_image_rgb = cv2.cvtColor(color_image_bgr, cv2.COLOR_BGR2RGB)
        cv2.imwrite(self.source_image_path, color_image_rgb)

        image_source, image = load_image(self.source_image_path)
        boxes, logits, phrases = predict(
            model=self.model,
            image=image,
            caption=request.prompt.data,
            box_threshold=BOX_THRESHOLD,
            text_threshold=TEXT_THRESHOLD,
        )

        self.get_logger().info(f"[LEFT] detections: {len(phrases)}")

        annotated_frame = self.annotate(
            image_source=image_source,
            boxes=boxes,
            logits=logits,
            phrases=phrases,
        )
        cv2.imwrite(
            "inference_images/annotated_image_left.jpg", annotated_frame
        )

        result = ObjectPositions()
        h, w, _ = image_source.shape
        boxes = boxes * torch.Tensor([w, h, w, h])
        xyxy = (
            box_convert(boxes=boxes, in_fmt="cxcywh", out_fmt="xyxy")
            .numpy()
            .astype(int)
        )

        # Inpaint invalid depth points (full depth image, not cropped)
        left_depth_image = self.left_depth_image
        if left_depth_image is not None:
            mask = (left_depth_image == 0).astype(np.uint8)
            left_depth_image = cv2.inpaint(
                left_depth_image,
                mask,
                inpaintRadius=3,
                flags=cv2.INPAINT_NS,
            )

        for i in range(len(phrases)):
            object_position = ObjectPosition()
            object_position.id = i
            object_position.label = phrases[i]

            # Center of the box in CROPPED coordinates
            x_center = int((xyxy[i][0] + xyxy[i][2]) / 2)
            y_center = int((xyxy[i][1] + xyxy[i][3]) / 2)

            # Map to FULL image coordinates for depth lookup
            x_center_full = x_center + x_offset
            y_center_full = y_center + y_offset

            pose = self.get_3d_position(
                x_center_full,
                y_center_full,
                depth_image=left_depth_image,
                camera_info=self.left_camera_info,
                camera_model=self.left_camera_model,
                depth_threshold=self.left_depth_threshold,
            )

            if pose is None:
                self.get_logger().warn(
                    f"[LEFT] Pose None for object {i}, skipping"
                )
                continue

            object_position.pose = pose

            # Store bbox in FULL image coordinates
            object_position.x_min = int(xyxy[i][0] + x_offset)
            object_position.y_min = int(xyxy[i][1] + y_offset)
            object_position.x_max = int(xyxy[i][2] + x_offset)
            object_position.y_max = int(xyxy[i][3] + y_offset)

            result.object_position.append(object_position)

        # NOTE: result.image is the annotated *crop* when is_local=True.
        result.image = self.cv_bridge.cv2_to_imgmsg(
            annotated_frame, encoding="bgr8"
        )
        self.latest_result = result
        response.result = result
        return response

    def right_get_object_locations(self, request, response):
        if self.right_color_image is None:
            self.get_logger().warn(
                "[RIGHT] No color image yet, returning empty response"
            )
            response.result = ObjectPositions()
            return response

        # Decide whether to crop or use full image
        color_image_bgr_full = self.right_color_image.copy()
        h_full, w_full, _ = color_image_bgr_full.shape

        if getattr(request, "is_local", False):
            # Clamp crop to image bounds
            x_min = max(0, min(CROP_X_MIN, w_full - 1))
            x_max = max(x_min + 1, min(CROP_X_MAX, w_full))
            y_min = max(0, min(CROP_Y_MIN, h_full - 1))
            y_max = max(y_min + 1, min(CROP_Y_MAX, h_full))

            color_image_bgr = color_image_bgr_full[y_min:y_max, x_min:x_max]
            x_offset = x_min
            y_offset = y_min
            self.get_logger().info(
                f"[RIGHT] Using local crop: x[{x_min},{x_max}), y[{y_min},{y_max})"
            )
        else:
            color_image_bgr = color_image_bgr_full
            x_offset = 0
            y_offset = 0
            self.get_logger().info("[RIGHT] Using full image for detection")

        # Convert to RGB and run detector
        color_image_rgb = cv2.cvtColor(color_image_bgr, cv2.COLOR_BGR2RGB)
        cv2.imwrite(self.source_image_path, color_image_rgb)

        image_source, image = load_image(self.source_image_path)
        boxes, logits, phrases = predict(
            model=self.model,
            image=image,
            caption=request.prompt.data,
            box_threshold=BOX_THRESHOLD,
            text_threshold=TEXT_THRESHOLD,
        )

        self.get_logger().info(f"[RIGHT] detections: {len(phrases)}")

        annotated_frame = self.annotate(
            image_source=image_source,
            boxes=boxes,
            logits=logits,
            phrases=phrases,
        )
        cv2.imwrite(
            "inference_images/annotated_image_right.jpg", annotated_frame
        )

        result = ObjectPositions()
        h, w, _ = image_source.shape
        boxes = boxes * torch.Tensor([w, h, w, h])
        xyxy = (
            box_convert(boxes=boxes, in_fmt="cxcywh", out_fmt="xyxy")
            .numpy()
            .astype(int)
        )

        right_depth_image = self.right_depth_image
        if right_depth_image is not None:
            mask = (right_depth_image == 0).astype(np.uint8)
            right_depth_image = cv2.inpaint(
                right_depth_image,
                mask,
                inpaintRadius=3,
                flags=cv2.INPAINT_NS,
            )

        for i in range(len(phrases)):
            object_position = ObjectPosition()
            object_position.id = i
            object_position.label = phrases[i]

            # Center of the box in CROPPED coordinates
            x_center = int((xyxy[i][0] + xyxy[i][2]) / 2)
            y_center = int((xyxy[i][1] + xyxy[i][3]) / 2)

            # Map to FULL image coordinates for depth lookup
            x_center_full = x_center + x_offset
            y_center_full = y_center + y_offset

            pose = self.get_3d_position(
                x_center_full,
                y_center_full,
                depth_image=right_depth_image,
                camera_info=self.right_camera_info,
                camera_model=self.right_camera_model,
                depth_threshold=self.right_depth_threshold,
            )

            if pose is None:
                self.get_logger().warn(
                    f"[RIGHT] Pose None for object {i}, skipping"
                )
                continue

            object_position.pose = pose

            # Store bbox in FULL image coordinates
            object_position.x_min = int(xyxy[i][0] + x_offset)
            object_position.y_min = int(xyxy[i][1] + y_offset)
            object_position.x_max = int(xyxy[i][2] + x_offset)
            object_position.y_max = int(xyxy[i][3] + y_offset)

            result.object_position.append(object_position)

        result.image = self.cv_bridge.cv2_to_imgmsg(
            annotated_frame, encoding="bgr8"
        )
        self.latest_result = result
        response.result = result
        return response

    # === Image and camera info callbacks ===

    def left_color_image_callback(self, msg: Image):
        self.left_color_image = self.cv_bridge.imgmsg_to_cv2(
            msg, desired_encoding="passthrough"
        )

    def left_depth_image_callback(self, msg: Image):
        self.left_depth_image = self.cv_bridge.imgmsg_to_cv2(
            msg, desired_encoding="passthrough"
        )

    def left_camera_info_callback(self, msg: CameraInfo):
        self.left_camera_info = msg
        self.left_camera_model.fromCameraInfo(msg)

    def right_color_image_callback(self, msg: Image):
        self.right_color_image = self.cv_bridge.imgmsg_to_cv2(
            msg, desired_encoding="passthrough"
        )

    def right_depth_image_callback(self, msg: Image):
        self.right_depth_image = self.cv_bridge.imgmsg_to_cv2(
            msg, desired_encoding="passthrough"
        )

    def right_camera_info_callback(self, msg: CameraInfo):
        self.right_camera_info = msg
        self.right_camera_model.fromCameraInfo(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Deprojection()
    node.get_logger().info("Deprojection server ready...")
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
