#!/usr/bin/env python3
import argparse
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def main():
    parser = argparse.ArgumentParser(
        description="Subscribe to a ROS2 Image topic and save frames as an MP4 video."
    )
    parser.add_argument(
        "--topic", type=str, default="/camera/image_raw",
        help="ROS2 topic name of type sensor_msgs/msg/Image (default: /camera/image_raw)"
    )
    parser.add_argument(
        "--output", type=str, default="output.mp4",
        help="Output MP4 filename (default: output.mp4)"
    )
    parser.add_argument(
        "--fps", type=float, default=30.0,
        help="Frames per second of the output video (default: 30)"
    )
    parser.add_argument(
        "--codec", type=str, default="mp4v",
        help="Video codec FourCC (e.g., mp4v, H264, avc1)"
    )
    args = parser.parse_args()

    # --- Initialize ROS2 and bridge ---
    rclpy.init()
    node = rclpy.create_node("image_to_video_standalone")
    bridge = CvBridge()

    writer = None
    frame_count = 0
    first_frame_received = False
    output_path = str(Path(args.output).with_suffix(".mp4"))

    def callback(msg: Image):
        nonlocal writer, frame_count, first_frame_received
        try:
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            node.get_logger().warn(f"Failed to convert image: {e}")
            return

        # Initialize writer when first frame arrives
        if not first_frame_received:
            h, w = cv_img.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*args.codec)
            writer = cv2.VideoWriter(output_path, fourcc, args.fps, (w, h))
            if not writer.isOpened():
                node.get_logger().error("Failed to open video writer.")
                rclpy.shutdown()
                return
            first_frame_received = True
            node.get_logger().info(f"Started recording to '{output_path}'.")

        writer.write(cv_img)
        frame_count += 1

    # --- Subscribe and wait ---
    node.create_subscription(Image, args.topic, callback, qos_profile_sensor_data)
    node.get_logger().info(f"Waiting for images on topic '{args.topic}'...")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted. Saving video...")
    finally:
        if writer is not None:
            writer.release()
            node.get_logger().info(f"Saved {frame_count} frames to {output_path}")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

