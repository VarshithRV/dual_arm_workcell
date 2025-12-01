#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import sys
from pathlib import Path

class DumpURDF(Node):
    def __init__(self, output_file):
        super().__init__("dump_robot_description")
        self.output_file = output_file

        # Allow robot_state_publisher to start publishing parameters
        time.sleep(1.0)

        if not self.has_parameter("robot_description"):
            self.declare_parameter("robot_description", "")

        # Fetch param
        desc = self.get_parameter("robot_description").get_parameter_value().string_value

        if not desc:
            self.get_logger().error("robot_description parameter is empty.")
        else:
            Path(self.output_file).write_text(desc)
            self.get_logger().info(f"Saved robot_description to {self.output_file}")

        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    output_file = sys.argv[1] if len(sys.argv) > 1 else "robot_dump.urdf"
    node = DumpURDF(output_file)
    rclpy.spin(node)

if __name__ == "__main__":
    main()
