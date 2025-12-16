#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import random

class ServoTwistPublisher(Node):
    def __init__(self):
        super().__init__("dt_simulator")

        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            "/left_forward_velocity_controller/commands",
            10,
        )

        # Base timer period (1 ms)
        self.base_period = 0.001

        self.timer = self.create_timer(
            self.base_period,
            self.timer_callback,
        )

    def timer_callback(self):
        msg = Float64MultiArray()
        msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.publisher_.publish(msg)

        # Optional: jitter the timer period safely
        jitter = random.randint(0, 9) * 0.001
        self.timer.timer_period_ns = int((self.base_period + jitter) * 1e9)


def main(args=None):
    rclpy.init(args=args)
    node = ServoTwistPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
