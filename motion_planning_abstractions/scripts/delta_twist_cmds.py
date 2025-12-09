#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped


class ServoTwistPublisher(Node):
    def __init__(self):
        super().__init__("servo_twist_publisher")

        self.pub = self.create_publisher(
            TwistStamped,
            "/left_servo_node_main/delta_twist_cmds",
            10
        )

        # publish at 50 Hz (MoveIt Servo typical rate)
        self.timer = self.create_timer(0.02, self.timer_callback)

    def timer_callback(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"     # change if needed

        # ----- example twist command -----
        # linear velocity
        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.01

        # angular velocity
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0
        # ----------------------------------

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ServoTwistPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
