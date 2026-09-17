#!/usr/bin/env python3

"""Convert navigation's gimbal-frame planar velocity to the chassis frame."""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


class NavigationToChassis(Node):
    def __init__(self):
        super().__init__("navigation_to_chassis")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(Twist, "/navigation/cmd_vel", 10)
        self.subscription = self.create_subscription(
            Twist, "/navigation/cmd_vel_base_link", self.convert, 10
        )

    def convert(self, command):
        try:
            transform = self.tf_buffer.lookup_transform(
                "chassis", "base_link", rclpy.time.Time()
            )
        except TransformException as exception:
            self.publisher.publish(Twist())
            self.get_logger().warning(
                f"Cannot convert navigation velocity to chassis: {exception}",
                throttle_duration_sec=2.0,
            )
            return

        q = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        cosine, sine = math.cos(yaw), math.sin(yaw)
        output = Twist()
        output.linear.x = cosine * command.linear.x - sine * command.linear.y
        output.linear.y = sine * command.linear.x + cosine * command.linear.y
        output.linear.z = command.linear.z
        output.angular = command.angular
        self.publisher.publish(output)


def main():
    rclpy.init()
    node = NavigationToChassis()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
