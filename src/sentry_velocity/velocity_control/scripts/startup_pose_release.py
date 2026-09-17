#!/usr/bin/env python3

"""Release the Gazebo startup pose lock after RViz publishes /initialpose."""

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from std_msgs.msg import Bool


class StartupPoseRelease(Node):
    def __init__(self):
        super().__init__("startup_pose_release")
        self.publisher = self.create_publisher(Bool, "/simulation/robot_release", 10)
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self.release, 10
        )
        self.released = False
        self.release_requested = False
        self.status_subscription = self.create_subscription(
            Bool, "/simulation/robot_released", self.on_status, 10
        )
        self.retry_timer = self.create_timer(0.25, self.send_request)

    def release(self, _message):
        if self.released or self.release_requested:
            return
        self.release_requested = True
        self.get_logger().info("/initialpose received; requesting Gazebo unlock")
        self.send_request()

    def send_request(self):
        if not self.release_requested or self.released:
            return
        message = Bool()
        message.data = True
        self.publisher.publish(message)

    def on_status(self, message):
        if message.data and not self.released:
            self.released = True
            self.get_logger().info("Gazebo confirmed startup pose lock released")
        elif not message.data and self.released:
            self.released = False


def main(args=None):
    rclpy.init(args=args)
    node = StartupPoseRelease()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
