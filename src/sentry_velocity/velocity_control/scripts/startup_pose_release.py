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

    def release(self, _message):
        if self.released:
            return
        message = Bool()
        message.data = True
        for _ in range(10):
            self.publisher.publish(message)
        self.released = True
        self.get_logger().info("/initialpose received; releasing Gazebo startup pose lock")


def main(args=None):
    rclpy.init(args=args)
    node = StartupPoseRelease()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
