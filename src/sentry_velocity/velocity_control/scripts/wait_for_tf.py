#!/usr/bin/env python3

"""Exit only after a required TF edge is available."""

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


class TfGate(Node):
    def __init__(self):
        super().__init__("wait_for_tf")
        self.declare_parameter("target_frame", "odom")
        self.declare_parameter("source_frame", "base_link_fake")
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.source_frame = str(self.get_parameter("source_frame").value)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)


def main(args=None):
    rclpy.init(args=args)
    node = TfGate()
    node.get_logger().info(
        f"Waiting for TF {node.target_frame} <- {node.source_frame} "
        "before activating navigation lifecycle"
    )
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            try:
                if node.buffer.can_transform(
                    node.target_frame,
                    node.source_frame,
                    rclpy.time.Time(),
                ):
                    node.get_logger().info(
                        f"TF {node.target_frame} <- {node.source_frame} is ready"
                    )
                    return
            except TransformException:
                pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
