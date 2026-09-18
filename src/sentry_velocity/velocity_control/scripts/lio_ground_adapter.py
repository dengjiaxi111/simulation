#!/usr/bin/env python3
"""Translate isolated LIO outputs to a fixed odom origin at startup ground."""
import copy
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool
from navigation_initialpose_gate import quaternion_rotate


def translate_cloud(message, shift):
    output = copy.deepcopy(message)
    payload = bytearray(message.data)
    endian = ">" if message.is_bigendian else "<"
    fields = {field.name: field for field in message.fields}
    for axis, delta in zip(("x", "y", "z"), shift):
        field = fields[axis]
        if field.datatype not in (PointField.FLOAT32, PointField.FLOAT64) or field.count != 1:
            raise ValueError("PointCloud2 XYZ must be scalar floating-point fields")
        dtype = endian + ("f4" if field.datatype == PointField.FLOAT32 else "f8")
        values = np.ndarray((message.height, message.width), dtype=dtype,
                            buffer=payload, offset=field.offset,
                            strides=(message.row_step, message.point_step))
        values += delta
    output.data = bytes(payload)
    output.header.frame_id = "odom"
    return output


class GroundAdapter(Node):
    def __init__(self):
        super().__init__("lio_ground_adapter")
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.shift = None
        self.ready = False
        self.ready_sub = self.create_subscription(
            Bool, '/lio/extrinsics_ready', self.on_ready,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.odom_pub = self.create_publisher(Odometry, "/Odometry", 10)
        self.cloud_pub = self.create_publisher(PointCloud2, "/cloud_registered", 10)
        self.subscriptions_owned = [
            self.create_subscription(TFMessage, "/lio/raw_tf", self.on_tf, 100),
            self.create_subscription(Odometry, "/lio/raw_odometry", self.on_odom, 10),
            self.create_subscription(PointCloud2, "/lio/raw_cloud_registered", self.on_cloud, 10),
        ]

    def on_ready(self, message):
        self.ready = message.data
        if not self.ready:
            self.shift = None

    def initialize(self, position, orientation):
        if not self.ready:
            return False
        if self.shift is not None:
            return True
        try:
            ground = self.buffer.lookup_transform("base_link", "base_links",
                                                  rclpy.time.Time()).transform.translation
        except TransformException:
            return False
        offset = quaternion_rotate((orientation.x, orientation.y, orientation.z,
                                    orientation.w), (ground.x, ground.y, ground.z))
        self.shift = tuple(-p - d for p, d in zip(
            (position.x, position.y, position.z), offset))
        self.get_logger().info(f"Fixed ground odom initialized; translation={self.shift}")
        return True

    def translate_position(self, position):
        position.x += self.shift[0]
        position.y += self.shift[1]
        position.z += self.shift[2]

    def on_tf(self, message):
        if not self.ready:
            return
        for transform in message.transforms:
            if transform.header.frame_id == "odom" and transform.child_frame_id == "base_link":
                self.initialize(transform.transform.translation, transform.transform.rotation)
        if self.shift is None:
            return
        outputs = []
        for transform in message.transforms:
            output = copy.deepcopy(transform)
            if output.header.frame_id == "odom":
                self.translate_position(output.transform.translation)
            outputs.append(output)
        if outputs:
            self.broadcaster.sendTransform(outputs)

    def on_odom(self, message):
        if message.header.frame_id != "odom" or message.child_frame_id != "base_link":
            return
        if not self.initialize(message.pose.pose.position, message.pose.pose.orientation):
            return
        output = copy.deepcopy(message)
        self.translate_position(output.pose.pose.position)
        # The fixed change of origin has no rotation. Covariances, orientation,
        # and child-frame velocity therefore retain their original values.
        self.odom_pub.publish(output)

    def on_cloud(self, message):
        if self.shift is None or message.header.frame_id != "odom":
            return
        try:
            output = translate_cloud(message, self.shift)
        except (KeyError, ValueError) as error:
            self.get_logger().error(f"Cannot translate point cloud: {error}",
                                    throttle_duration_sec=5.0)
            return
        self.cloud_pub.publish(output)


def main():
    rclpy.init()
    node = GroundAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
