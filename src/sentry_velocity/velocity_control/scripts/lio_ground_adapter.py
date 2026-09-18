#!/usr/bin/env python3
"""Transform isolated LIO outputs to a fixed odom origin at startup ground."""
import copy
import math
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool
from navigation_initialpose_gate import quaternion_rotate, quaternion_multiply


def translate_cloud(message, shift, rotation):
    output = copy.deepcopy(message)
    payload = bytearray(message.data)
    endian = ">" if message.is_bigendian else "<"
    fields = {field.name: field for field in message.fields}
    arrays = []
    for axis in ("x", "y", "z"):
        field = fields[axis]
        if field.datatype not in (PointField.FLOAT32, PointField.FLOAT64) or field.count != 1:
            raise ValueError("PointCloud2 XYZ must be scalar floating-point fields")
        dtype = endian + ("f4" if field.datatype == PointField.FLOAT32 else "f8")
        values = np.ndarray((message.height, message.width), dtype=dtype,
                            buffer=payload, offset=field.offset,
                            strides=(message.row_step, message.point_step))
        arrays.append(values)
    points = np.stack(arrays, axis=-1) @ rotation.T + np.asarray(shift)
    for index, values in enumerate(arrays):
        values[:] = points[..., index]
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
        self.rotation = np.eye(3)
        self.rotation_q = (0.0, 0.0, 0.0, 1.0)
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
        x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
        yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        c, sn = math.cos(yaw), math.sin(yaw)
        self.rotation = np.array([[c, sn, 0.0], [-sn, c, 0.0], [0.0, 0.0, 1.0]])
        self.rotation_q = (0.0, 0.0, math.sin(-yaw/2), math.cos(-yaw/2))
        origin = np.asarray((position.x, position.y, position.z)) + offset
        self.shift = tuple(-(self.rotation @ origin))
        self.get_logger().info(f"Fixed ground odom initialized; translation={self.shift}")
        return True

    def translate_position(self, position):
        values = self.rotation @ np.asarray((position.x, position.y, position.z)) + self.shift
        position.x, position.y, position.z = map(float, values)

    def rotate_orientation(self, orientation):
        q = quaternion_multiply(self.rotation_q, (orientation.x, orientation.y, orientation.z, orientation.w))
        orientation.x, orientation.y, orientation.z, orientation.w = q

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
                self.rotate_orientation(output.transform.rotation)
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
        self.rotate_orientation(output.pose.pose.orientation)
        basis = np.zeros((6, 6))
        basis[:3, :3] = self.rotation
        basis[3:, 3:] = self.rotation
        covariance = np.asarray(message.pose.covariance).reshape(6, 6)
        output.pose.covariance = (basis @ covariance @ basis.T).ravel().tolist()
        # Child-frame twist is unchanged.
        self.odom_pub.publish(output)

    def on_cloud(self, message):
        if self.shift is None or message.header.frame_id != "odom":
            return
        try:
            output = translate_cloud(message, self.shift, self.rotation)
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
