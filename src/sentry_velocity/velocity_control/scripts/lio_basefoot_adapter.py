#!/usr/bin/env python3

"""Convert isolated LIO odom->base_link TF to the stable footprint root.

The simulated robot description owns base_footprint->chassis->base_link.
Small Point-LIO still estimates odom->base_link, but its TF is remapped to a
private topic by lio_after_tf.py.  This node is the only publisher of the
global odom->base_footprint edge.
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener
from tf2_msgs.msg import TFMessage


def q_normalize(q):
    norm = math.sqrt(sum(value * value for value in q))
    if norm < 1.0e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(value / norm for value in q)


def q_conjugate(q):
    return (-q[0], -q[1], -q[2], q[3])


def q_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def q_rotate(q, vector):
    q = q_normalize(q)
    rotated = q_multiply(q_multiply(q, (vector[0], vector[1], vector[2], 0.0)), q_conjugate(q))
    return rotated[:3]


def transform_compose(first, second):
    """Compose (R,t) first * second."""
    q1, t1 = first
    q2, t2 = second
    rotated_t2 = q_rotate(q1, t2)
    return (
        q_normalize(q_multiply(q1, q2)),
        (t1[0] + rotated_t2[0], t1[1] + rotated_t2[1], t1[2] + rotated_t2[2]),
    )


def transform_inverse(transform):
    q, t = transform
    inverse_q = q_conjugate(q_normalize(q))
    inverse_t = q_rotate(inverse_q, (-t[0], -t[1], -t[2]))
    return inverse_q, inverse_t


def yaw_from_q(q):
    x, y, z, w = q_normalize(q)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class LioBasefootAdapter(Node):
    def __init__(self):
        super().__init__("lio_basefoot_adapter")
        self.declare_parameter("lio_tf_topic", "/navigationros2/lio_tf")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("footprint_frame", "base_footprint")

        self.lio_tf_topic = str(self.get_parameter("lio_tf_topic").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.footprint_frame = str(self.get_parameter("footprint_frame").value)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            TFMessage, self.lio_tf_topic, self.on_lio_tf, 20
        )
        self.last_warning = self.get_clock().now()
        self.get_logger().info(
            f"Converting {self.lio_tf_topic}: {self.odom_frame}->{self.base_frame} "
            f"to global {self.odom_frame}->{self.footprint_frame}"
        )

    def on_lio_tf(self, message):
        for transform in message.transforms:
            if transform.header.frame_id.lstrip("/") != self.odom_frame:
                continue
            if transform.child_frame_id.lstrip("/") != self.base_frame:
                continue
            try:
                # lookup_transform(target, source) returns T_target_source.
                base_in_footprint = self.buffer.lookup_transform(
                    self.footprint_frame,
                    self.base_frame,
                    rclpy.time.Time.from_msg(transform.header.stamp),
                )
            except (TransformException, ValueError) as exception:
                # A freshly published joint state may not yet have a sample at
                # the exact LIO timestamp.  Latest is safe for this fixed/joint
                # kinematic edge and avoids dropping the complete odometry.
                try:
                    base_in_footprint = self.buffer.lookup_transform(
                        self.footprint_frame, self.base_frame, rclpy.time.Time()
                    )
                except TransformException:
                    now = self.get_clock().now()
                    if (now - self.last_warning).nanoseconds > 2_000_000_000:
                        self.get_logger().warning(
                            f"Waiting for {self.footprint_frame}->{self.base_frame}: {exception}"
                        )
                        self.last_warning = now
                    continue

            lio_q = (
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            )
            lio_t = (
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            )
            edge_q = (
                base_in_footprint.transform.rotation.x,
                base_in_footprint.transform.rotation.y,
                base_in_footprint.transform.rotation.z,
                base_in_footprint.transform.rotation.w,
            )
            edge_t = (
                base_in_footprint.transform.translation.x,
                base_in_footprint.transform.translation.y,
                base_in_footprint.transform.translation.z,
            )
            # T_odom_footprint = T_odom_base * inverse(T_footprint_base).
            footprint_q, footprint_t = transform_compose(
                (lio_q, lio_t), transform_inverse((edge_q, edge_t))
            )

            # The map/odom/navigation layer is planar.  Keep LIO's x/y/yaw,
            # but prevent sensor tilt or numerical height from moving the
            # ground reference and the RViz model through the map.
            yaw = yaw_from_q(footprint_q)
            output = TransformStamped()
            output.header.stamp = transform.header.stamp
            output.header.frame_id = self.odom_frame
            output.child_frame_id = self.footprint_frame
            output.transform.translation.x = footprint_t[0]
            output.transform.translation.y = footprint_t[1]
            output.transform.translation.z = 0.0
            output.transform.rotation.z = math.sin(yaw / 2.0)
            output.transform.rotation.w = math.cos(yaw / 2.0)
            self.broadcaster.sendTransform(output)


def main(args=None):
    rclpy.init(args=args)
    node = LioBasefootAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
