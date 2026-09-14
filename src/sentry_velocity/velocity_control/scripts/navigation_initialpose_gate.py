#!/usr/bin/env python3

"""Own map->odom during simulation and enable navigation after /initialpose."""

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


def quaternion_conjugate(q):
    return (-q[0], -q[1], -q[2], q[3])


def quaternion_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quaternion_normalize(q):
    norm = math.sqrt(sum(value * value for value in q))
    if norm == 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(value / norm for value in q)


def quaternion_rotate(q, vector):
    normalized = quaternion_normalize(q)
    rotated = quaternion_multiply(
        quaternion_multiply(normalized, (vector[0], vector[1], vector[2], 0.0)),
        quaternion_conjugate(normalized),
    )
    return rotated[:3]


def transform_inverse(transform):
    translation, rotation = transform
    inverse_rotation = quaternion_conjugate(quaternion_normalize(rotation))
    inverse_translation = quaternion_rotate(
        inverse_rotation, (-translation[0], -translation[1], -translation[2])
    )
    return inverse_translation, inverse_rotation


def transform_multiply(a, b):
    translation_a, rotation_a = a
    translation_b, rotation_b = b
    rotated_b = quaternion_rotate(rotation_a, translation_b)
    return (
        (
            translation_a[0] + rotated_b[0],
            translation_a[1] + rotated_b[1],
            translation_a[2] + rotated_b[2],
        ),
        quaternion_normalize(quaternion_multiply(rotation_a, rotation_b)),
    )


class NavigationInitialPoseGate(Node):
    def __init__(self):
        super().__init__("navigation_initialpose_gate")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("initialpose_topic", "/initialpose")
        self.declare_parameter("mode_topic", "/control_mode")
        self.declare_parameter("tf_publish_rate", 20.0)
        # The simulator's base_link is the elevated gimbal / LiDAR reference
        # frame.  Keep the 2-D initial pose semantics while lifting the RViz
        # robot model above the map plane by this amount.
        self.declare_parameter("base_height_offset", 0.7)

        self.map_frame = self.get_parameter("map_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        initialpose_topic = self.get_parameter("initialpose_topic").value
        mode_topic = self.get_parameter("mode_topic").value
        publish_rate = float(self.get_parameter("tf_publish_rate").value)
        self.base_height_offset = float(
            self.get_parameter("base_height_offset").value
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.mode_publisher = self.create_publisher(String, mode_topic, 10)
        self.initialpose_subscription = self.create_subscription(
            PoseWithCovarianceStamped, initialpose_topic, self.on_initialpose, 10
        )

        self.map_to_odom = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
        self.initialpose_received = False
        self.navigation_mode_messages = 0
        self.timer = self.create_timer(1.0 / publish_rate, self.on_timer)
        self.get_logger().info(
            f"Publishing provisional {self.map_frame}->{self.odom_frame}=identity; "
            f"waiting for {initialpose_topic} before enabling navigation"
        )

    def on_initialpose(self, message):
        pose = message.pose.pose
        map_to_base = (
            (
                pose.position.x,
                pose.position.y,
                pose.position.z + self.base_height_offset,
            ),
            (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
        )

        try:
            odom_to_base_message = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, rclpy.time.Time()
            )
            transform = odom_to_base_message.transform
            odom_to_base = (
                (transform.translation.x, transform.translation.y, transform.translation.z),
                (transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w),
            )
            self.map_to_odom = transform_multiply(
                map_to_base, transform_inverse(odom_to_base)
            )
        except TransformException as exception:
            self.get_logger().warning(
                f"Cannot look up {self.odom_frame}->{self.base_frame}; "
                f"using /initialpose directly as {self.map_frame}->{self.odom_frame}: {exception}"
            )
            self.map_to_odom = map_to_base

        # Repeat briefly so every simulation-side subscriber observes the switch.
        self.initialpose_received = True
        self.navigation_mode_messages = 20
        self.get_logger().info(
            "Initial pose received: updated map->odom and enabling navigation mode"
        )

    def publish_mode(self, mode):
        message = String()
        message.data = mode
        self.mode_publisher.publish(message)

    def on_timer(self):
        translation, rotation = self.map_to_odom
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.odom_frame
        transform.transform.translation.x = translation[0]
        transform.transform.translation.y = translation[1]
        transform.transform.translation.z = translation[2]
        transform.transform.rotation.x = rotation[0]
        transform.transform.rotation.y = rotation[1]
        transform.transform.rotation.z = rotation[2]
        transform.transform.rotation.w = rotation[3]
        self.tf_broadcaster.sendTransform(transform)

        # Keep asserting keyboard mode until an initial pose is actually
        # received.  This also overrides any stale navigation-mode publisher
        # left over from an earlier launch and keeps the gimbal command at zero.
        if not self.initialpose_received:
            self.publish_mode("keyboard")
        if self.navigation_mode_messages > 0:
            self.publish_mode("navigation")
            self.navigation_mode_messages -= 1


def main(args=None):
    rclpy.init(args=args)
    node = NavigationInitialPoseGate()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
