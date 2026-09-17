#!/usr/bin/env python3

"""Algorithm-independent gate for sensor TF and optional static IMU readiness."""

import math
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformException, TransformListener


class SensorGate(Node):
    def __init__(self):
        super().__init__("wait_for_sensors")
        self.declare_parameter("lidar_frame", "livox_frame")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("imu_topic", "/livox/imu")
        self.declare_parameter("require_static_imu", True)
        self.declare_parameter("sample_count", 200)
        self.declare_parameter("stable_duration", 0.9)
        self.declare_parameter("acceleration_norm", 9.81)
        self.declare_parameter("acceleration_tolerance", 1.0)
        self.declare_parameter("max_angular_speed", 0.1)
        self.declare_parameter("max_acceleration_stddev", 0.15)
        self.require_static_imu = bool(self.get_parameter("require_static_imu").value)
        self.sample_count = int(self.get_parameter("sample_count").value)
        if self.sample_count < 2:
            raise ValueError("sample_count must be at least 2")
        self.stable_duration = float(self.get_parameter("stable_duration").value)
        self.acceleration_norm = float(self.get_parameter("acceleration_norm").value)
        self.acceleration_tolerance = float(self.get_parameter("acceleration_tolerance").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.max_acceleration_stddev = float(self.get_parameter("max_acceleration_stddev").value)
        if (self.stable_duration <= 0 or self.acceleration_norm <= 0 or
                self.acceleration_tolerance < 0 or self.max_angular_speed < 0 or
                self.max_acceleration_stddev < 0):
            raise ValueError("Invalid sensor readiness thresholds")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.imu_samples = deque(maxlen=self.sample_count)
        self.last_imu_wall_time = 0.0
        self.imu_subscription = self.create_subscription(
            Imu, str(self.get_parameter("imu_topic").value), self.on_imu, qos_profile_sensor_data
        )

    def on_imu(self, message):
        a, w = message.linear_acceleration, message.angular_velocity
        acceleration = (a.x, a.y, a.z)
        norm = math.sqrt(sum(v * v for v in acceleration))
        gyro = math.sqrt(w.x * w.x + w.y * w.y + w.z * w.z)
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if not (abs(norm - self.acceleration_norm) <= self.acceleration_tolerance
                and gyro <= self.max_angular_speed):
            self.imu_samples.clear()
            return
        if self.imu_samples and stamp <= self.imu_samples[-1][0]:
            self.imu_samples.clear()
        self.imu_samples.append((stamp, acceleration))
        self.last_imu_wall_time = time.monotonic()

    def imu_ready(self):
        if not self.require_static_imu:
            return True
        if len(self.imu_samples) < self.sample_count or time.monotonic() - self.last_imu_wall_time > 1.0:
            return False
        if self.imu_samples[-1][0] - self.imu_samples[0][0] < self.stable_duration:
            return False
        means = [sum(a[i] for _, a in self.imu_samples) / self.sample_count for i in range(3)]
        variance = sum(
            sum((a[i] - means[i]) ** 2 for i in range(3))
            for _, a in self.imu_samples
        ) / self.sample_count
        return variance <= self.max_acceleration_stddev ** 2


def main(args=None):
    rclpy.init(args=args)
    node = SensorGate()
    lidar_frame = str(node.get_parameter("lidar_frame").value)
    base_frame = str(node.get_parameter("base_frame").value)
    node.get_logger().info(
        f"Waiting for sensor TF {base_frame} -> {lidar_frame}; "
        f"static IMU required: {node.require_static_imu}"
    )
    ready = False
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            try:
                if node.imu_ready() and node.tf_buffer.can_transform(
                    lidar_frame, base_frame, rclpy.time.Time()
                ):
                    node.get_logger().info("Sensors ready")
                    ready = True
                    break
            except TransformException:
                pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if ready else 1


if __name__ == "__main__":
    raise SystemExit(main())
