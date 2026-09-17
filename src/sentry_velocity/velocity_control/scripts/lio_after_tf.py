#!/usr/bin/env python3

"""Start Small Point-LIO only after the simulator's sensor TF is available."""

import os
import math
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformException, TransformListener


class TfWaiter(Node):
    def __init__(self):
        super().__init__("lio_after_tf")
        self.declare_parameter("lio_params_file", "")
        self.declare_parameter("lidar_frame", "livox_frame")
        self.declare_parameter("base_frame", "base_link")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.imu_samples = deque(maxlen=200)
        self.last_imu_wall_time = 0.0
        self.imu_subscription = self.create_subscription(
            Imu, "/livox/imu", self.on_imu, qos_profile_sensor_data
        )

    def on_imu(self, message):
        a, w = message.linear_acceleration, message.angular_velocity
        acceleration = (a.x, a.y, a.z)
        norm = math.sqrt(sum(v * v for v in acceleration))
        gyro = math.sqrt(w.x * w.x + w.y * w.y + w.z * w.z)
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if not (8.8 <= norm <= 10.8 and gyro <= 0.1):
            self.imu_samples.clear()
            return
        if self.imu_samples and stamp <= self.imu_samples[-1][0]:
            self.imu_samples.clear()
        self.imu_samples.append((stamp, acceleration))
        self.last_imu_wall_time = time.monotonic()

    def imu_ready(self):
        if len(self.imu_samples) < 200 or time.monotonic() - self.last_imu_wall_time > 1.0:
            return False
        if self.imu_samples[-1][0] - self.imu_samples[0][0] < 0.9:
            return False
        means = [sum(a[i] for _, a in self.imu_samples) / 200 for i in range(3)]
        variance = sum(
            sum((a[i] - means[i]) ** 2 for i in range(3))
            for _, a in self.imu_samples
        ) / 200
        return variance <= 0.15 ** 2


def main(args=None):
    rclpy.init(args=args)
    node = TfWaiter()
    params_file = str(node.get_parameter("lio_params_file").value)
    lidar_frame = str(node.get_parameter("lidar_frame").value)
    base_frame = str(node.get_parameter("base_frame").value)
    if params_file and not os.path.isfile(params_file):
        node.get_logger().error(f"LIO parameter file does not exist: {params_file}")
        node.destroy_node()
        rclpy.shutdown()
        raise FileNotFoundError(params_file)
    node.get_logger().info(
        f"Waiting for TF {base_frame} -> {lidar_frame} and stable, gravity-bearing "
        "IMU samples before starting small_point_lio"
    )
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)
        try:
            if node.imu_ready() and node.tf_buffer.can_transform(
                lidar_frame, base_frame, rclpy.time.Time()
            ):
                break
        except TransformException:
            pass

    if not rclpy.ok():
        node.destroy_node()
        return
    node.get_logger().info("Sensor TF is ready; starting small_point_lio")
    node.destroy_node()
    rclpy.shutdown()
    command = ["ros2", "run", "small_point_lio", "small_point_lio_node", "--ros-args"]
    if params_file:
        command.extend(["--params-file", params_file])
    # Keep simulation-only overrides after the YAML so mid360_sim.yaml cannot
    # change the already-validated livox_frame back to its lidar_link default.
    command.extend([
        "-p", "use_sim_time:=true",
        "-p", f"lidar_frame:={lidar_frame}",
        "-r", "__node:=small_point_lio",
    ])
    os.execvp(command[0], command)


if __name__ == "__main__":
    main()
