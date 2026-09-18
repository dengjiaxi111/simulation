#!/usr/bin/env python3
"""Reject Small Point-LIO's identity-extrinsic fallback without editing it."""
import os
import signal
import subprocess
import sys
import time
import threading
import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener, TransformException


def main():
    rclpy.init(args=[])
    node = rclpy.create_node('lio_supervisor')
    publisher = node.create_publisher(Bool, '/lio/extrinsics_ready',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    extrinsics = node.create_publisher(TFMessage, '/lio/extrinsic_tf_static',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def repeat_extrinsic():
        try:
            transform = buffer.lookup_transform('base_link', 'livox_frame', rclpy.time.Time())
            extrinsics.publish(TFMessage(transforms=[transform]))
        except TransformException:
            pass

    timer = node.create_timer(.1, repeat_extrinsic)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    command = [os.path.join(get_package_prefix('small_point_lio'), 'lib',
                           'small_point_lio', 'small_point_lio_node'), *sys.argv[1:]]
    child = None
    stopping = False
    code = 0

    def stop(*_):
        nonlocal stopping
        stopping = True
        if child and child.poll() is None:
            child.terminate()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        while not stopping and not buffer.can_transform(
                'base_link', 'livox_frame', rclpy.time.Time()):
            time.sleep(.1)
        while not stopping:
            publisher.publish(Bool(data=False))
            child = subprocess.Popen(command, stdout=subprocess.PIPE,
                                     stderr=subprocess.STDOUT, text=True, bufsize=1)
            retry = False
            for line in child.stdout:
                print(line, end='', flush=True)
                if 'Extrinsic calibration cached:' in line:
                    publisher.publish(Bool(data=True))
                if 'assuming identity transform' in line:
                    publisher.publish(Bool(data=False))
                    node.get_logger().error('Rejected invalid extrinsics; restarting LIO')
                    retry = True
                    child.terminate()
                    break
            try:
                code = child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill()
                code = child.wait()
            if not retry or stopping:
                break
            time.sleep(.5)
    finally:
        publisher.publish(Bool(data=False))
        if child and child.poll() is None:
            child.terminate()
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()
        executor.shutdown()
        spin.join(timeout=2)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if not stopping and code:
        raise SystemExit(code)


if __name__ == '__main__':
    main()
