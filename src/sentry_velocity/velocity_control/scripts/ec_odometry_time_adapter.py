#!/usr/bin/env python3
"""Keep upstream EC linear feedback; derive virtual yaw speed with real stamps."""
import copy
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class Adapter(Node):
    def __init__(self):
        super().__init__('ec_odometry_time_adapter')
        self.previous = None
        self.publisher = self.create_publisher(Odometry, '/Odometry/EC', 10)
        self.subscription = self.create_subscription(
            Odometry, '/simulation/raw_odometry_ec', self.convert, 10)

    def convert(self, message):
        q = message.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        stamp = message.header.stamp.sec + message.header.stamp.nanosec*1e-9
        output = copy.deepcopy(message)
        output.twist.twist.angular.z = 0.0
        if self.previous is not None:
            dt = stamp-self.previous[0]
            if dt == 0:
                return
            if 0 < dt < 0.5:
                delta = yaw-self.previous[1]
                output.twist.twist.angular.z = math.atan2(math.sin(delta),math.cos(delta))/dt
        self.previous = stamp, yaw
        self.publisher.publish(output)

def main():
    rclpy.init()
    node=Adapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
