#!/usr/bin/env python3

"""Start the simulation RViz only after the latched static map is available."""

import os
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class MapWaiter(Node):
    def __init__(self):
        super().__init__("rviz_after_map")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("rviz_config", "")
        self.declare_parameter("map_wait_timeout", 20.0)
        self.map_received = False

        topic = self.get_parameter("map_topic").value
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.subscription = self.create_subscription(
            OccupancyGrid, topic, self._on_map, qos
        )
        self.get_logger().info(
            f"Waiting for the first static map on {topic} before opening RViz"
        )

    def _on_map(self, _message):
        self.map_received = True


def main(args=None):
    rclpy.init(args=args)
    node = MapWaiter()
    rviz_config = node.get_parameter("rviz_config").value
    timeout = float(node.get_parameter("map_wait_timeout").value)
    # Use wall time for the watchdog.  The simulated clock may be stopped
    # while Gazebo is still coming up, and must not disable this fallback.
    start = time.monotonic()
    while rclpy.ok() and not node.map_received:
        rclpy.spin_once(node, timeout_sec=0.25)
        if timeout > 0.0:
            elapsed = time.monotonic() - start
            if elapsed >= timeout:
                node.get_logger().error(
                    f"No map received on {node.get_parameter('map_topic').value} "
                    f"after {timeout:.1f}s; opening RViz anyway for diagnosis"
                )
                break

    if node.map_received:
        node.get_logger().info("Static map received; opening RViz")
    node.destroy_node()
    rclpy.shutdown()
    command = ["rviz2"]
    if rviz_config:
        command.extend(["-d", rviz_config])
    command.extend(["--ros-args", "-p", "use_sim_time:=true"])
    # Prefer the discrete NVIDIA GLX provider on hybrid-GPU machines.  OGRE's
    # Copy render-to-texture mode also avoids the indexed map shader failure
    # seen with the default framebuffer path.
    os.environ.pop("DRI_PRIME", None)
    os.environ["__GLX_VENDOR_LIBRARY_NAME"] = "nvidia"
    os.environ["__NV_PRIME_RENDER_OFFLOAD"] = "1"
    os.environ["__VK_LAYER_NV_optimus"] = "NVIDIA_only"
    os.environ["CUDA_VISIBLE_DEVICES"] = "0"
    os.environ["QT_XCB_GL_INTEGRATION"] = "xcb_glx"
    os.environ["OGRE_RTT_MODE"] = "Copy"
    os.execvp(command[0], command)


if __name__ == "__main__":
    main()
