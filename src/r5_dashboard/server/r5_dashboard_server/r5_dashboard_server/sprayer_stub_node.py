"""sprayer_stub node: stands in for the unfinished sprayer (spec 5.5).

Accepts /r5/sprayer/enable (std_msgs/Bool) and /r5/sprayer/flow_setpoint (std_msgs/Float32, L/min)
and logs what a real sprayer would do. Run it with the simulator, or on the robot until the sprayer
hardware exists.
"""

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Float32

from .shutdown import init_without_signal_handlers


class SprayerStub(Node):
    def __init__(self):
        super().__init__("sprayer_stub")
        self.on = False
        self.flow = 0.0
        self.litres = 0.0
        self.last = self.get_clock().now()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, "/r5/sprayer/enable", self.on_enable, latched)
        self.create_subscription(Float32, "/r5/sprayer/flow_setpoint", self.on_flow, 10)
        self.get_logger().info("Sprayer stub running: commands are logged, nothing is sprayed.")

    def on_enable(self, msg: Bool) -> None:
        if msg.data != self.on:
            self.on = msg.data
            self.get_logger().info(f"Sprayer {'ON' if self.on else 'OFF'} (total {self.litres:.2f} L)")

    def on_flow(self, msg: Float32) -> None:
        now = self.get_clock().now()
        dt = (now - self.last).nanoseconds / 1e9
        self.last = now
        if self.on and 0 < dt < 2:
            self.litres += self.flow * dt / 60.0
        self.flow = msg.data


def main(args=None):
    init_without_signal_handlers(args)
    node = SprayerStub()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
