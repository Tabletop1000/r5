"""A second node for a node's frequent subscriptions, spun by its own single-threaded executor.

job_manager and motor_test need rclpy's MultiThreadedExecutor, because their service and action
callbacks wait for other callbacks to run. That executor costs far more per message than the
single-threaded one: on a laptop, two 30 Hz odometry subscriptions took 92% of a core with it
and 8% without it. Subscriptions that only store the latest message go on this node instead.
Their callbacks then run in this node's thread, so they must not block.
"""

import threading

from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node


class Listener:
    def __init__(self, name: str):
        self.node = Node(name)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self._spin, name=name, daemon=True)

    def _spin(self) -> None:
        try:
            self.executor.spin()
        except ExternalShutdownException:
            pass

    def start(self) -> None:
        self.thread.start()

    def shutdown(self) -> None:
        self.executor.shutdown(timeout_sec=1.0)
        self.node.destroy_node()
