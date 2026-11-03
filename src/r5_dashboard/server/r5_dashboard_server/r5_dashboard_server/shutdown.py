"""Clean shutdown for the dashboard's nodes.

rclpy's own signal handler shuts the ROS context down before a node's `finally` block runs, so a
last "stop the motors" message or a final save couldn't be published. These nodes turn SIGINT and
SIGTERM into KeyboardInterrupt instead, and shut ROS down themselves afterwards.
"""

import signal

import rclpy
from rclpy.signals import SignalHandlerOptions


def init_without_signal_handlers(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    def interrupt(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
