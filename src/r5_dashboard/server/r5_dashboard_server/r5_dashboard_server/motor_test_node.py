"""motor_test node: bounded motor tests for the Robot page (spec RB-9, RB-10).

Action  /r5/motor_test  (r5_dashboard_interfaces/action/MotorTest)

Speed and duration are parameters on the robot, capped in code, and the goal can't change them.
The test always ends after duration_s, so a dropped link can't leave the motors running.
"""

from __future__ import annotations

import threading
import time

import rclpy
import rclpy.executors
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from r5_dashboard_interfaces.action import MotorTest
from r5_dashboard_interfaces.msg import JobStatus
from rclpy.qos import DurabilityPolicy, QoSProfile

from .shutdown import init_without_signal_handlers
from .motor_test import BOTH, evaluate, side_name, twist_for, wheel_speeds

# JobStatus states while a job is under way (spec RB-10: no motor tests then).
JOB_ACTIVE = {1, 2, 3, 4, 5, 6, 7}

# Hard limits, whatever the parameters say.
MAX_TEST_SPEED_MPS = 0.1
MAX_DURATION_S = 5.0
RATE_HZ = 20.0


class MotorTestNode(Node):
    def __init__(self):
        super().__init__("motor_test")
        p = self.declare_parameter
        self.wheel_separation = float(p("wheel_separation", 0.36).value)
        self.wheel_radius = float(p("wheel_radius", 0.1).value)
        self.speed = min(float(p("test_speed_mps", 0.06).value), MAX_TEST_SPEED_MPS)
        self.duration = min(float(p("duration_s", 2.0).value), MAX_DURATION_S)
        self.left_joints = list(p("left_wheel_joints", ["front_left_wheel_joint", "rear_left_wheel_joint"]).value)
        self.right_joints = list(p("right_wheel_joints", ["front_right_wheel_joint", "rear_right_wheel_joint"]).value)
        self.moving_threshold = float(p("moving_threshold_mps", 0.02).value)
        self.stamped = bool(p("stamped_cmd_vel", True).value)
        cmd_topic = str(p("cmd_vel_topic", "/cmd_vel").value)

        group = ReentrantCallbackGroup()
        self.cmd_pub = self.create_publisher(TwistStamped if self.stamped else Twist, cmd_topic, 10)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10, callback_group=group)
        self.create_subscription(Odometry, "/odometry/local", self.on_odometry, 10, callback_group=group)
        self.job_active = False
        self.create_subscription(
            JobStatus,
            "/r5/job/status",
            self.on_job,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group=group,
        )

        self.lock = threading.Lock()
        self.busy = False
        self.wheel_mps: tuple[float, float] | None = None
        self.wheel_stamp = 0.0
        self.odom_twist: tuple[float, float] | None = None

        ActionServer(
            self,
            MotorTest,
            "/r5/motor_test",
            execute_callback=self.execute,
            goal_callback=lambda _goal: GoalResponse.ACCEPT,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT,
            callback_group=group,
        )
        self.get_logger().info(
            f"Motor tests run at {self.speed:.3f} m/s for {self.duration:.1f} s on {cmd_topic}"
        )

    def on_joints(self, msg: JointState) -> None:
        if not msg.velocity:
            return
        by_name = dict(zip(msg.name, msg.velocity))
        left = [by_name[j] for j in self.left_joints if j in by_name]
        right = [by_name[j] for j in self.right_joints if j in by_name]
        if left and right:
            self.wheel_mps = (
                sum(left) / len(left) * self.wheel_radius,
                sum(right) / len(right) * self.wheel_radius,
            )
            self.wheel_stamp = time.monotonic()

    def on_job(self, msg: JobStatus) -> None:
        self.job_active = msg.state in JOB_ACTIVE

    def on_odometry(self, msg: Odometry) -> None:
        self.odom_twist = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def publish_cmd(self, linear: float, angular: float) -> None:
        if self.stamped:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            twist = msg.twist
        else:
            msg = twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_pub.publish(msg)

    def refuse(self, goal_handle, message: str):
        goal_handle.succeed()
        self.get_logger().warning(f"Motor test refused: {message}")
        return MotorTest.Result(success=False, message=message)

    def execute(self, goal_handle):
        side, forward = goal_handle.request.side, goal_handle.request.forward
        if side > BOTH:
            return self.refuse(goal_handle, f"Unknown side {side}.")
        with self.lock:
            if self.busy:
                return self.refuse(goal_handle, "Another motor test is already running.")
            self.busy = True
        try:
            if self.job_active:
                return self.refuse(goal_handle, "A spray job is running. Stop it before testing the motors.")
            if self.odom_twist and (
                abs(self.odom_twist[0]) > self.moving_threshold or abs(self.odom_twist[1]) > 0.05
            ):
                return self.refuse(goal_handle, "The robot is moving. Motor tests only run while it's standing still.")
            return self.run(goal_handle, side, forward)
        finally:
            self.stop_motors()
            with self.lock:
                self.busy = False

    def run(self, goal_handle, side: int, forward: bool):
        commanded = wheel_speeds(side, forward, self.speed)
        linear, angular = twist_for(*commanded, self.wheel_separation)
        direction = "forward" if forward else "reverse"
        self.get_logger().info(f"Motor test: {side_name(side)} {direction}")

        samples: list[tuple[float, float]] = []
        start = time.monotonic()
        period = 1.0 / RATE_HZ
        while (elapsed := time.monotonic() - start) < self.duration:
            if goal_handle.is_cancel_requested:
                self.stop_motors()
                goal_handle.canceled()
                return MotorTest.Result(success=False, message="Stopped by the user.")
            self.publish_cmd(linear, angular)
            fresh = self.wheel_mps if time.monotonic() - self.wheel_stamp < 0.5 else None
            if fresh and elapsed > self.duration / 2:
                samples.append(fresh)
            goal_handle.publish_feedback(
                MotorTest.Feedback(
                    elapsed_s=elapsed,
                    duration_s=self.duration,
                    speeds_valid=fresh is not None,
                    left_speed=fresh[0] if fresh else 0.0,
                    right_speed=fresh[1] if fresh else 0.0,
                )
            )
            time.sleep(period)

        self.stop_motors()
        measured = None
        if samples:
            measured = (
                sum(s[0] for s in samples) / len(samples),
                sum(s[1] for s in samples) / len(samples),
            )
        ok, message = evaluate(side, commanded, measured)
        goal_handle.succeed()
        return MotorTest.Result(
            success=ok,
            message=message,
            commanded_left=commanded[0],
            commanded_right=commanded[1],
            measured_valid=measured is not None,
            measured_left=measured[0] if measured else 0.0,
            measured_right=measured[1] if measured else 0.0,
        )

    def stop_motors(self) -> None:
        for _ in range(3):
            self.publish_cmd(0.0, 0.0)


def main(args=None):
    init_without_signal_handlers(args)
    node = MotorTestNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.stop_motors()
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
