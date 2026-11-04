"""robot_status node: usage counters, service schedule and alerts for the dashboard.

Publishes  /r5/stats   (r5_dashboard_interfaces/RobotStats, 1 Hz)
           /r5/alerts  (r5_dashboard_interfaces/AlertArray, on change and 1 Hz)
Services   /r5/stats/service_done  (std_srvs/Trigger)  record a completed service
           /r5/alerts/acknowledge  (r5_dashboard_interfaces/AcknowledgeAlert)
Reads      /odometry/local, /diagnostics_agg
"""

from __future__ import annotations

import time
from pathlib import Path

import rclpy
import rclpy.executors
from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_srvs.srv import Trigger

from r5_dashboard_interfaces.msg import Alert, AlertArray, JobStatus, RobotStats
from r5_dashboard_interfaces.srv import AcknowledgeAlert

from .shutdown import init_without_signal_handlers
from .stats import (
    ERROR,
    INFO,
    WARNING,
    AlertTracker,
    DiagnosticEntry,
    ServiceRecord,
    StatsFileDamaged,
    StatsStore,
    service_status,
)

MAINTENANCE_ALERT = "robot:maintenance"
NO_DIAGNOSTICS_ALERT = "robot:no_diagnostics"
REFILL_ALERT = "job:refill"
JOB_PROBLEM_ALERT = "job:problem"
JOB_FAILED_ALERT = "job:failed"


def to_time(seconds: float) -> Time:
    sec = int(seconds)
    return Time(sec=sec, nanosec=int((seconds - sec) * 1e9))


class RobotStatusNode(Node):
    def __init__(self):
        super().__init__("robot_status")
        data_dir = Path(self.declare_parameter("data_dir", "~/.r5").value).expanduser()
        self.interval_km = float(self.declare_parameter("service_interval_km", 100.0).value)
        self.warn_km = float(self.declare_parameter("service_warning_km", 10.0).value)
        self.diag_timeout_s = float(self.declare_parameter("diagnostics_timeout_s", 10.0).value)
        save_period_s = float(self.declare_parameter("save_period_s", 60.0).value)

        self.store = StatsStore(data_dir)
        try:
            self.stats = self.store.load()
        except StatsFileDamaged as exc:
            self.get_logger().error(str(exc))
            self.stats = self.store.load()  # Now empty.
        self.alerts = AlertTracker()
        self.started = time.monotonic()
        self.last_tick = self.started
        self.last_diagnostics = None

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.stats_pub = self.create_publisher(RobotStats, "/r5/stats", latched)
        self.alerts_pub = self.create_publisher(AlertArray, "/r5/alerts", latched)
        self.create_subscription(Odometry, "/odometry/local", self.on_odometry, 10)
        self.create_subscription(DiagnosticArray, "/diagnostics_agg", self.on_diagnostics, 10)
        self.create_subscription(JobStatus, "/r5/job/status", self.on_job, latched)
        self.create_service(Trigger, "/r5/stats/service_done", self.on_service_done)
        self.create_service(AcknowledgeAlert, "/r5/alerts/acknowledge", self.on_acknowledge)
        self.create_timer(1.0, self.tick)
        self.create_timer(save_period_s, self.save)
        self.tick()

    # ---- Inputs ----

    def on_odometry(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.stats.odometer.update(p.x, p.y)

    def on_diagnostics(self, msg: DiagnosticArray) -> None:
        self.last_diagnostics = time.monotonic()
        entries = [DiagnosticEntry(s.name, _level(s.level), s.message) for s in msg.status]
        if self.alerts.update_diagnostics(entries, time.time()):
            self.publish_alerts()

    def on_job(self, msg: JobStatus) -> None:
        """Spray job events that need the user (spec AS-9)."""
        now = time.time()
        changed = self.alerts.set_condition(
            REFILL_ALERT,
            msg.state == JobStatus.WAITING_FOR_REFILL,
            WARNING,
            "Spray job",
            f"Tank empty — robot returned home for refill ({msg.field_name}).",
            now,
        )
        paused_by_failure = msg.state == JobStatus.PAUSED and msg.message.startswith("Navigation failed")
        changed |= self.alerts.set_condition(JOB_PROBLEM_ALERT, paused_by_failure, ERROR, "Spray job", msg.message, now)
        changed |= self.alerts.set_condition(
            JOB_FAILED_ALERT, msg.state == JobStatus.FAILED, ERROR, "Spray job", msg.message, now
        )
        if changed:
            self.publish_alerts()

    def on_service_done(self, _request, response):
        record = ServiceRecord(time.time(), self.stats.odometer.distance_m)
        self.stats.last_service = record
        self.store.log_service(record)
        self.save()
        self.get_logger().info(f"Service recorded at {record.distance_m / 1000:.1f} km")
        self.tick()
        response.success = True
        response.message = "Service recorded."
        return response

    def on_acknowledge(self, request, response):
        response.success = self.alerts.acknowledge(request.id)
        response.message = "Acknowledged." if response.success else "That alert is no longer active."
        if response.success:
            self.publish_alerts()
        return response

    # ---- Periodic ----

    def tick(self) -> None:
        now = time.monotonic()
        self.stats.add_operating_time(now - self.last_tick)
        self.last_tick = now

        status = service_status(self.stats, self.interval_km, self.warn_km)
        if status.overdue:
            message = f"Service overdue by {-status.km_to_service:.1f} km."
        else:
            message = f"Service due in {status.km_to_service:.1f} km."
        self.alerts.set_condition(
            MAINTENANCE_ALERT, status.due_soon or status.overdue, WARNING, "Maintenance", message, time.time()
        )

        diag_age = now - (self.last_diagnostics or self.started)
        missing = diag_age > self.diag_timeout_s
        if missing:
            self.alerts.update_diagnostics([], time.time())  # Old diagnostics can't be trusted.
        self.alerts.set_condition(
            NO_DIAGNOSTICS_ALERT,
            missing,
            INFO,
            "Diagnostics",
            "No diagnostics from the robot. Is diagnostic_aggregator running?",
            time.time(),
        )

        self.publish_stats(status)
        self.publish_alerts()

    def publish_stats(self, status) -> None:
        msg = RobotStats()
        msg.operating_hours = self.stats.operating_s / 3600.0
        msg.distance_km = self.stats.odometer.distance_m / 1000.0
        msg.service_interval_km = self.interval_km
        msg.km_to_service = status.km_to_service
        msg.service_due_soon = status.due_soon
        msg.service_overdue = status.overdue
        if self.stats.last_service:
            msg.has_last_service = True
            msg.last_service_time = to_time(self.stats.last_service.time_s)
            msg.last_service_distance_km = self.stats.last_service.distance_m / 1000.0
        self.stats_pub.publish(msg)

    def publish_alerts(self) -> None:
        msg = AlertArray()
        for a in self.alerts.alerts:
            msg.alerts.append(
                Alert(
                    id=a.id,
                    severity=a.severity,
                    source=a.source,
                    message=a.message,
                    since=to_time(a.since_s),
                    acknowledged=a.acknowledged,
                )
            )
        self.alerts_pub.publish(msg)

    def save(self) -> None:
        try:
            self.store.save(self.stats)
        except OSError as exc:
            self.get_logger().error(f"Couldn't save usage counters: {exc}")


def _level(level) -> int:
    # DiagnosticStatus.level is a byte, which Python sees as bytes of length 1.
    return level[0] if isinstance(level, (bytes, bytearray)) else int(level)


def main(args=None):
    init_without_signal_handlers(args)
    node = RobotStatusNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.save()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
