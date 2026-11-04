#!/usr/bin/env python3
"""Report the computer temperature, WiFi signal and software versions for the dashboard."""

import os
from pathlib import Path
import subprocess

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float32

THERMAL_ZONES = Path("/sys/class/thermal")
WIRELESS = Path("/proc/net/wireless")


def read_temperature_c():
    """Return the hottest thermal zone in °C, or None if there are none."""
    temps = []
    for zone in THERMAL_ZONES.glob("thermal_zone*/temp"):
        try:
            temps.append(int(zone.read_text()) / 1000.0)
        except (OSError, ValueError):
            pass
    return max(temps, default=None)


def read_wifi_dbm(interface):
    """Return (interface, signal in dBm) from /proc/net/wireless, or None if it isn't listed."""
    try:
        lines = WIRELESS.read_text().splitlines()[2:]
    except OSError:
        return None
    for line in lines:
        name, _, fields = line.partition(":")
        name = name.strip()
        values = fields.split()
        if len(values) < 3 or (interface and name != interface):
            continue
        level = float(values[2].rstrip("."))
        # Some drivers report an unsigned byte, where 256 means 0 dBm.
        return name, level - 256 if level > 0 else level
    return None


def git_version(*paths):
    """Return `git describe` for the first repository containing one of paths, or 'unknown'."""
    for parent in [p for path in paths for p in (path, *path.parents)]:
        if (parent / ".git").exists():
            try:
                result = subprocess.run(
                    ["git", "-c", "safe.directory=*", "-C", str(parent),
                     "describe", "--always", "--dirty", "--tags"],
                    capture_output=True, text=True, timeout=5, check=True)
                return result.stdout.strip()
            except (OSError, subprocess.SubprocessError):
                break
    return "unknown"


def status(name, level, message, **values):
    return DiagnosticStatus(
        name=name, level=level, message=message, hardware_id="r5",
        values=[KeyValue(key=k, value=str(v)) for k, v in values.items()])


class SystemMonitor(Node):

    def __init__(self):
        super().__init__("system_monitor")
        self.wifi_interface = self.declare_parameter("wifi_interface", "").value
        # A Raspberry Pi starts throttling at 80-85 °C.
        self.temp_warn_c = self.declare_parameter("temperature_warning_c", 75.0).value
        self.temp_error_c = self.declare_parameter("temperature_error_c", 80.0).value
        self.wifi_poor_dbm = self.declare_parameter("wifi_poor_dbm", -80.0).value
        repo_path = self.declare_parameter("repo_path", "").value

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.versions_pub = self.create_publisher(DiagnosticStatus, "/r5/versions", latched)
        self.wifi_pub = self.create_publisher(Float32, "/r5/wifi_signal_dbm", 10)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)

        # On the robot the workspace is the repository, so the installed script is inside it. The
        # working directory is a fallback for installs elsewhere.
        paths = [Path(repo_path)] if repo_path else [Path(os.path.realpath(__file__)), Path.cwd()]
        self.versions_pub.publish(status(
            "Versions", DiagnosticStatus.OK, "",
            r5=git_version(*paths), ROS=os.environ.get("ROS_DISTRO", "unknown")))

        self.warned_no_wifi = False
        self.create_timer(1.0, self.update)

    def update(self):
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()

        temp = read_temperature_c()
        if temp is None:
            array.status.append(status(
                "r5: Computer temperature", DiagnosticStatus.WARN, "No temperature sensor"))
        else:
            level, message = DiagnosticStatus.OK, f"{temp:.0f} °C"
            if temp >= self.temp_error_c:
                level, message = DiagnosticStatus.ERROR, f"Overheating: {temp:.0f} °C"
            elif temp >= self.temp_warn_c:
                level, message = DiagnosticStatus.WARN, f"Hot: {temp:.0f} °C"
            array.status.append(status(
                "r5: Computer temperature", level, message, celsius=f"{temp:.1f}"))

        wifi = read_wifi_dbm(self.wifi_interface)
        if wifi is None:
            if not self.warned_no_wifi:
                self.get_logger().warning("No wireless interface found, so no WiFi signal.")
                self.warned_no_wifi = True
        else:
            name, dbm = wifi
            self.wifi_pub.publish(Float32(data=dbm))
            level = DiagnosticStatus.WARN if dbm < self.wifi_poor_dbm else DiagnosticStatus.OK
            array.status.append(status(
                "r5: WiFi", level, f"{dbm:.0f} dBm", interface=name, dbm=f"{dbm:.0f}"))

        self.diagnostics_pub.publish(array)


def main():
    rclpy.init()
    rclpy.spin(SystemMonitor())


if __name__ == "__main__":
    main()
