#!/usr/bin/env python3
"""Add the covariance Gazebo leaves out, so the EKFs weight simulated sensors like real ones."""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix

# From r5_sensors/config/bno055_params.yaml.
IMU_ORIENTATION_VARIANCE = 0.0159
IMU_ANGULAR_VELOCITY_VARIANCE = 0.04
IMU_ACCELERATION_VARIANCE = 0.017

# The standard deviations set on the GPS sensor in gps.urdf.xacro: 1.5 m and 2.5 m.
GPS_HORIZONTAL_VARIANCE = 2.25
GPS_VERTICAL_VARIANCE = 6.25


def diagonal(variance):
    return [variance, 0.0, 0.0, 0.0, variance, 0.0, 0.0, 0.0, variance]


class SensorCovariance(Node):

    def __init__(self):
        super().__init__("sensor_covariance")
        self.imu_pub = self.create_publisher(Imu, "/bno055/imu", qos_profile_sensor_data)
        self.fix_pub = self.create_publisher(NavSatFix, "/fix", qos_profile_sensor_data)
        self.create_subscription(Imu, "/sim/imu", self.on_imu, qos_profile_sensor_data)
        self.create_subscription(NavSatFix, "/sim/fix", self.on_fix, qos_profile_sensor_data)

    def on_imu(self, msg):
        msg.orientation_covariance = diagonal(IMU_ORIENTATION_VARIANCE)
        msg.angular_velocity_covariance = diagonal(IMU_ANGULAR_VELOCITY_VARIANCE)
        msg.linear_acceleration_covariance = diagonal(IMU_ACCELERATION_VARIANCE)
        self.imu_pub.publish(msg)

    def on_fix(self, msg):
        msg.position_covariance = [
            GPS_HORIZONTAL_VARIANCE, 0.0, 0.0,
            0.0, GPS_HORIZONTAL_VARIANCE, 0.0,
            0.0, 0.0, GPS_VERTICAL_VARIANCE,
        ]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.fix_pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(SensorCovariance())


if __name__ == "__main__":
    main()
