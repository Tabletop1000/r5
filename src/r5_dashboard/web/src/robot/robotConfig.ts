// Facts about the robot the dashboard needs. Keep in step with r5's diffbot_controllers.yaml and
// sensor configs.

export const ROBOT_CONFIG = {
  wheelRadiusM: 0.1,
  leftWheelJoints: ['front_left_wheel_joint', 'rear_left_wheel_joint'],
  rightWheelJoints: ['front_right_wheel_joint', 'rear_right_wheel_joint'],
  /** Expected message rates, Hz. BNO055 data_query_frequency, u-blox rate. */
  imuRateHz: 100,
  gpsRateHz: 1,
  /** IMU temperature above this is shown as a warning, °C. */
  temperatureWarnC: 60,
} as const;
