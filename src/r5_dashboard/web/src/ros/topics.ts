// ROS names the dashboard uses. Names marked "placeholder" don't have a publisher on the robot yet
// (see TODO_ROBOT.md); the dashboard shows "Not available" until data arrives.

export interface TopicSpec {
  name: string;
  type: string;
  throttleMs?: number;
}

export const TOPICS = {
  /** Usage counters and service schedule, from r5_dashboard_server's robot_status node. */
  stats: { name: '/r5/stats', type: 'r5_dashboard_interfaces/msg/RobotStats', throttleMs: 1000 },
  /** Active alerts, from robot_status. */
  alerts: { name: '/r5/alerts', type: 'r5_dashboard_interfaces/msg/AlertArray', throttleMs: 500 },
  /** Placeholder: no publisher yet. */
  battery: { name: '/battery_state', type: 'sensor_msgs/msg/BatteryState', throttleMs: 1000 },
  /** Placeholder: WiFi signal strength in dBm, no publisher yet. */
  wifiSignal: { name: '/r5/wifi_signal_dbm', type: 'std_msgs/msg/Float32', throttleMs: 1000 },
  /** Placeholder: software and firmware versions, no publisher yet. */
  versions: { name: '/r5/versions', type: 'diagnostic_msgs/msg/DiagnosticStatus' },
  fix: { name: '/fix', type: 'sensor_msgs/msg/NavSatFix' },
  /** u-blox position/velocity/time message. Only on the real robot; gives satellites and RTK state. */
  navPvt: { name: '/navpvt', type: 'ublox_msgs/msg/NavPVT', throttleMs: 500 },
  imu: { name: '/bno055/imu', type: 'sensor_msgs/msg/Imu', throttleMs: 200 },
  /** BNO055 calibration, a JSON string like {"sys": 3, "gyro": 3, "accel": 3, "mag": 3}. */
  imuCalibration: { name: '/bno055/calib_status', type: 'std_msgs/msg/String' },
  imuTemperature: { name: '/bno055/temp', type: 'sensor_msgs/msg/Temperature', throttleMs: 1000 },
  jointStates: { name: '/joint_states', type: 'sensor_msgs/msg/JointState', throttleMs: 200 },
  /** Boundary recording state and track, from field_manager. */
  recordingStatus: { name: '/r5/recording/status', type: 'r5_dashboard_interfaces/msg/RecordingStatus', throttleMs: 500 },
  /** Goes up whenever fields or the home point change, so every open dashboard reloads them. */
  fieldsRevision: { name: '/r5/fields/revision', type: 'std_msgs/msg/UInt32' },
} as const satisfies Record<string, TopicSpec>;

export const SERVICES = {
  serviceDone: { name: '/r5/stats/service_done', type: 'std_srvs/srv/Trigger' },
  acknowledgeAlert: { name: '/r5/alerts/acknowledge', type: 'r5_dashboard_interfaces/srv/AcknowledgeAlert' },
  listFields: { name: '/r5/fields/list', type: 'r5_dashboard_interfaces/srv/ListFields' },
  saveField: { name: '/r5/fields/save', type: 'r5_dashboard_interfaces/srv/SaveField' },
  deleteField: { name: '/r5/fields/delete', type: 'r5_dashboard_interfaces/srv/DeleteField' },
  getHome: { name: '/r5/home/get', type: 'r5_dashboard_interfaces/srv/GetHome' },
  setHome: { name: '/r5/home/set', type: 'r5_dashboard_interfaces/srv/SetHome' },
  startRecording: { name: '/r5/recording/start', type: 'std_srvs/srv/Trigger' },
  stopRecording: { name: '/r5/recording/stop', type: 'std_srvs/srv/Trigger' },
  clearRecording: { name: '/r5/recording/clear', type: 'std_srvs/srv/Trigger' },
  /** From rosapi, which starts with rosbridge. Used to measure round-trip time. */
  getTime: { name: '/rosapi/get_time', type: 'rosapi_msgs/srv/GetTime' },
} as const;

export const ACTIONS = {
  motorTest: { name: '/r5/motor_test', type: 'r5_dashboard_interfaces/action/MotorTest' },
} as const;

// ---- Message shapes (only the fields the dashboard reads) ----

export interface RosTime {
  sec: number;
  nanosec: number;
}

export interface Header {
  stamp: RosTime;
  frame_id: string;
}

export interface BatteryState {
  voltage: number | null;
  percentage: number | null;
  power_supply_status: number;
}

export interface RobotStats {
  operating_hours: number;
  distance_km: number;
  service_interval_km: number;
  km_to_service: number;
  service_due_soon: boolean;
  service_overdue: boolean;
  has_last_service: boolean;
  last_service_time: RosTime;
  last_service_distance_km: number;
}

export const ALERT_SEVERITY = { INFO: 0, WARNING: 1, ERROR: 2 } as const;

export interface Alert {
  id: string;
  severity: number;
  source: string;
  message: string;
  since: RosTime;
  acknowledged: boolean;
}

export interface AlertArray {
  alerts: Alert[];
}

export interface KeyValue {
  key: string;
  value: string;
}

export interface DiagnosticStatus {
  level: number;
  name: string;
  message: string;
  hardware_id: string;
  values: KeyValue[];
}

export interface NavSatFix {
  header: Header;
  status: { status: number; service: number };
  latitude: number;
  longitude: number;
  altitude: number;
  position_covariance: number[];
  position_covariance_type: number;
}

export interface NavPvt {
  fix_type: number;
  flags: number;
  num_sv: number;
  /** Horizontal accuracy estimate, mm. */
  h_acc: number;
}

export interface Quaternion {
  x: number;
  y: number;
  z: number;
  w: number;
}

export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

export interface Imu {
  header: Header;
  orientation: Quaternion;
  angular_velocity: Vector3;
  linear_acceleration: Vector3;
}

export interface Temperature {
  temperature: number;
}

export interface JointState {
  name: string[];
  position: number[];
  velocity: number[];
  effort: number[];
}

export interface Float32 {
  data: number;
}

export interface StringMsg {
  data: string;
}

export interface TriggerResponse {
  success: boolean;
  message: string;
}

export interface MotorTestGoal {
  side: number;
  forward: boolean;
}

export interface MotorTestFeedback {
  elapsed_s: number;
  duration_s: number;
  speeds_valid: boolean;
  left_speed: number;
  right_speed: number;
}

export interface MotorTestResult {
  success: boolean;
  message: string;
  commanded_left: number;
  commanded_right: number;
  measured_valid: boolean;
  measured_left: number;
  measured_right: number;
}

// ---- Fields (r5_dashboard_interfaces) ----

export interface LatLon {
  latitude: number;
  longitude: number;
}

export interface GeoPolygon {
  points: LatLon[];
}

export interface FieldMsg {
  id: string;
  name: string;
  boundary: LatLon[];
  keep_out: GeoPolygon[];
  headland_width_m: number;
  source: string;
  notes: string;
  area_m2: number;
  created: RosTime;
  modified: RosTime;
}

export const RECORDING_STATE = { IDLE: 0, RECORDING: 1, FINISHED: 2 } as const;

export interface RecordingStatus {
  state: number;
  points: LatLon[];
  skipped_points: number;
  length_m: number;
  required_fix: string;
  current_fix: string;
  message: string;
  started: RosTime;
}

export interface SaveFieldResponse {
  success: boolean;
  message: string;
  field: FieldMsg;
}

export interface GetHomeResponse {
  is_set: boolean;
  position: LatLon;
}

export interface UInt32 {
  data: number;
}
