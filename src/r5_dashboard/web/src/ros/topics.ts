// ROS topics the dashboard reads. Names marked "placeholder" don't have a publisher on the robot yet
// (spec 4.4, Q8); the dashboard shows "Not available" until data arrives.

export const TOPICS = {
  /** sensor_msgs/BatteryState — placeholder, no publisher yet. */
  battery: { name: '/battery_state', type: 'sensor_msgs/msg/BatteryState', throttleMs: 1000 },
  /** diagnostic_msgs/DiagnosticArray from diagnostic_aggregator (spec 5.1). */
  diagnostics: { name: '/diagnostics_agg', type: 'diagnostic_msgs/msg/DiagnosticArray', throttleMs: 1000 },
} as const;

export interface BatteryState {
  voltage: number | null;
  percentage: number | null;
  power_supply_status: number;
}

export interface DiagnosticStatus {
  level: number;
  name: string;
  message: string;
  hardware_id: string;
}

export interface DiagnosticArray {
  status: DiagnosticStatus[];
}
