# Robot-side TODO

Work needed on the robot (the `r5` repository, its configuration, hardware and firmware) for the
dashboard to be complete. Dashboard-side code lives in this repository and is tracked by the
delivery plan in [spec.md](spec.md) section 9.

Each item says what the dashboard expects, so the two sides meet without guesswork. Until an item
is done, the dashboard shows "Not available" for it rather than failing.

Last updated: 2026-11-04 (after delivery step 3, Manage Fields).

## Status data

- [ ] **Battery.** Publish `sensor_msgs/msg/BatteryState` on `/battery_state` at 1 Hz or more.
  - Dashboard reads `percentage` (0–1; NaN if unknown), `voltage`, and `power_supply_status`
    (1 charging, 2 discharging, 3 not charging, 4 full).
  - Shown in the status strip and on the Robot page.
- [ ] **WiFi signal strength.** Publish `std_msgs/msg/Float32` on `/r5/wifi_signal_dbm` at 1 Hz:
  the long-range link's received signal in dBm (for example from `iw dev <iface> link` or
  `/proc/net/wireless`). The dashboard colours it: −67 dBm or better good, down to −80 dBm fair,
  worse is poor. Round-trip time already works.
- [ ] **Software and firmware versions.** Publish `diagnostic_msgs/msg/DiagnosticStatus` on
  `/r5/versions` with transient-local durability (latched), once at startup. Put one entry per
  component in `values`, e.g. `{key: "r5", value: "<git describe>"}`,
  `{key: "motor node", value: "<firmware version>"}`.
- [ ] **Temperatures.** The IMU temperature (`/bno055/temp`) is shown already. Add the main
  computer and motor driver temperatures to `/diagnostics` with warning/error thresholds, so
  overheating appears as an alert. (A dedicated display can be added to the dashboard later.)
- [ ] **Motor current** (spec RB-7). Report each wheel's motor current. Proposal: an `effort` state
  interface in the ros2_control hardware plugin, carrying current in amps, so it appears in
  `/joint_states.effort`. The dashboard then needs a small change to show it.
- [ ] **Diagnostics.** Run `diagnostic_aggregator` on the robot so `/diagnostics_agg` exists.
  - Group analysers by area, e.g. `/Sensors/IMU`, `/Sensors/GPS`, `/Motors/Left`, `/Computer`.
    The last part of the name is shown as the alert's source.
  - Add `diagnostic_updater` reporting to the motor node interface (link up, faults) if it doesn't
    already have it.
  - Until this runs, the dashboard shows an info alert, "No diagnostics from the robot".
- [ ] **BNO055 calibration rate.** `calib_status_frequency` is 0.1 Hz, so calibration can take
  10 s to appear on the Robot page. Consider 1 Hz.

## GNSS (ZED-F9 RTK)

- [ ] Add a ZED-F9 configuration to `r5_sensors` alongside `m9n.yaml`.
- [ ] Make sure the u-blox driver publishes `ublox_msgs/msg/NavPVT` on `/navpvt`. The dashboard
  uses it for satellite count, RTK float/fixed, and horizontal accuracy (`h_acc`, mm). The
  `ublox_msgs` package must be installed wherever rosbridge runs (it is, on the robot).
- [ ] Feed corrections from the base station to the receiver (radio or serial link).
- [ ] **RTK correction age** (spec RI-8). Publish `std_msgs/msg/Float32` on
  `/r5/gnss/correction_age_s`: seconds since the last correction, from NavPVT `flags3`
  (last correction age) or the RTCM input. The dashboard then needs a small change to show it,
  and should warn when it grows.
- [ ] Set `fix_mode`, rates and the RTK-fixed requirement for field recording (spec FM-4).

## Motion and safety

- [ ] **Velocity arbitration.** Add `twist_mux` (TwistStamped) in front of the base controller.
  Suggested priority, highest first: Xbox controller, dashboard motor test, Nav2.
  - Then set the dashboard `motor_test` node's `cmd_vel_topic` parameter to its mux input. Today it
    publishes straight to `/cmd_vel`, which Nav2's velocity smoother also publishes to.
  - The dashboard's software Stop button (spec NAV-4, in step 4) will use this too.
- [ ] **Hardware emergency stop**, independent of all software (spec NF-9).
- [ ] **Motor test limits.** The `motor_test` node defaults to 0.06 m/s for 2 s, with hard caps of
  0.1 m/s and 5 s in code. Check these are safe on the real robot, and that `wheel_separation`
  (0.36 m), `wheel_radius` (0.1 m) and the wheel joint names match `diffbot_controllers.yaml`.
  The dashboard has the same wheel facts in `web/src/robot/robotConfig.ts`.
- [ ] **Joint velocities.** The motor test and wheel speed readout need `/joint_states.velocity` from
  the real hardware plugin, in rad/s.

## Driving and recording fields (step 3)

- [ ] **Xbox controller.** Run `joy` and `teleop_twist_joy` on the robot, with the controller connected
  to the robot (USB dongle or Bluetooth), not to the laptop (spec RI-4 to RI-7).
  - Use a held enable (deadman) button, a speed cap, and TwistStamped output into `twist_mux`.
  - `joy` must publish `sensor_msgs/Joy` on `/joy`. The dashboard's `field_manager` reads it: holding
    button index 3 (Y on an Xbox controller under the Linux driver) for 1 s starts or stops a boundary
    recording. Change `joy_record_button` if Y is used for something else, or set it to −1 to turn
    this off. Make sure the record button isn't also the deadman or a drive button.
- [ ] **RTK for recording.** `field_manager` only keeps positions with an RTK fixed solution by
  default (`record_fix_requirement`). It tells RTK fixed from float using `/navpvt`
  (`ublox_msgs/NavPVT`), so the u-blox driver must publish it and `ublox_msgs` must be installed
  where `field_manager` runs. Without `/navpvt` it can only see "corrections in use" from `/fix`.
- [ ] **Simulator GPS noise (optional, for testing).** The simulated GPS has 1.5 m noise, so a
  boundary recorded in Gazebo zigzags and crosses itself. To try recording realistically in
  simulation, lower the noise in `gps.urdf.xacro` (and the matching covariance in
  `sensor_covariance.py`) to RTK-like values, e.g. 0.02 m.

## Installation on the robot

- [ ] Link this repository into the robot workspace, run `rosdep install`, and build
  `r5_dashboard_interfaces` and `r5_dashboard_server` (see [README.md](README.md)).
- [ ] Start `r5_dashboard_server`'s `dashboard.launch.py` from `r5_bringup/r5.launch.py`.
- [ ] Set the dashboard password: `ros2 run r5_dashboard_server set_password`.
- [ ] Open the firewall for TCP 8443 (HTTPS) and 8080 (redirect). Keep 9090 (rosbridge) closed; it
  only listens on localhost anyway.
- [ ] Optional: serve on 443 so the address is just `https://<robot>`. This needs
  `CAP_NET_BIND_SERVICE` for the server (or a port redirect).
- [ ] **Clock.** Alert times, service dates and session expiry use the robot's clock. Make sure
  it is set at boot (NTP over the WiFi link, or GPS time via chrony), especially if the computer
  has no real-time clock.
- [ ] Back up `~/.r5` (stats, service log, password hash, certificate), and later fields.

## Coming with later dashboard steps

These are listed now so the hardware and robot configuration can be planned. Details will be
filled in as each step is designed.

- [ ] **Step 4, Area Spray:**
  - Fields2Cover integration.
  - Sprayer placeholder interfaces from spec 5.5: `/r5/sprayer/enable` (`std_msgs/Bool`) and
    `/r5/sprayer/flow_setpoint` (`std_msgs/Float32`, L/min), and a stub for the simulator.
  - Nav2 navigation to the home point for refills (spec AS-9).
- [ ] **Step 4, motor test interlock:** once the job manager exists, refuse motor tests while a job
  is running (spec RB-10).
