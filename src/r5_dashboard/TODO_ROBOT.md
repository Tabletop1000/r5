# Robot-side TODO

Work needed on the robot (the `r5` repository, its configuration, hardware and firmware) for the
dashboard to be complete. Dashboard-side code lives in this repository and is tracked by the
delivery plan in [spec.md](spec.md) section 9.

Each item says what the dashboard expects, so the two sides meet without guesswork. Until an item
is done, the dashboard shows "Not available" for it rather than failing.

Last updated: 2026-11-04 (after robot phases 0 to 4: repository move, twist_mux, robot monitoring and
the Xbox controller).

## Status data

- [ ] **Battery.** An INA226 will measure the main battery. Publish `sensor_msgs/msg/BatteryState` on `/battery_state` at 1 Hz or more.
  - Dashboard reads `percentage` (0–1; NaN if unknown), `voltage`, and `power_supply_status`
    (1 charging, 2 discharging, 3 not charging, 4 full).
  - Shown in the status strip and on the Robot page.
- [x] **WiFi signal strength.** `r5_monitor`'s `system_monitor` publishes it from
  `/proc/net/wireless` (parameter `wifi_interface`, default the first listed). If the long-range
  link is an external radio connected by Ethernet, the Pi can't see its signal, and this needs the
  radio's own status page or API instead. Original request: publish `std_msgs/msg/Float32` on `/r5/wifi_signal_dbm` at 1 Hz:
  the long-range link's received signal in dBm (for example from `iw dev <iface> link` or
  `/proc/net/wireless`). The dashboard colours it: −67 dBm or better good, down to −80 dBm fair,
  worse is poor. Round-trip time already works.
- [x] **Software and firmware versions.** `system_monitor` publishes `r5` (`git describe` of
  this repository) and `ROS`.
  - [ ] Add the motor node firmware version once the firmware reports it.

  Original request: publish `diagnostic_msgs/msg/DiagnosticStatus` on
  `/r5/versions` with transient-local durability (latched), once at startup. Put one entry per
  component in `values`, e.g. `{key: "r5", value: "<git describe>"}`,
  `{key: "motor node", value: "<firmware version>"}`.
- [x] **Computer temperature.** `system_monitor` reports the hottest thermal zone to
  `/diagnostics` (warning 75 °C, error 80 °C).
- [ ] **Motor driver temperature.** Needs a sensor. The IMU temperature (`/bno055/temp`) is shown already. Add the main
  computer and motor driver temperatures to `/diagnostics` with warning/error thresholds, so
  overheating appears as an alert. (A dedicated display can be added to the dashboard later.)
- [ ] **Motor current** (spec RB-7). The motor drivers report current; the motornode firmware needs
  to send it. Report each wheel's motor current. Proposal: an `effort` state
  interface in the ros2_control hardware plugin, carrying current in amps, so it appears in
  `/joint_states.effort`. The dashboard then needs a small change to show it.
- [x] **Diagnostics.** `r5_bringup/launch/monitor.launch.py` runs `diagnostic_aggregator` with
  `config/diagnostics.yaml`: `/Sensors/IMU`, `/Sensors/GPS`, `/Motors`, `/Computer`, and `/Other`
  for anything unmatched. The hardware plugin reports each motor node's link (`r5: Motor node
  front`/`rear`: link up, seconds since data, write failures).
  - [ ] On the robot, check whether the BNO055 and u-blox drivers publish diagnostics. If they do,
    remove `discard_stale` from their groups so a driver that goes quiet raises an alert. It's on
    now so an empty group doesn't show as a permanent Stale alert.

  Original request: run `diagnostic_aggregator` on the robot so `/diagnostics_agg` exists.
  - Group analysers by area, e.g. `/Sensors/IMU`, `/Sensors/GPS`, `/Motors/Left`, `/Computer`.
    The last part of the name is shown as the alert's source.
  - Add `diagnostic_updater` reporting to the motor node interface (link up, faults) if it doesn't
    already have it.
  - Until this runs, the dashboard shows an info alert, "No diagnostics from the robot".
- [x] **BNO055 calibration rate.** Now 1 Hz. `calib_status_frequency` is 0.1 Hz, so calibration can take
  10 s to appear on the Robot page. Consider 1 Hz.

## GNSS (ZED-F9 RTK)

- [ ] Add a ZED-F9 configuration to `r5_sensors` alongside `m9n.yaml`.
- [ ] Make sure the u-blox driver publishes `ublox_msgs/msg/NavPVT` on `/navpvt`. The dashboard
  uses it for satellite count, RTK float/fixed, and horizontal accuracy (`h_acc`, mm). The
  `ublox_msgs` package must be installed wherever rosbridge runs (it is, on the robot).
- [ ] Feed corrections from the base station to the receiver: the standard F9P setup, with the
  radio into the receiver's UART2.
- [ ] **RTK correction age** (spec RI-8). `ublox_msgs` 2.3's `NavPVT` has no `flags3` field; the
  correction age appears to be in `reserved1[0:2]`, which needs checking on the receiver. Publish `std_msgs/msg/Float32` on
  `/r5/gnss/correction_age_s`: seconds since the last correction, from NavPVT `flags3`
  (last correction age) or the RTCM input. The dashboard then needs a small change to show it,
  and should warn when it grows.
- [ ] Set `fix_mode`, rates and the RTK-fixed requirement for field recording (spec FM-4).

## Motion and safety

- [x] **Velocity arbitration.** `twist_mux` runs in front of the base controller
  (`r5_bringup/config/twist_mux.yaml`): `/mux/stop` 255, `/mux/joy` 100, `/mux/keyboard` 90,
  `/mux/test` 50, `/mux/nav` 10, all with a 0.5 s timeout, output `/cmd_vel`. `motor_test` uses
  `/mux/test`, and Stop holds zeros on `/mux/stop` for 2 s.

  Original request: add `twist_mux` (TwistStamped) in front of the base controller.
  Suggested priority, highest first: Xbox controller, dashboard motor test, Nav2.
  - Then set the dashboard `motor_test` node's `cmd_vel_topic` parameter to its mux input. Today it
    publishes straight to `/cmd_vel`, which Nav2's velocity smoother also publishes to.
  - The dashboard's software Stop button (spec NAV-4, in step 4) will use this too.
- [ ] **Hardware emergency stop**, independent of all software (spec NF-9).
- [ ] **Motor test limits.** The wheel facts match `diffbot_controllers.yaml` (0.36 m, 0.1 m and
  the joint names). Still to check on the real robot that the speed and time are safe. The `motor_test` node defaults to 0.06 m/s for 2 s, with hard caps of
  0.1 m/s and 5 s in code. Check these are safe on the real robot, and that `wheel_separation`
  (0.36 m), `wheel_radius` (0.1 m) and the wheel joint names match `diffbot_controllers.yaml`.
  The dashboard has the same wheel facts in `web/src/robot/robotConfig.ts`.
- [x] **Joint velocities.** The hardware plugin already reports each wheel's velocity in rad/s from
  the motor nodes. Confirm on the robot. The motor test and wheel speed readout need `/joint_states.velocity` from
  the real hardware plugin, in rad/s.

## Driving and recording fields (step 3)

- [x] **Xbox controller.** `r5_bringup/launch/command.launch.py` runs `joy` and `teleop_twist_joy`
  (`config/joystick.yaml`): hold LB to drive, RB for full speed, left stick forward, right stick
  turn, into `/mux/joy`. Y stays free for recording.
  - [ ] Pair the controller with the robot and check the button numbers with `ros2 topic echo /joy`.

  Original request: run `joy` and `teleop_twist_joy` on the robot, with the controller connected
  to the robot (USB dongle or Bluetooth), not to the laptop (spec RI-4 to RI-7).
  - Use a held enable (deadman) button, a speed cap, and TwistStamped output into `twist_mux`.
  - `joy` must publish `sensor_msgs/Joy` on `/joy`. The dashboard's `field_manager` reads it: holding
    button index 3 (Y on an Xbox controller under the Linux driver) for 1 s starts or stops a boundary
    recording. Change `joy_record_button` if Y is used for something else, or set it to −1 to turn
    this off. Make sure the record button isn't also the deadman or a drive button.
- [ ] **RTK for recording.** `field_manager` only keeps positions with an RTK fixed solution by
  default (launch argument `fix_requirement`; spray jobs use the same setting). It tells RTK fixed from float using `/navpvt`
  (`ublox_msgs/NavPVT`), so the u-blox driver must publish it and `ublox_msgs` must be installed
  where `field_manager` runs. Without `/navpvt` it can only see "corrections in use" from `/fix`.
- [ ] **Simulator GPS noise (optional, for testing).** The simulated GPS has 1.5 m noise, so a
  boundary recorded in Gazebo zigzags and crosses itself. To try recording realistically in
  simulation, lower the noise in `gps.urdf.xacro` (and the matching covariance in
  `sensor_covariance.py`) to RTK-like values, e.g. 0.02 m.

## Installation on the robot

- [x] Move this repository into the robot workspace: it's now `src/r5_dashboard` in the `r5`
  repository, so `git pull` and `rosdep install` cover it.
- [x] Start `r5_dashboard_server`'s `dashboard.launch.py` from `r5_bringup/r5.launch.py`
  (`dashboard:=false` turns it off, `fix_requirement` defaults to `any` and `sprayer_stub` to
  `true` for now).
- [ ] Copy the web app to the robot with `scripts/deploy_web.sh`, then rebuild
  `r5_dashboard_server` there.
- [ ] Set the dashboard password: `ros2 run r5_dashboard_server set_password`.
- [ ] Open the firewall for TCP 8443 (HTTPS) and 8080 (redirect). Keep 9090 (rosbridge) closed; it
  only listens on localhost anyway.
- [ ] Optional: serve on 443 so the address is just `https://<robot>`. This needs
  `CAP_NET_BIND_SERVICE` for the server (or a port redirect).
- [ ] **Clock.** Consider a DS3231 clock module, because the Pi has no real-time clock.
  Alert times, service dates and session expiry use the robot's clock. Make sure
  it is set at boot (NTP over the WiFi link, or GPS time via chrony), especially if the computer
  has no real-time clock.
- [ ] Back up `~/.r5` (stats, service log, password hash, certificate), and later fields.

## Spray jobs (step 4)

The dashboard's `job_manager` plans with Fields2Cover and drives with the robot's existing Nav2
setup. It was tested end to end in the simulator (rows, turns, pause/resume, refill trip, stop).
These robot-side items decide how well it works on the real robot:

- [ ] **Install Fields2Cover on the robot**: `ros-jazzy-fields2cover` (rosdep key `fields2cover`).
  `job_manager` uses its Python bindings, which the apt package includes.
- [ ] **Nav2 tuning for coverage** (`r5_navigation/params/nav2_params.yaml`):
  - `general_goal_checker.xy_goal_tolerance` is 1.5 m. `job_manager` extends each row by that much
    plus 0.5 m and switches the sprayer by position, so rows are still sprayed to the end. A
    tighter tolerance (e.g. 0.3 m) would let it use shorter lead-ins and lead-outs. Its
    `goal_tolerance_m` parameter must match.
  - `FollowPath.lookahead_dist` is 2.0 m, more than the 1.5 m row spacing. Turns will be cut
    short and rows joined late. Consider ~0.6–1.0 m for coverage, or a second controller
    plugin for coverage with its own settings (set `job_manager`'s `controller_id`).
  - `desired_linear_vel` 0.3 m/s caps the spray speed at 1.08 km/h. `job_manager` lowers it per
    job through `/speed_limit`. Raise it if spraying should go faster, and update
    `job_manager`'s `max_speed_kmh`.
- [ ] **`/fromLL`.** `job_manager` converts field coordinates to the `map` frame with
  navsat_transform's `/fromLL` service, which is available in the simulator. Make sure it is on the
  robot too, and that the `map` frame is the one Nav2 uses.
- [x] **Twist mux.** `job_manager` cancels all Nav2 goals on STOP and holds zero velocity on
  `/mux/stop`, the top-priority input (parameter `stop_cmd_vel_topic`).
- [ ] **Sprayer hardware** (spec 5.5). It will be an ESP32 with its own firmware, on USB like the
  motor nodes, so it needs its own udev name (for example `/dev/ttySprayer`). Subscribe to `/r5/sprayer/enable` (`std_msgs/Bool`,
  latched) and `/r5/sprayer/flow_setpoint` (`std_msgs/Float32`, L/min). The setpoint follows the
  robot's actual ground speed, so the rate per m² holds when it slows down. Then set
  `job_manager`'s `sprayer_is_placeholder` to false and its boom width, tank and pump limits.
- [ ] **Tank level sensor (later).** Liquid is estimated from flow × time today.
- [ ] **Turning radius.** `job_manager`'s `turn_radius_m` (1.0 m) shapes the planned turns. For a
  skid-steer robot that can pivot, smaller is possible, but the controller must be able to follow it.
- [ ] **Battery minimum.** Jobs are refused below `min_battery_percent` (30%) once `/battery_state`
  is published; until then the check is a warning.
- [ ] **Route home.** Trips home use `NavigateToPose` with the global costmap, which has no
  obstacles. Keep the route between fields and home clear (spec AS-9).

## Coming with later dashboard steps

These are listed now so the hardware and robot configuration can be planned. Details will be
filled in as each step is designed.

- [ ] **Mow and Spot Spray** (spec MW-1, SS-1): robot interfaces to be defined when those pages are designed.
