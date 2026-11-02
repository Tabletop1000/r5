# r5_localisation

Estimates where R5 is by combining wheel encoders, the BNO055 IMU and GPS. It uses two Extended
Kalman Filters from [`robot_localization`](https://docs.ros.org/en/jazzy/p/robot_localization/)
and `navsat_transform_node`, and it's the position source for waypoint navigation.

The M9N GPS is a stand-in for testing, so expect position errors of a couple of metres. The design
carries over to RTK without structural changes.

## How it works

No single sensor is enough. Wheels and the gyro are smooth but drift, because wheels slip and small
errors add up. GPS doesn't drift, but it's noisy, slow at 1 Hz, and can jump. Two filters keep those
properties separate:

```text
 wheel odometry ──┬──────────────────────► local EKF ──► odom -> base_footprint
                  │                     ▲                /odometry/local
 BNO055 ──────────┼── yaw rate ─────────┘
    │             │
    │             └──────────────────────► global EKF ─► map -> odom
    ├──────────────── absolute yaw, ─────►     ▲   │     /odometry/global
    │                 yaw rate                 │   │
    │                                          │   │
    └── heading ──► navsat_transform ──────────┘   │
                     ▲        ▲   /odometry/gps    │
 M9N GPS ── fix ─────┘        └────────────────────┘
```

- **Local EKF** (`odom` frame) never sees GPS or absolute heading, so its output is continuous. Use
  it for anything that needs smooth motion, such as path following.
- **Global EKF** (`map` frame) adds GPS and absolute heading. It doesn't drift but can jump, so use
  it for working out where the robot is relative to a waypoint.
- **`navsat_transform`** turns each latitude and longitude into a position in the `map` frame. GPS
  can't go straight into an EKF, because the filter only understands positions in a local frame.

The two filters together give the transform chain `map` -> `odom` -> `base_footprint`. When GPS
corrects the position, `map` -> `odom` moves and `odom` -> `base_footprint` doesn't.

## What each filter uses

| Measurement | Source | Local EKF | Global EKF | Why |
| --- | --- | --- | --- | --- |
| Forward and sideways velocity | Wheels | Yes | Yes | The best short-term speed measurement |
| Yaw rate | Gyro | Yes | Yes | Skid-steer wheels slip when turning, so wheel yaw rate isn't used |
| Absolute yaw | BNO055 | No | Yes | Stops heading drifting, but magnetic disturbances can affect it |
| Position (x, y) | GPS | No | Yes | Corrects long-term drift |

Roll, pitch and height are ignored (`two_d_mode: true`), which suits flat, open ground.

## Running it

`robot_localization` is a new dependency, so install it on the robot first:

```bash
rosdep install --from-paths src --ignore-src -r -y --skip-keys "ros_gz_sim ros_gz_bridge gz_ros2_control"
```

Localisation starts with everything else:

```bash
ros2 launch r5_bringup r5.launch.py
```

To run it alone, while the base and sensors are already up:

```bash
ros2 launch r5_localisation localisation.launch.py
```

`r5_localisation` now owns `odom` -> `base_footprint`, because `enable_odom_tf` is off in the
controller. Running `diffbot.launch.py` by itself therefore publishes no `odom` transform.

The `map` origin is set by the first GPS fix after startup. The first fix can be slow, because the
GPS node may cold-reset the receiver when it starts, so wait for
`ros2 topic echo /fix --field status` to show `status: 0` or higher before moving.

| Topic | Contents |
| --- | --- |
| `/odometry/local` | Smooth pose in `odom` |
| `/odometry/global` | Drift-free pose in `map` |
| `/odometry/gps` | Each GPS fix as a position in `map` |
| `/gps/filtered` | The global estimate as latitude and longitude |

## Calibration

Do these in order, outside and away from buildings. Heading matters most: every GPS position is
projected into `map` using it, so a 10 degree heading error puts a position 10 m from the origin
about 1.7 m off.

### 1. Check the gyro direction

Turn the robot left, seen from above, and watch:

```bash
ros2 topic echo /bno055/imu --field angular_velocity.z
```

The value must be positive. If it isn't, change `placement_axis_remap` in
`r5_sensors/config/bno055_params.yaml`.

### 2. Calibrate the magnetometer

Move the robot through slow turns until the magnetometer value on `/bno055/calib_status` reaches 3.

### 3. Set `yaw_offset`

The BNO055 reports its own heading convention, plus magnetic rather than true north. `yaw_offset`
corrects both, so `magnetic_declination_radians` stays at 0.

1. Start the robot and wait for a fix.
2. Note the start of the track from `ros2 topic echo /odometry/gps --field pose.pose.position`.
3. Drive in a straight line for at least 20 m, then stop.
4. Note the end of the track, then call the angle of the line between the two points α:
   `α = atan2(end_y - start_y, end_x - start_x)`.
5. Read the EKF heading β, in degrees, from the last RPY value of:
   `ros2 run tf2_ros tf2_echo map base_footprint`.
6. Add `radians(α - β)` to `yaw_offset` in `params/ekf.yaml`, wrapping α - β to ±180 degrees first.
7. Rebuild, restart and repeat until α - β is close to 0. The heading is read once at startup, so
   the new value needs a restart.

Repeat the drive in a different direction. If the required offset changes with direction, motors or
steel near the IMU are bending the magnetic field. Move the IMU if you can.

## Limits

- **Heading:** absolute yaw depends on the magnetometer. If it's unreliable near the motors, turn
  off yaw in `imu0_config` for the global EKF and get heading from a dual-antenna GPS or from RTK
  track direction later.
- **`map` orientation:** `map` zero heading follows the BNO055, so it may not point east, even once
  calibrated. Convert latitude and longitude with `navsat_transform`'s `fromLL` service rather than
  assuming `map` x is east.
- **GPS antenna position:** `gps_link` is in the URDF at `base_link`, so update its offset once the
  antenna is mounted. This matters a lot more once RTK is fitted.
- **Noise values:** the covariances are starting values, and `diff_drive_controller` reports fixed
  ones. Expect to tune them from logged drives.

## Moving to RTK

Keep the same structure, then:

1. Point `/fix` in `launch/localisation.launch.py` at the new receiver if the topic
   changes.
2. Check the receiver reports realistic covariance, because the global EKF uses it to weight GPS.
3. Set the true antenna offset on `gps_link`.
4. Consider taking heading from a dual-antenna setup instead of the magnetometer.
