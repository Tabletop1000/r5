# Step 2: Robot model and launch

By the end of this step the lidar is part of the robot. Its frame is in the URDF, the robot's own
body is removed from the scan, both start with `r5.launch.py`, and the dashboard raises an alert if
scans stop arriving.

## URDF

[`lidar.urdf.xacro`](../../src/r5_description/description/urdf/sensors/lidar.urdf.xacro) was
written for a larger lidar (the 77 mm housing and the `rplidar_black` material). Update its
defaults to the D500:

```xml
    lidar_width:=0.0386
    lidar_height:=0.0335
    mass:=<weigh it>
    material_name:='lidar_black'
    update_rate:=10
    ray_count:=<the number of ranges from the bench check, about 450 to 500>
    min_range:='0.05'
    max_range:='12.0'
    range_resolution:='0.01'
    topic_name:='scan'
```

`min_range` is 0.05 m rather than the sensor's 0.03 m, because points that close are almost always
the robot itself or water on the window.

For the visual, replace `meshes/r5_visual/lidar.stl` with the LD19 STL from the Waveshare wiki. It's
in millimetres, so the existing `mesh_scale` of 0.001 still applies. Check its origin and axes in
`view_robot.launch.py`, and adjust `mesh_xyz_offset` and `mesh_rpy_offset` until it sits on the
frame with the window at z = 0.

Then set the mount in [`r5.urdf.xacro`](../../src/r5_description/description/urdf/r5.urdf.xacro)
from your measurements. `xyz_offset` is the window centre relative to `base_link`, so z is
(window height above ground − 0.334):

```xml
  <xacro:lidar_sensor
    prefix="$(arg prefix)"
    parent="base_link"
    frame_id="laser_frame"
    xyz_offset="<x> <y> <z>"
    rpy_offset="0 0 <yaw if it doesn't face forwards>"
    topic_name="scan"/>
```

Leave `topic_name` as `scan`. It's the Gazebo topic, which step 3 bridges to `/scan_raw`.

Check with:

```bash
colcon build --packages-select r5
colcon test --packages-select r5 --ctest-args -R urdf
ros2 launch r5 view_robot.launch.py
```

With the robot running, repeat the box test from step 1 in RViz with **Fixed Frame** set to
`base_footprint`. The box must be in front of the robot model, at the right distance from its nose.

## Removing the robot's own body

Anything on the robot inside the sensor's range shows up as an obstacle 5 to 30 cm away, inside the
stop zone. Without filtering, the robot would never move under Nav2.

[`laser_filters`](https://github.com/ros-perception/laser_filters)' box filter removes every point
inside a box in `base_footprint`, whatever caused it. It's more robust than cropping angles, because
it also catches the boom, the tank and anything added later. Note that this makes the box a blind
zone: make it as tight as the robot allows.

Add `<exec_depend>laser_filters</exec_depend>` to `src/r5_sensors/package.xml`, then create
`src/r5_sensors/config/scan_filters.yaml`:

```yaml
# Removes points on the robot itself from /scan_raw and publishes the rest on /scan. Runs on the
# robot and in simulation. The box is the robot's outline in base_footprint, including the boom,
# plus 3 cm. Anything inside it is invisible, so keep it tight.
scan_filter:
  ros__parameters:
    filter1:
      name: robot_body
      type: laser_filters/LaserScanBoxFilter
      params:
        box_frame: base_footprint
        min_x: -0.34   # measure: rearmost point of the robot - 0.03
        max_x: 0.34    # measure: front of the wheels or bumper + 0.03
        min_y: -0.25   # measure: widest point, including a folded boom
        max_y: 0.25
        min_z: -0.1
        max_z: 1.5
        invert: false
```

And `src/r5_sensors/launch/scan_filter.launch.py`:

```python
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# /scan_raw from the lidar (or Gazebo) to /scan, with the robot's own body removed.
def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="laser_filters",
                executable="scan_to_scan_filter_chain",
                name="scan_filter",
                output="both",
                parameters=[
                    PathJoinSubstitution(
                        [FindPackageShare("r5_sensors"), "config", "scan_filters.yaml"]
                    )
                ],
                remappings=[("scan", "/scan_raw"), ("scan_filtered", "/scan")],
            )
        ]
    )
```

If the booms fold out wider than the box, either make the box match the unfolded width, which
blinds the robot to things right at its sides, or keep the box tight and mount the lidar where the
booms are below the plane. The second is better.

## Launch

The driver is robot hardware, so it goes in
[`r5.launch.py`](../../src/r5_bringup/launch/r5.launch.py) beside the IMU and GPS:

```python
            include_launch("r5_sensors", "bno055_imu_node-launch.py"),
            include_launch("r5_sensors", "ublox_gps_node-launch.py"),
            include_launch("r5_sensors", "lidar.launch.py"),
```

The filter runs on the robot and in simulation, so it goes in
[`stack.launch.py`](../../src/r5_bringup/launch/stack.launch.py), inside `robot_nodes` so it gets
`use_sim_time`:

```python
            include_launch("r5_sensors", "scan_filter.launch.py"),
            include_launch("r5_navigation", "navigation.launch.py"),
```

Add `- **Lidar:** ...` to the simulation limits list in development.md once step 3 is done, and the
lidar row to the repository layout table (`src/r5_sensors` now has the lidar config and launch
files too).

## An alert when the lidar goes quiet

If the lidar stops, the collision monitor stops the robot (step 4 tests this), but nobody knows why.
The dashboard's alerts come from `robot_status`, which already raises alerts from timeouts
(`robot:no_diagnostics`). Add one for the lidar in
[`robot_status_node.cpp`](../../src/r5_dashboard/server/r5_dashboard_nodes/src/robot_status_node.cpp):

- Subscribe to `/scan` with `rclcpp::SensorDataQoS()`, and store only the arrival time. Don't
  process the ranges.
- In the existing 1 Hz timer, set an alert `sensor:lidar`, severity ERROR, source "Lidar",
  message "No lidar data for N s", when the last scan is older than 1 s.

Use `/scan` rather than `/scan_raw`, so the alert also catches a dead filter. Watching the topic,
rather than diagnostics, catches an unplugged cable, a crashed driver or a dead filter. It doesn't
catch a dirty window, which step 6 tests.

`job_manager` refuses to start a job while any ERROR alert is active (its "No errors" check), so
this also stops a job starting without a lidar. Add a unit test in
`test/test_stats.cpp` alongside the existing alert tests, if the timeout logic goes in `stats.cpp`.

Don't send `/scan` to the web app. 500 floats at 10 Hz over Wi-Fi to every dashboard is wasted
bandwidth. If you want a lidar test on the Robot page later (like Test IMU in `sensorTests.ts`), have
it subscribe with rosbridge's `throttle_rate` and report the rate and the nearest point.

## Check

On the robot:

```bash
colcon build --packages-select r5 r5_sensors r5_bringup r5_dashboard_nodes
ros2 launch r5_bringup r5.launch.py
```

- `ros2 topic hz /scan` shows about 10 Hz.
- In RViz, with **Fixed Frame** `base_footprint`, `/scan` has no points on or right next to the
  robot model, and `/scan_raw` does.
- Walk around the robot. Your legs appear on `/scan` all the way round, except in any shadow behind
  the GPS antenna. Note how wide that shadow is.
- Unplug the lidar. Within a couple of seconds the dashboard shows the Lidar alert. Plug it back in.
  The driver respawns and the alert clears.
- Note the memory figure on the dashboard against the one from before this step.
