# Step 1: Hardware and driver

By the end of this step the D500 is on the robot's USB, it has a stable device name, and `/scan`
shows up in RViz with the right orientation.

## The sensor

The D500 kit is an LDROBOT STL-19P (the same family as the LD19) with a USB adapter board.

| | |
| --- | --- |
| Range | 0.03 to 12 m |
| Scan rate | 10 Hz, 5000 points/s, so about 500 points per turn |
| Angular resolution | 0.72° |
| Accuracy | ±10 mm to 0.5 m, ±20 mm to 2 m, ±30 mm to 12 m |
| Interface | One-way UART, 230400 baud 8N1. It streams as soon as it's powered, with no commands. |
| USB adapter | CP2102 (USB ID `10c4:ea60`) |
| Power | 5 V, 290 mA |
| Size | 38.6 × 38.6 × 33.5 mm |
| Operating temperature | −10 to 45 °C |
| Laser | Class 1 infrared |
| Angles | Zero is the front face of the sensor, and angles increase **clockwise**. ROS uses counter-clockwise, and the driver converts. |

The datasheet, 3D models and the Windows tool are linked from the
[Waveshare wiki](https://www.waveshare.com/wiki/D500_LiDAR_Kit). It doesn't give an IP rating, so
treat the sensor as unsealed.

## Mounting

### Height

The URDF has the lidar 25 mm above `base_link`, which puts the scan plane about 0.36 m above the
ground (`base_link` is 0.334 m up). Compare that with the obstacles you care about:

| Obstacle | Typical height | Seen at 0.36 m? |
| --- | --- | --- |
| Standing adult or child | 0.8 m and up | Yes, legs |
| Standing sheep, dog or calf | 0.4 to 0.9 m | Usually |
| Lying animal, sitting person | 0.2 to 0.4 m | Often not |
| Quad bike, trailer, tank | 0.5 m and up | Yes |
| Drawbar, hose, irrigation pipe | Under 0.2 m | No |

A lower plane sees more, but it hits more grass, and on bumpy ground it hits the ground itself.
When the robot pitches by θ, a level plane at height h meets the ground at h / tan θ:

| Pitch | Ground hit at h = 0.36 m | At h = 0.20 m |
| --- | --- | --- |
| 3° | 6.9 m | 3.8 m |
| 5° | 4.1 m | 2.3 m |
| 10° | 2.0 m | 1.1 m |

The stop zone in step 4 reaches about 0.6 m ahead of the lidar, and the slowdown zone about 1.7 m.
At 0.36 m, only a pitch over about 12° puts ground into the slowdown zone, so ground hits are
rarely a problem. **Start with the plane at about 0.36 m and level.** Move it lower only if step 6
shows you're missing obstacles that matter, and the grass allows it.

### Position and occlusion

Anything above the chassis at scan height casts a shadow, and the robot is blind behind it. On R5
the likely culprits are the GPS antenna (the URDF has it 30 mm above `base_link`, 0.1 m behind the
lidar), the sprayer tank and the boom.

- Put the lidar at the front, as the URDF does (0.1 m ahead of `base_link`). R5 doesn't reverse
  under Nav2 (`allow_reversing: false`), so the front and sides matter most.
- Raise the GPS antenna above the scan plane, or accept a blind sector behind it. The antenna wants
  to be highest anyway.
- Keep the boom and nozzles below the plane, or outside the robot box in step 2, and away from the
  lidar window, so spray doesn't land on it.
- Mark the sensor's front face (its zero direction) and point it forwards. If it has to point
  another way, set the yaw in the URDF in step 2.
- Leave the window clear of brackets. It's one horizontal slit around the whole housing.
- Measure the height of the window's centre above the ground once it's mounted. Step 2 needs it.

### Wiring

The USB adapter board plugs into the lidar's 4-pin ZH1.5 connector (Tx, PWM, GND, 5 V) and gives a
USB port. Leave PWM to the adapter, which runs the motor at its default 10 Hz.

- Use a short, good-quality cable, and strain-relieve both ends. Vibration loosens micro USB plugs.
- 290 mA is well within what a Pi USB port supplies. If the Pi already struggles with power (the
  motor nodes and GPS are on USB too), check `vcgencmd get_throttled` for under-voltage flags before
  and after adding the lidar.

## Device name

The motor nodes already have udev names (`/dev/ttyMotornodeFront` and so on), and the GPS is on USB
too. `/dev/ttyUSB0` changes with plug order, so give the lidar its own name.

Plug the lidar in on the robot and find it:

```bash
ls -l /dev/serial/by-id/
udevadm info -a -n /dev/ttyUSB0 | grep -E 'idVendor|idProduct|serial|KERNELS' | head
```

Many ESP32 boards also use a CP2102, so `10c4:ea60` alone may match a motor node too. Compare with
`/etc/udev/rules.d/90-motornodes.rules` and pick whatever is unique:

- **Serial number**, if the lidar's CP2102 has its own:

  ```
  # /etc/udev/rules.d/91-lidar.rules
  SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="<serial>", SYMLINK+="ttyLidar", GROUP="dialout", MODE="0660"
  ```

- **USB port**, if clone chips all report the same serial (often `0001`). Find the port path with
  `udevadm info -q property -n /dev/ttyUSB0 | grep ID_PATH=`, then label that port on the Pi:

  ```
  SUBSYSTEM=="tty", ENV{ID_PATH}=="<path>", SYMLINK+="ttyLidar", GROUP="dialout", MODE="0660"
  ```

Reload and check:

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/ttyLidar
```

If the device appears and then vanishes, check `journalctl -k` for `brltty`, which grabs some USB
serial adapters on Ubuntu. Remove it with `sudo apt remove brltty`.

Add the rule file to the same notes as the motor node rules, so a rebuilt SD card gets it too.

## Driver

Use LDROBOT's own C++ driver, [`ldlidar_stl_ros2`](https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2).
It supports the LD19/STL-19P, it's small, and it publishes `sensor_msgs/LaserScan`. It isn't
released as a Jazzy apt package, so build it from source in the workspace. The "ROS2" zip on the
Waveshare wiki is a copy of the same driver.

Add it as a pinned submodule, so the robot and the laptop build the same code:

```bash
cd ~/r5_ws/r5
git submodule add https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git src/third_party/ldlidar_stl_ros2
git -C src/third_party/ldlidar_stl_ros2 log -1 --format=%H   # note the commit you tested
```

On the robot, `git pull` doesn't fetch submodules, so after pulling run:

```bash
git submodule update --init
```

Add that line to "Running on the robot" in [development.md](../development.md).

Build it in the Docker container first. It doesn't need the hardware to build:

```bash
colcon build --packages-select ldlidar_stl_ros2
```

If it fails on Jazzy's compiler, the fix is usually a missing standard header. Patch it in a fork,
and point the submodule at the fork, rather than editing the submodule in place.

Don't use the driver's own launch files. They publish a static transform from `base_link` to
`base_laser`, which would fight `robot_state_publisher` once the lidar is in the URDF in step 2.

### Parameters

Create `src/r5_sensors/config/lidar.yaml`:

```yaml
# Waveshare D500 (LDROBOT STL-19P) on USB. See doc/lidar/01-hardware-and-driver.md.
# The driver publishes raw scans. r5_sensors/scan_filter.launch.py removes the robot's own body
# and republishes them on /scan.
ldlidar:
  ros__parameters:
    product_name: "LDLiDAR_LD19"   # the STL-19P speaks the LD19 protocol
    topic_name: "scan_raw"
    frame_id: "laser_frame"        # the link in lidar.urdf.xacro
    port_name: "/dev/ttyLidar"
    port_baudrate: 230400
    laser_scan_dir: true           # counter-clockwise, as ROS expects
    enable_angle_crop_func: false  # the box filter in step 2 is easier to reason about
    angle_crop_min: 0.0
    angle_crop_max: 0.0
```

Check these names against `launch/ld19.launch.py` in the commit you pinned. They have changed
between releases.

### Launch file

Create `src/r5_sensors/launch/lidar.launch.py`:

```python
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# The D500 lidar driver. Robot only: the simulation bridges Gazebo's scan to /scan_raw instead.
def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ldlidar_stl_ros2",
                executable="ldlidar_stl_ros2_node",
                name="ldlidar",
                output="both",
                parameters=[
                    PathJoinSubstitution([FindPackageShare("r5_sensors"), "config", "lidar.yaml"])
                ],
                respawn=True,
                respawn_delay=2.0,
            )
        ]
    )
```

`respawn` restarts the driver if it exits, for example after a USB glitch. The robot stops while
it's down, because the collision monitor in step 4 gets no scans.

Add `<exec_depend>ldlidar_stl_ros2</exec_depend>` to `src/r5_sensors/package.xml`. `rosdep` skips
it because it's in the workspace.

## Bench check

On the robot, with the lidar on its mount:

```bash
colcon build --packages-select ldlidar_stl_ros2 r5_sensors
source install/setup.bash
ros2 launch r5_sensors lidar.launch.py
```

In a second terminal:

```bash
ros2 topic hz /scan_raw                                  # about 10 Hz
ros2 topic echo --once /scan_raw --field header          # frame_id: laser_frame
ros2 topic echo --once /scan_raw | grep -E 'angle_(min|max|increment)|range_(min|max)'
```

Note the number of ranges in one scan, about 450 to 500. Step 3 uses it.

Then check the orientation. This catches the most common mistake, a scan that's mirrored or
rotated. In the container on the laptop:

```bash
rviz2
```

Set **Fixed Frame** to `laser_frame`, add a **LaserScan** display on `/scan_raw`, set **Reliability**
to Best Effort, and add an **Axes** display. Then:

1. Hold a box about 1 m in front of the sensor. It must appear on the red (+x) axis.
2. Move it to the robot's left. It must appear on the green (+y) axis.

If left and right are swapped, flip `laser_scan_dir`. If the box is rotated by a fixed angle, the
sensor isn't facing forwards. Set the yaw in the URDF in step 2, not in the driver.

Also note what you see close to the sensor with nothing in front of it: the GPS antenna, mast or
brackets. Step 2 removes these.
