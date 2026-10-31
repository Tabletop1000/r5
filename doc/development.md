# Development

R5 runs ROS 2 Jazzy. The robot runs the control, sensor and localisation nodes. Development happens
on a laptop, where a Docker container provides ROS for building, testing, RViz and teleop, so ROS
doesn't need installing on the host.

## Repository layout

| Path | Contents |
| --- | --- |
| `src/r5_description` | URDF, ros2_control hardware plugin, controller config and base launch files. The package is named `r5`. |
| `src/r5_sensors` | Config and launch files for the BNO055 IMU and u-blox GPS |
| `src/r5_localisation` | Dual EKF localisation. See its [README](../src/r5_localisation/README.md). |
| `src/r5_bringup` | `r5.launch.py`, which starts everything on the robot |
| `docker/` | The laptop development image |

The motor node firmware is a separate ESP-IDF project, kept outside this repository in
`~/code/motornode`.

## Laptop setup

These steps were written for Fedora, but the container works the same on any Linux host with Docker.

### Build the image

Run this from the repository root. Repeat it whenever `docker/Dockerfile` changes.

```bash
docker build -t r5-laptop docker
```

### Run the container

```bash
xhost +si:localuser:root
docker run -it --rm --net=host --ipc=host \
  -e DISPLAY --device /dev/dri \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  -v ~/r5_ws/r5:/home/jdutra/git/r5 \
  --security-opt label=disable \
  r5-laptop bash
```

| Option | Purpose |
| --- | --- |
| `xhost +si:localuser:root` | Lets the container's root user open windows on your display. Needed once per login. |
| `--net=host`, `--ipc=host` | Puts the container on the laptop's network, so it can see the robot's topics |
| `-e DISPLAY`, `/tmp/.X11-unix`, `/dev/dri` | Display and GPU access for RViz |
| `-v ~/r5_ws/r5:/home/jdutra/git/r5` | Mounts the repository at the same path it has on the robot |
| `--security-opt label=disable` | Stops SELinux blocking access to the mounted folders on Fedora |
| `--rm` | Removes the container on exit. The mounted repository is kept, and anything installed with `apt` is lost, so add permanent packages to the Dockerfile. |

Keep the mount path as it is. The URDF contains absolute mesh paths from the robot, so RViz only
finds the meshes when the repository is at the same path.

The container runs as root, so `build/`, `install/` and `log/` end up owned by root. Git ignores
them, but deleting them from the host needs `sudo`.

### Connect to the robot

Fedora's firewall can block ROS discovery. Allow the robot's address, which trusts all traffic
from it:

```bash
sudo firewall-cmd --zone=trusted --add-source=<robot-ip> --permanent
sudo firewall-cmd --reload
```

With the robot running, check from the container:

```bash
ros2 topic list
```

### Check the connection

The container shares the laptop's network, so these tools see what the laptop sees. If the robot's
topics are missing, work down the list. Each step depends on the one before it.

1. Confirm which network the laptop is on, and find the robot. `r4` may not resolve, so use its IP
   address if it doesn't:

   ```bash
   ip -br addr
   ping -c 4 <robot-ip>
   ```

2. Check you can log in, and that the robot's nodes are running:

   ```bash
   ssh jdutra@<robot-ip>
   ```

3. Compare the ROS settings on both machines. `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION` must match,
   and `ROS_LOCALHOST_ONLY` must not be `1`:

   ```bash
   env | grep -E '^(ROS|RMW)_'
   ```

4. Test multicast, which ROS uses to find other machines. Run the first command on one machine and
   the second on the other, then swap them, because it can fail in only one direction:

   ```bash
   ros2 multicast receive
   ros2 multicast send
   ```

5. Watch for ROS traffic from the robot. Discovery uses UDP ports from 7400 upwards on domain 0:

   ```bash
   tcpdump -ni any host <robot-ip> and udp portrange 7400-7500
   ```

6. If topics appear but data is slow or patchy, measure the link. This needs `iperf3` on the robot
   too, where you run `iperf3 -s` first:

   ```bash
   iperf3 -c <robot-ip>
   ros2 topic hz /odometry/global
   ```

`ping` works but multicast doesn't usually means a firewall or a Wi-Fi network that blocks multicast.
`ros2 doctor --report` prints a summary of the network and ROS settings.

## Building

Inside the container:

```bash
colcon build
source install/setup.bash
```

A new container needs the `source` step again. The sensor drivers aren't in the image, so the sensor
launch files only run on the robot.

## Testing

The launch test uses mock hardware, so no robot is needed:

```bash
colcon test --packages-select r5 --ctest-args -E view_robot
colcon test-result --verbose
```

`-E view_robot` skips the test that opens RViz.

To run the base without hardware, with RViz:

```bash
ros2 launch r5 diffbot.launch.py use_mock_hardware:=true gui:=true
```

The controller doesn't publish the `odom` transform, because `r5_localisation` does, so an RViz
view fixed to `odom` won't show the robot moving in this mode.

To view the robot model:

```bash
ros2 launch r5 view_robot.launch.py
```

## Running on the robot

The robot is the host `r4`, with this repository at `~/git/r5`. Commit and push from the laptop,
then on the robot:

```bash
cd ~/git/r5
git pull
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
ros2 launch r5_bringup r5.launch.py
```

`rosdep` is only needed when dependencies change. The launch stops with a fatal error if a motor node
is missing, so check the devices if that happens:

```bash
ls -l /dev/ttyMotornode*
```

Those device names come from udev rules on the robot, in
`/etc/udev/rules.d/90-motornodes.rules`.

To read a motor node directly, stop the launch first, because only one program can use the port:

```bash
ros2 run r5 motornode_console /dev/ttyMotornodeFront
```

## Viewing the robot in RViz

With `r5.launch.py` running on the robot, run this in the container:

```bash
rviz2 -d install/r5/share/r5/rviz/r5.rviz
```

After changing `r5.rviz`, run `colcon build --packages-select r5` so the installed copy updates.

The camera follows `base_footprint`, so the robot stays in view and the grid moves past it. The fixed
frame is `odom`, which works without GPS. To see the GPS-corrected position instead, set
**Global Options > Fixed Frame** to `map`.

## Driving with teleop

The controller only accepts `TwistStamped`, so teleop needs these flags. Without them the robot
ignores commands and shows no error:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p stamped:=true -p frame_id:=base_footprint
```

Hold a key down to keep moving, because the controller stops after 0.5 s without a command. If you
run teleop from the laptop, the two clocks must agree to within that 0.5 s, or commands are rejected.

Keep the wheels off the ground for the first run after any change to the hardware interface.
