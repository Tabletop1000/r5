# Development

R5 runs ROS 2 Jazzy. The robot runs the control, sensor and localisation nodes. Development happens
on a laptop, where a Docker container provides ROS for building, testing, RViz and teleop, so ROS
doesn't need installing on the host.

## Repository layout

| Path | Contents |
| --- | --- |
| `src/r5_description` | URDF, ros2_control hardware plugin, controller config and base launch files. The package is named `r5`. |
| `src/r5_sensors` | The BNO055 IMU driver (C++), and config and launch files for the IMU and u-blox GPS |
| `src/r5_localisation` | Dual EKF localisation. See its [README](../src/r5_localisation/README.md). |
| `src/r5_navigation` | Nav2 configuration and launch files |
| `src/r5_monitor` | Computer temperature, WiFi signal and versions, for the dashboard |
| `src/r5_dashboard` | The operator dashboard: web app, server and its ROS nodes. See its [README](../src/r5_dashboard/README.md). |
| `src/r5_simulation` | Gazebo world and launch file, for the laptop only |
| `src/r5_bringup` | `r5.launch.py`, which starts everything on the robot, plus the velocity command switch, Xbox controller and diagnostics |
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
mkdir -p ~/.r5-dev
docker run -it --rm --net=host --ipc=host \
  -e DISPLAY --device /dev/dri \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  -v ~/r5_ws/r5:/home/jdutra/git/r5 \
  -v ~/.r5-dev:/root/.r5 \
  --security-opt label=disable \
  r5-laptop bash
```

| Option | Purpose |
| --- | --- |
| `xhost +si:localuser:root` | Lets the container's root user open windows on your display. Needed once per login. |
| `--net=host`, `--ipc=host` | Puts the container on the laptop's network, so it can see the robot's topics |
| `-e DISPLAY`, `/tmp/.X11-unix`, `/dev/dri` | Display and GPU access for RViz |
| `-v ~/r5_ws/r5:/home/jdutra/git/r5` | Mounts the repository at the same path it has on the robot |
| `-v ~/.r5-dev:/root/.r5` | Keeps the simulated dashboard's password, fields and settings between containers |
| `--security-opt label=disable` | Stops SELinux blocking access to the mounted folders on Fedora |
| `--rm` | Removes the container on exit. The mounted repository is kept, and anything installed with `apt` is lost, so add permanent packages to the Dockerfile. |

Keep the mount path as it is. The URDF contains absolute mesh paths from the robot, so RViz only
finds the meshes when the repository is at the same path.

The container runs as root, so `build/`, `install/` and `log/` end up owned by root. Git ignores
them, but deleting them from the host needs `sudo`.

### Open another terminal in the same container
Get the ID of the running container with `docker ps`.
Open a second terminal and run 
`docker exec -it <container id>`

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

A new container needs the `source` step again. The sensor launch files only run on the robot: the
IMU driver builds anywhere but needs the IMU on `/dev/i2c-1`, and the GPS driver isn't in the image.

After rebuilding the Docker image, clear the old build first. Its files point at libraries from the
previous image, so `colcon build` fails with `No rule to make target '/opt/ros/jazzy/lib/...'`:

```bash
rm -rf build install log
```

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

To use RVIZ in the simulator:
```bash
rviz2 -d install/r5/share/r5/rviz/r5.rviz --ros-args -p use_sim_time:=true
```


## Running on the robot

The robot is the host `r4`, with this repository at `~/git/r5`. Commit and push from the laptop,
then on the robot:

```bash
cd ~/git/r5
git pull
rosdep install --from-paths src --ignore-src -r -y --skip-keys "ros_gz_sim ros_gz_bridge gz_ros2_control"
colcon build
source install/setup.bash
ros2 launch r5_bringup r5.launch.py
```

`rosdep` is only needed when dependencies change. `--skip-keys` keeps Gazebo off the robot, because
`r5_simulation` is only for the laptop. Run `sudo apt update && sudo apt upgrade` before
it, because ROS packages from different dates can crash with symbol errors. The launch stops with a fatal error if a motor node
is missing, so check the devices if that happens:

```bash
ls -l /dev/ttyMotornode*
```

Those device names come from udev rules on the robot, in
`/etc/udev/rules.d/90-motornodes.rules`.

`r5.launch.py` also starts the dashboard. Its arguments:

| Argument | Default | |
| --- | --- | --- |
| `dashboard` | `true` | Start the dashboard server and its nodes |
| `joystick` | `true` | Start the Xbox controller nodes |
| `fix_requirement` | `any` | Worst GPS fix the dashboard accepts. Change to `rtk_fixed` once the ZED-F9 is fitted. |
| `sprayer_stub` | `true` | Log sprayer commands, until the sprayer exists |

The dashboard needs some one-off setup on the robot, covered in its
[README](../src/r5_dashboard/README.md#install-on-the-robot): the web app, a password and the firewall.

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

## Velocity commands

Everything that moves the robot sends its commands to `twist_mux`, which passes the highest
priority recent one to `/cmd_vel`. Inputs that stop for 0.5 s are ignored. Don't publish to
`/cmd_vel` directly, because it would fight `twist_mux`.

| Input | Priority | Sent by |
| --- | --- | --- |
| `/mux/stop` | 255 | The dashboard's Stop button, held for 2 s |
| `/mux/joy` | 100 | Xbox controller, while LB is held |
| `/mux/keyboard` | 90 | `teleop_twist_keyboard` |
| `/mux/test` | 50 | The dashboard's motor test |
| `/mux/nav` | 10 | Nav2 |

The priorities are in `src/r5_bringup/config/twist_mux.yaml`.

## Driving with the Xbox controller

Connect the controller to the robot, not the laptop, with its USB dongle or over Bluetooth. Hold
**LB** and use the left stick for forward and back and the right stick to turn. Hold **RB** as well
for full speed. Releasing LB stops the robot. Holding **Y** for 1 s starts or stops a boundary
recording on the dashboard.

To pair over Bluetooth, put the controller in pairing mode, then on the robot:

```bash
bluetoothctl
# inside bluetoothctl:
scan on
pair <controller address>
trust <controller address>
connect <controller address>
```

The robot's user must be able to read the controller, so add it to the `input` group once and log
in again:

```bash
sudo usermod -aG input $USER
```

Check it's seen with `ros2 topic echo /joy` while moving a stick. The button numbers are in
`src/r5_bringup/config/joystick.yaml`.

## Driving with teleop

The controller only accepts `TwistStamped`, so teleop needs these flags. Without them the robot
ignores commands and shows no error:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p stamped:=true -p frame_id:=base_footprint -r cmd_vel:=/mux/keyboard
```

Hold a key down to keep moving, because `twist_mux` stops passing commands after 0.5 s. If you run
teleop from the laptop, the two clocks must agree to within that 0.5 s, or commands are rejected.

Keep the wheels off the ground for the first run after any change to the hardware interface.

## Simulation

`r5_simulation` runs the robot in Gazebo Harmonic on the laptop, with the same launch file as the
robot for everything above the hardware (`r5_bringup/launch/stack.launch.py`): localisation, Nav2,
`twist_mux`, monitoring and the dashboard. Gazebo stands in for the motors, IMU and GPS.

The container shares the laptop's network, so a simulation on the robot's `ROS_DOMAIN_ID` can send
`/cmd_vel` and goals to the real robot. Use a different domain in every terminal for the simulation:

```bash
export ROS_DOMAIN_ID=42
```

Build the dashboard's web app on the laptop, outside the container, the first time and whenever it
changes. It needs Node.js 22:

```bash
cd ~/r5_ws/r5/src/r5_dashboard/web
npm install
npm run build
```

Then, in the container, build, set the dashboard password once, and start the simulation:

```bash
colcon build
source install/setup.bash
ros2 run r5_dashboard_server set_password   # first time only
ros2 launch r5_simulation simulation.launch.py
```

The dashboard is at `https://localhost:8443`. Accept the certificate warning the first time. Add
`headless:=true` to run without the Gazebo window, or `dashboard:=false` to leave the dashboard out.
The robot starts at the world origin facing east, and Nav2 is ready when the log shows
`Managed nodes are active`.

Send a goal from a second terminal, in the same domain:

```bash
source install/setup.bash
ros2 action send_goal --feedback /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: base_footprint}, pose: {position: {x: 8.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

To watch it in RViz, use simulated time and set **Global Options > Fixed Frame** to `map`:

```bash
rviz2 -d install/r5/share/r5/rviz/r5.rviz --ros-args -p use_sim_time:=true
```

To drive with teleop, add `-p use_sim_time:=true` to the teleop flags above, or commands are rejected.
For an Xbox controller plugged into the laptop, start the simulation with `joystick:=true`, and run
the container with `--device /dev/input` so it can see the controller.

The simulation doesn't model everything:

- **Motors:** wheels always reach their commanded speed, so the sim won't show a stall.
- **Tyres:** the grip values in `wheel.urdf.xacro` are chosen so the robot can turn, not measured.
- **GPS:** noise is random on each fix, while a real receiver wanders slowly.
- **Obstacles:** the world is empty, and Nav2 has no obstacle sensing yet.
