# Step 4: Collision monitor

By the end of this step Nav2's commands pass through
[`collision_monitor`](https://docs.nav2.org/configuration/packages/configuring-collision-monitor.html),
which slows the robot when something is near and stops it when something is close. This is the
safety layer. Step 5 only decides what the job does about it.

## How it works

`collision_monitor` takes each velocity command, checks the latest scan against fixed polygons
around the robot, and publishes the command unchanged, scaled down or zeroed. It doesn't use a
costmap or a plan, and it reacts within one command, so its behaviour is easy to predict.

It sits between the velocity smoother and `twist_mux`:

```
before:  velocity_smoother ── cmd_vel_smoothed ──► /mux/nav
after:   velocity_smoother ── /cmd_vel_smoothed ──► collision_monitor ──► /mux/nav
```

Because it's in series, it fails safe. If it crashes, nothing reaches `/mux/nav`, `twist_mux` drops
the input after 0.5 s, and `diff_drive_controller` stops the wheels.

## What it guards

Only Nav2. The Xbox controller (priority 100), keyboard (90), motor test (50) and dashboard Stop
(255) go straight to `twist_mux` as before. The reasons:

- A person holding LB can see what's in front of the robot, and sometimes needs to drive close to
  things: onto a trailer, up to a fill point, or past a gatepost.
- The motor test runs with the wheels off the ground.
- The Stop button must never depend on the lidar.

If you'd rather guard everything, move the monitor after `twist_mux` instead: remap `twist_mux`'s
`cmd_vel_out` to `/cmd_vel_mux`, and point the monitor from `/cmd_vel_mux` to `/cmd_vel`. Stop still
works, because zero passes through unchanged. Then the joystick can't get closer than the stop zone
to anything, and a lidar fault stops the joystick too.

## Stopping distance

Size the stop zone from how far R5 travels after something appears. At the 0.3 m/s maximum:

| | Time | Distance |
| --- | --- | --- |
| Scan: up to one turn of the lidar | 100 ms | 30 mm |
| Monitor: runs on each command from the smoother, at 10 Hz | up to 100 ms | 30 mm |
| Controller manager at 10 Hz, serial, ESP32 PID | about 150 ms | 45 mm |
| Braking: `diff_drive_controller` limits deceleration to 1.0 m/s² | 300 ms | 45 mm |
| **Total** | **about 0.65 s** | **about 0.15 m** |

The monitor's zero bypasses the velocity smoother's gentler 0.5 m/s², but not the controller's
1.0 m/s² limit. The tyres are slippery rubber, so on wet grass the robot can slide further than
this. Step 6 measures the real figure. Start with a stop zone 0.4 m beyond the front of the robot.

The slowdown zone gives the robot time before that. At 30 % (0.09 m/s), something that steps into
the stop zone from the slowdown zone is only 0.05 m from where the robot stops.

## Configuration

The polygons are in `base_footprint`, whose origin is midway between the axles. The robot is about
±0.31 m long and ±0.22 m wide without the boom (wheels at x ±0.203, radius 0.1). Measure your
robot, including the boom, and adjust:

| Zone | x (back to front) | y | Beyond the robot |
| --- | --- | --- | --- |
| Robot | −0.31 to 0.31 | ±0.22 | |
| Stop | −0.40 to 0.75 | ±0.35 | 0.44 m in front, 0.13 m at the sides, 0.09 m behind |
| Slowdown | −0.50 to 1.80 | ±0.60 | 1.5 m in front, 0.38 m at the sides |

The zones reach behind the robot as well as in front, because R5 turns on the spot
(`use_rotate_to_heading`) and its rear corners swing out.

Add to [`nav2_params.yaml`](../../src/r5_navigation/params/nav2_params.yaml):

```yaml
# Slows or stops Nav2's commands when the lidar sees something close. It sits between the
# velocity smoother and twist_mux, so only Nav2 is affected: the joystick, keyboard, motor test
# and dashboard Stop go straight to twist_mux. See doc/lidar/04-collision-monitor.md.
collision_monitor:
  ros__parameters:
    base_frame_id: "base_footprint"
    odom_frame_id: "odom"
    cmd_vel_in_topic: "/cmd_vel_smoothed"
    cmd_vel_out_topic: "/mux/nav"
    state_topic: "/collision_monitor_state"
    enable_stamped_cmd_vel: true
    transform_tolerance: 0.3
    # A scan older than this doesn't count. Then the robot stops (test this in step 6).
    source_timeout: 0.5
    # Moves the scan points by how far the robot has driven since the scan was taken.
    base_shift_correction: true
    # After a stop, zeros are published for this long, then nothing.
    stop_pub_timeout: 2.0
    polygons: ["stop_zone", "slowdown_zone"]
    stop_zone:
      type: "polygon"
      # 0.4 m beyond the front, 0.13 m beyond the sides and back. From the stopping distance.
      points: "[[0.75, 0.35], [0.75, -0.35], [-0.40, -0.35], [-0.40, 0.35]]"
      action_type: "stop"
      # Points needed to act. A person's leg at 1 m is about 8 points, so 4 ignores single
      # points from spray, rain and dust.
      min_points: 4
      visualize: true
      polygon_pub_topic: "polygon_stop"
      enabled: true
    slowdown_zone:
      type: "polygon"
      points: "[[1.80, 0.60], [1.80, -0.60], [-0.50, -0.60], [-0.50, 0.60]]"
      action_type: "slowdown"
      min_points: 4
      slowdown_ratio: 0.3
      visualize: true
      polygon_pub_topic: "polygon_slowdown"
      enabled: true
    observation_sources: ["scan"]
    scan:
      type: "scan"
      topic: "/scan"
      enabled: true
```

Check the parameter names against the Jazzy collision monitor docs. In Jazzy, `points` is a string
and the polygons use `min_points`. Older examples online use a list and `max_points`.

Also correct the comments that are now wrong: the header of `nav2_params.yaml` ("no obstacle
sensors"), the costmap comment ("The costmaps stay empty until the lidar has a driver") and the
reason given in `navigation.launch.py`, which now does route through `collision_monitor`.

Leave `use_collision_detection` and `use_cost_regulated_linear_velocity_scaling` off in
`FollowPath`. They read the costmap, which stays empty (see [step 7](07-driving-around-obstacles.md)).

### The "approach" alternative

Instead of fixed zones, `action_type: "approach"` projects the current command forward and scales
the speed so the robot is never closer than `time_before_collision` seconds to an obstacle. It
uses the robot's footprint polygon rather than zones. It's smoother, but its behaviour depends on
the command, which makes it harder to test. Start with stop and slowdown, and try approach once
step 6 has given you a baseline.

## Launch

In [`navigation.launch.py`](../../src/r5_navigation/launch/navigation.launch.py), send the smoother's
output to the monitor:

```python
        remappings=[("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "/cmd_vel_smoothed")],
```

Add the node:

```python
    collision_monitor_node = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        output="both",
        parameters=[params],
    )
```

Add `"collision_monitor"` to the lifecycle manager's `node_names`, the node to the returned list,
and `<exec_depend>nav2_collision_monitor</exec_depend>` to `src/r5_navigation/package.xml`.

## Check in simulation

```bash
colcon build --packages-select r5_navigation
ros2 launch r5_simulation simulation.launch.py
```

In RViz, add **Polygon** displays for `/polygon_stop` and `/polygon_slowdown`. Then send a goal past
the person, 8 m east:

```bash
ros2 action send_goal --feedback /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: base_footprint}, pose: {position: {x: 8.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

Watch the state:

```bash
ros2 topic echo /collision_monitor_state
```

The robot slows when the person enters the slowdown zone (`action_type: 2`), and stops with the
person about 0.4 m in front (`action_type: 1`). Fifteen seconds later the goal fails with a
progress checker error. That's expected: step 5 handles it.

## Check on the robot, without Nav2

Nav2 doesn't drive the real robot properly yet, but you can test the monitor without it, by driving
with teleop into the monitor's input instead of `twist_mux`:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p stamped:=true -p frame_id:=base_footprint -r cmd_vel:=/cmd_vel_smoothed
```

Set the speed to 0.3 m/s or less, then, with someone holding LB on the Xbox controller as an
override (holding LB with the sticks centred sends zero at a higher priority):

1. With the wheels off the ground, put a box in front. The wheels stop and the state shows
   `action_type: 1`.
2. On the ground, drive slowly at a cardboard box. The robot stops before it.
3. Unplug the lidar while driving at nothing. The robot must stop within about a second. If it
   doesn't, a stale source isn't stopping the robot in this Nav2 version. **Don't go further until
   you've solved that**, for example with a watchdog in front of the monitor.
