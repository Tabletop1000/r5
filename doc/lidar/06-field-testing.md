# Step 6: Field testing

The simulator shows the logic works. Only the field shows whether the zones are big enough, whether
grass, spray and sun cause false stops, and whether the robot really stops when the lidar fails.
Work through these in order. Each section assumes the ones before it passed.

## Safety for the tests

- Someone holds the Xbox controller for every test, ready to hold LB with the sticks centred. That
  sends zero at priority 100, above Nav2. The dashboard Stop button is the second override.
- **Use cardboard boxes and a soft dummy, not people or animals,** until the "Walk-in" tests below.
  For those, the person walks in from the side at a distance, never stands in front of a moving
  robot, and starts with the robot at its slowest.
- Start with the sprayer empty or filled with water.

## Record everything

Record a bag on the robot for every session. It lets you tune zones afterwards without going back
out:

```bash
ros2 bag record -o ~/bags/lidar_$(date +%F_%H%M) \
  /scan /scan_raw /tf /tf_static /odometry/local /cmd_vel /mux/nav /cmd_vel_smoothed \
  /collision_monitor_state /collision_detector_state /r5/job/status /r5/sprayer/enable /bno055/imu
```

A scan is about 4 KB, so the lidar adds about 150 MB an hour. Play a bag back on the laptop with
`ros2 bag play --clock` and RViz with `use_sim_time:=true`.

## 1. Stopping distance

Mark a line on the ground. Drive at it under Nav2 (or with teleop through the monitor, as in step 4)
with a box standing on it, at full speed (0.3 m/s). Measure from the box to the front of the robot
once it stops. Repeat:

| Surface | Runs | Gap must be |
| --- | --- | --- |
| Dry short grass | 5 | Over 0.15 m every time |
| Wet grass | 5 | Over 0.15 m every time |
| Downhill on your steepest field | 5 | Over 0.15 m every time |

If any gap is smaller, increase the front of the stop zone (in both the monitor and the detector)
by the shortfall plus 0.1 m. Expect downhill on wet grass to be the worst, with these tyres.

## 2. Failure modes

Each of these must stop the robot within about a second, with the robot driving under Nav2 at
nothing:

| Test | Expected |
| --- | --- |
| Unplug the lidar's USB | Stops. Lidar alert. Carries on once plugged back in (job) or not at all (plain goal). |
| `pkill -f ldlidar_stl_ros2_node` | Stops. The driver respawns. |
| `pkill -f scan_to_scan_filter_chain` | Stops, and stays stopped: the filter doesn't respawn. |
| `pkill -f collision_monitor` | Stops: `twist_mux` loses `/mux/nav`. |
| Hold a cloth over the window | **Probably doesn't stop.** See below. |
| Stop the sensor's motor (hold the turret lightly, briefly) | Stops: the driver stops publishing, or publishes garbage. Note which. |

The cloth test matters most. Anything touching the window is under 0.05 m away, or inside the body
filter's box, so it's removed, and the scan says nothing is there. Mud, or a film of spray, does the
same. "No points" looks just like "nothing in the way", and in an open field most ranges are empty
anyway, so counting valid points won't work.

Look for what a covered window does to `/scan_raw` instead: many ranges at exactly zero or under
0.05 m, low intensities, or the points on the robot's own body (the GPS mast, the boom) going
missing. The last one is the most reliable sign: those points are always there with a clear
window. If one of these works, add it to step 2's Lidar alert in `robot_status`. Until then, check
the window before each job.

## 3. False stops

Drive a full job, or at least several rows and turns, in each of these with nothing in the way.
Count the stops and slowdowns in `/collision_monitor_state`:

| Condition | Acceptable |
| --- | --- |
| Your shortest grass | None |
| Your longest grass | None, or decide the lidar must go higher |
| Spraying water | None. If spray causes stops, it's landing in the beam: move the nozzles or the lidar before raising `min_points`. |
| Light rain, if you can | Note the count |
| Low sun straight ahead, morning or evening | Note the count |
| Over the worst bump or ridge in the field | None in the stop zone. Slowdown is acceptable. |
| Along the headland next to the fence | None. Posts may show in the slowdown zone if the headland is narrow. |

For each false stop, find it in the bag and look at the points that caused it. Typical causes and
fixes:

- **A few scattered points close in:** spray or dust. Raise `min_points` slowly (4 → 6). A person's
  leg at 1.5 m is about 5 points, so don't go much higher.
- **A line of points across the front at 1.5 to 2 m:** the ground, on a pitch or a crest. Shorten
  the slowdown zone, or tilt the lidar up by 1 to 2°.
- **Points just outside the robot box:** the boom or something on the robot. Enlarge the box filter
  in that direction only.

## 4. Real obstacles

With the dummy, boxes and, last, a person walking in from the side:

| Test | Expected |
| --- | --- |
| Box in the middle of a row | Slows, stops, sprayer off, waits, alert after 2 min |
| Box removed while waiting | Carries on 3 s later, sprayer on at the right place |
| Box 0.5 m to one side of the path | Slows only, keeps going |
| Box during a headland turn, at a rear corner | Stops the turn |
| Dummy lying flat | **Not seen.** Record this as a known limit, and decide whether it's acceptable. |
| Dummy sitting | Note whether it's seen |
| Person walking across in front at 2 m | Slows, may stop, carries on once they've gone |
| Person walking towards the robot | Stops, and stays stopped while they stand there |
| Another vehicle or the quad bike | Stops |

## 5. Sign off

Before using it in real work, write down in this file or the robot's notes:

- The measured stopping gaps and the final zone sizes.
- What the lidar doesn't see at your mounting height: lying animals, low objects, and the shadow
  behind the GPS antenna.
- The false stop rate in grass, spray and sun.
- The firmware command timeout: done, or still open.

Then update the comments in `nav2_params.yaml` with the measured figures, so the next person to
change the zones knows where the numbers came from.
