# Step 5: Jobs and the dashboard

After step 4 the robot stops, but the job doesn't know why. Three things go wrong:

1. **The sprayer stays on.** During a row, `run_swath`'s `monitor()` sets the sprayer from the
   robot's position along the row, every 0.1 s. A stopped robot is still on the row, so the sprayer
   stays enabled, pointing at whatever made it stop. Flow follows measured speed, so it drops to
   zero, but the valve stays open and the nozzles drip.
2. **The job gives up.** After 15 s without movement, the controller's progress checker aborts the
   goal. `run_goal` retries once, then pauses with "Navigation failed", which is misleading.
3. **The operator isn't told** there's something in the way.

By the end of this step a job turns the sprayer off at once, waits for the way to clear, carries on
by itself, and pauses with an alert if the way stays blocked.

## Where the logic goes

Spray rows and turns go straight to the controller's `/follow_path` action, and transits go through
`/navigate_to_pose`. Both go through `run_goal` in
[`job_manager_node.py`](../../src/r5_dashboard/server/r5_dashboard_server/r5_dashboard_server/job_manager_node.py).
The behaviour tree only sees transits, so the waiting can't live there. It goes in `run_goal`, which
already handles pause and stop the same way: cancel the goal, wait, send it again.

Sending the same `FollowPath` goal again works, because pure pursuit picks up from the nearest
point on the path, and the sprayer goes by position along the row. Nothing is sprayed twice, as with
Resume after a pause.

Do this in the Python `job_manager`, so the field tests run with it. Step 4d of the C++ rewrite can
then port it with the rest, using the Python as the reference.

## Detecting an obstacle, and its clearing

`collision_monitor` publishes `/collision_monitor_state` only when its action changes, and it only
checks the scan when a command arrives. Once `job_manager` cancels the goal, commands stop, so the
monitor never reports that the way has cleared.

Nav2's `collision_detector`, from the same package, checks the same kind of polygons on a timer,
with no commands needed, and publishes which ones have points in them. `job_manager` uses it:

- **`stop_zone`** matches the monitor's stop zone. Points in it mean the robot has stopped, or is
  about to.
- **`clear_zone`** is a little larger. The way counts as clear once it has been empty for a few
  seconds. The gap between the two zones stops the robot starting and stopping repeatedly when
  something stands at the edge.

Add to [`nav2_params.yaml`](../../src/r5_navigation/params/nav2_params.yaml):

```yaml
# Tells job_manager whether something is in the way, independently of collision_monitor, which
# only checks while commands arrive. stop_zone must match collision_monitor's.
collision_detector:
  ros__parameters:
    frequency: 10.0
    base_frame_id: "base_footprint"
    odom_frame_id: "odom"
    transform_tolerance: 0.3
    source_timeout: 0.5
    base_shift_correction: true
    polygons: ["stop_zone", "clear_zone"]
    stop_zone:
      type: "polygon"
      points: "[[0.75, 0.35], [0.75, -0.35], [-0.40, -0.35], [-0.40, 0.35]]"  # = collision_monitor
      min_points: 4
      visualize: false
      enabled: true
    clear_zone:
      type: "polygon"
      points: "[[1.00, 0.45], [1.00, -0.45], [-0.50, -0.45], [-0.50, 0.45]]"
      min_points: 4
      visualize: true
      polygon_pub_topic: "polygon_clear"
      enabled: true
    observation_sources: ["scan"]
    scan:
      type: "scan"
      topic: "/scan"
      enabled: true
```

Add it to `navigation.launch.py` and the lifecycle manager like the monitor (executable
`collision_detector`). It publishes `nav2_msgs/msg/CollisionDetectorState` on
`/collision_detector_state`, with matching `polygons` and `detections` arrays.

A detector that has no fresh scan reports nothing detected, which would look like "clear". So
`job_manager` must count the way as clear only while the detector's state is fresh **and** the
Lidar alert from step 2 isn't active. If the lidar dies while the robot waits, the monitor still
stops the robot as soon as the goal is resent. But the job should wait, not start and stop again.

## Changes to `job_manager`

### Messages

Add the message to [`settings.py`](../../src/r5_dashboard/server/r5_dashboard_server/r5_dashboard_server/settings.py)
and [`settings.hpp`](../../src/r5_dashboard/server/r5_dashboard_nodes/include/r5_dashboard/settings.hpp),
beside `TOO_STEEP` / `kTooSteep`. They're kept twice until step 4d, and must match exactly:

```python
OBSTACLE = "Something has been in the way of the robot for"
```

### Parameters

| Parameter | Default | |
| --- | --- | --- |
| `obstacle_clear_s` | 3.0 | How long `clear_zone` must stay empty before the goal is resent |
| `obstacle_timeout_s` | 120.0 | How long to wait before pausing the job and raising an alert |

People and animals usually move within seconds. Two minutes is long enough for a curious sheep, and
short enough that an operator finds out about a stuck robot reasonably soon.

### Subscription

```python
from nav2_msgs.msg import CollisionDetectorState

        self.blocked = False          # points in stop_zone
        self.clear_zone_empty = False
        self.detector_t = 0.0         # time.monotonic() of the last state
        self.create_subscription(CollisionDetectorState, "/collision_detector_state",
                                 self.on_detector, 10, callback_group=group)

    def on_detector(self, msg: CollisionDetectorState) -> None:
        zones = dict(zip(msg.polygons, msg.detections))
        self.blocked = zones.get("stop_zone", False)
        self.clear_zone_empty = not zones.get("clear_zone", True)
        self.detector_t = time.monotonic()

    def way_is_clear(self) -> bool:
        fresh = time.monotonic() - self.detector_t < 0.5
        lidar_ok = not any(e.startswith("Lidar:") for e in self.alert_errors)
        return fresh and lidar_ok and self.clear_zone_empty
```

### In `run_goal`

Check for an obstacle in the polling loop **before** `monitor()`, so the sprayer is never switched
back on in the same cycle:

```python
            obstructed = False
            while not result_future.done():
                if interrupted():
                    handle.cancel_goal_async()
                    break
                if self.blocked:
                    obstructed = True
                    self.publish_sprayer(False, 0.0)
                    handle.cancel_goal_async()
                    break
                if monitor and monitor():
                    ...
```

After the result, before counting a failure:

```python
            if obstructed:
                self.wait_until_clear(state, action)
                continue  # resend the goal; not a failure
```

A goal that the progress checker aborts while something is in the stop zone shouldn't count as a
failure either. In the failure branch, check `self.blocked` too, and wait instead.

### Waiting

```python
    def wait_until_clear(self, state: int, action: str) -> None:
        """Sprayer off, wait for clear_zone to stay empty, or pause after obstacle_timeout_s."""
        self.publish_sprayer(False, 0.0)
        self.set_status(current_action="Waiting: something is in the way")
        self.get_logger().info("Obstacle in the stop zone; waiting")
        since = time.monotonic()
        clear_since = None
        while True:
            self.check_pause(state, action)  # Stop and Pause still work while waiting
            if self.way_is_clear():
                clear_since = clear_since or time.monotonic()
                if time.monotonic() - clear_since >= self.obstacle_clear_s:
                    self.get_logger().info("Way clear; carrying on")
                    return
            else:
                clear_since = None
            waited = time.monotonic() - since
            if waited >= self.obstacle_timeout_s:
                self.request_stop(
                    f"{OBSTACLE} {waited / 60:.0f} min, so the job paused. "
                    "Check what's there, then press Resume.")
                return  # run_goal's check_pause waits for Resume
            time.sleep(0.1)
```

After Resume, `run_goal` resends the goal. If the obstacle is still there, the robot waits again, so
Resume can't drive it into something.

`wait_until_clear` only runs within a job. Outside jobs, the collision monitor still stops Nav2
goals, which then fail as before.

### Tests

Add cases to [`test_job.py`](../../src/r5_dashboard/server/r5_dashboard_server/test/test_job.py) or a
new `test_obstacle.py`, with fake detector states and a fake clock:

- Blocked during a row: the sprayer goes off in the same cycle, and the goal is cancelled.
- Clear for less than `obstacle_clear_s`, then blocked again: the robot doesn't move.
- Clear for `obstacle_clear_s`: the same goal is resent, and the failure count hasn't changed.
- Blocked for `obstacle_timeout_s`: paused, with a message starting with `OBSTACLE`.
- Detector state stale, or the Lidar alert active: never counts as clear.
- Stop pressed while waiting: the job stops.

## Dashboard

### Alert

In [`robot_status_node.cpp`](../../src/r5_dashboard/server/r5_dashboard_nodes/src/robot_status_node.cpp),
add the message to `kJobWaitingAlerts`, so a job paused by an obstacle raises a warning on the Robot
page and in the status strip, as the slope and battery pauses do:

```cpp
const std::pair<const char *, const char *> kJobWaitingAlerts[] = {
  {settings::kLinkLost, "job:link_lost"},
  {settings::kBatteryLow, "job:battery_low"},
  {settings::kTooSteep, "job:too_steep"},
  {settings::kObstacle, "job:obstacle"},
};
```

Short waits only show as "Waiting: something is in the way" in the job's current action. They
don't raise an alert, because they're routine and clear themselves.

### Before a job starts

`job_manager`'s start checks already fail on any ERROR alert, so the Lidar alert from step 2 blocks
a job. Also add a check that says so explicitly, like the Sprayer check: "Lidar: 10 Hz", or FAIL
with "No lidar data". Then the operator sees the reason on the Spray page rather than only on the
Robot page.

## Behaviour tree

Update the comment in
[`navigate_to_pose.xml`](../../src/r5_navigation/behavior_trees/navigate_to_pose.xml). There are
still no recovery actions, but now it's because `job_manager` handles obstacles for every goal,
including the `/follow_path` goals that never reach the tree. Keep spin and back-up out: the lidar
makes them possible, but they move the robot off its planned path without anyone asking.

## Check in simulation

Start a small spray job in a field that the person model stands in, then:

1. The robot slows, stops with the person about 0.4 m away, and the sprayer goes off at once (watch
   `/r5/sprayer/enable`). The dashboard shows "Waiting: something is in the way".
2. Move the person out of the way (step 3). Three seconds later the robot carries on along the same
   row, and the sprayer comes back on at the right place.
3. Put the person back in the way and wait two minutes. The job pauses and the dashboard shows the
   alert. Press Resume with the person still there: the robot doesn't move. Move the person: it
   carries on.
4. Press Stop while it's waiting. The job stops.
5. Note the memory figure on the dashboard, and compare it with the one from before step 2.
