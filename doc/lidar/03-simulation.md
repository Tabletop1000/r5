# Step 3: Simulation

By the end of this step the simulated robot publishes `/scan_raw` from Gazebo, the scan filter
runs as it does on the robot, and the world has obstacles to drive at. Steps 4 and 5 are much
quicker to get right here than in a field.

## World

The lidar macro already defines a `gpu_lidar` sensor, but Gazebo only runs sensors that need
rendering when the Sensors system is loaded. Add it to
[`flat.sdf`](../../src/r5_simulation/worlds/flat.sdf) beside the IMU and NavSat systems:

```xml
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
```

The container already has `/dev/dri` for RViz, so the GPU lidar renders. With `headless:=true`,
the server has no window to render into. Add `--headless-rendering` to `gz_args` in
`simulation.launch.py` when headless:

```python
PythonExpression(["'-s --headless-rendering ' if '", headless, "' == 'true' else ''"]),
```

## Obstacles

Add a few models to the world, each a little apart along the robot's +x axis (east, the way it
faces at the start), so a straight goal drives into them. Give each a `<collision>` and a `<visual>`
with the same geometry. The lidar sees the visual and the physics uses the collision.

| Model | Geometry | What it tests |
| --- | --- | --- |
| `person` | Cylinder, r 0.2 m, h 1.7 m, at x = 6 | Stop and wait |
| `dog` | Box 0.6 × 0.25 × 0.5 m, at x = 12, y = 0.5 | Off-centre, inside the slowdown but not the stop zone |
| `lying_animal` | Box 0.8 × 0.4 × 0.3 m, at x = 18 | Below the scan plane: the sim should show it's **not** seen |
| `post` | Cylinder, r 0.05 m, h 1.2 m, at x = 3, y = 1.5 | A fence post beside the path. It must not stop the robot. |

For example:

```xml
    <model name="person">
      <static>true</static>
      <pose>6 0 0.85 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><cylinder><radius>0.2</radius><length>1.7</length></cylinder></geometry>
        </collision>
        <visual name="visual">
          <geometry><cylinder><radius>0.2</radius><length>1.7</length></cylinder></geometry>
        </visual>
      </link>
    </model>
```

To test waiting and resuming, move the person out of the way while the robot waits, in the Gazebo
window (translate tool) or from a terminal:

```bash
gz service -s /world/flat/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean \
  --timeout 1000 --req 'name: "person", position: {x: 6, y: 3, z: 0.85}'
```

Keep these obstacles in a second world, `obstacles.sdf`, and add a `world` launch argument, if you'd
rather keep `flat.sdf` empty for testing Nav2 on its own.

## Bridge

Add to [`gz_bridge.yaml`](../../src/r5_simulation/config/gz_bridge.yaml):

```yaml
- ros_topic_name: /scan_raw
  gz_topic_name: /scan
  ros_type_name: sensor_msgs/msg/LaserScan
  gz_type_name: gz.msgs.LaserScan
  direction: GZ_TO_ROS
```

`LaserScan` has no covariance, so unlike the IMU and GPS it doesn't need to go through
`sensor_covariance.py`. It goes to `/scan_raw`, so `scan_filter` from step 2 runs exactly as on the
robot.

Gazebo's lidar has no noise unless you add some. Add this inside `<ray>` in the macro, so the
filter and `min_points` get something realistic to deal with:

```xml
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.015</stddev>
          </noise>
```

## Check

```bash
colcon build --packages-select r5 r5_simulation r5_sensors r5_bringup
ros2 launch r5_simulation simulation.launch.py
```

- `ros2 topic hz /scan` shows 10 Hz.
- `ros2 topic echo --once /scan --field header.frame_id` shows `laser_frame`.
- In RViz (`use_sim_time:=true`, **Fixed Frame** `base_footprint`), the person, dog and post show up
  as arcs of points in the right places, and the lying animal doesn't.
- The scan has no points on the robot model. If Gazebo's lidar hits the robot's own visuals, the
  box filter should remove them. If it doesn't, the box is too small.

At this point a goal 8 m east still drives straight through the person. That's step 4.

Add to the simulation limits in development.md:

- **Lidar:** a perfect plane with Gaussian noise. The sim doesn't model grass, spray, sun or pitch
  on bumps, so it can't tell you about false stops.
