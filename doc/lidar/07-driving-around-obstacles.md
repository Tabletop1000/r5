# Step 7 (optional): Driving around obstacles

Steps 1 to 6 stop the robot and wait. This step explains why that's the right default for R5, and
what it would take to steer around obstacles instead, if field use shows that waiting isn't enough.

## Why R5 stops rather than steering

- **On a row, a detour defeats the purpose.** A spray row has to be driven along its line. Driving
  round an obstacle leaves a gap, or sprays the edge of the next row twice. Rows go straight to
  `/follow_path` with no planner, so they can't detour anyway.
- **Detours are the one way the robot could reach a fence.** The planned path stays inside the field,
  and the lidar can't be trusted to see wire. Planning round an obstacle near the headland could
  take the robot through the boundary.
- **What it meets usually moves.** People and animals move off. Equipment that stays is better
  handled by an operator who knows why it's there.
- **It costs memory and CPU.** An obstacle layer, plus a keep-out filter and its map server, are more
  nodes, and more costmap updates on the Pi.

The case for steering is transit: going to the start of a field, home and back across open ground,
where the path is free and the planner already runs.

## What it would take, for transit only

### 1. Obstacles in the costmaps

Add an `ObstacleLayer` on `/scan` to the local and global costmaps in `nav2_params.yaml`, before the
inflation layer:

```yaml
      plugins: ["obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        observation_sources: scan
        scan:
          topic: /scan
          data_type: "LaserScan"
          clearing: true
          marking: true
          obstacle_max_range: 6.0   # short: further hits are more likely ground on a crest
          raytrace_max_range: 8.0
```

Keep `obstacle_max_range` well short of 12 m. Ground hits on bumps get likelier with range (step 1),
and the costmap remembers them, unlike the collision monitor.

Then `FollowPath`'s `use_collision_detection` and `use_cost_regulated_linear_velocity_scaling` have
something to read, and the 1 Hz replanning in `navigate_to_pose.xml` routes round what it sees.

### 2. A keep-out mask from the fields

The global costmap needs to know where the robot mustn't go, or a detour can leave the field. Nav2's
keep-out filter does this, from a mask image served by `map_server` and `costmap_filter_info_server`:

- Generate the mask from the saved fields: everything outside the field boundaries and the route home
  is lethal, and so are the fields' `keep_out` polygons. Reuse the field geometry in `geo.cpp` and
  `fields.cpp`, rather than writing another latitude/longitude conversion.
- Regenerate it when a field is saved or deleted, and when the map origin changes.
- Add `KeepoutFilter` to the global costmap's `filters`.

This is the largest part of the work. It also gives the planner the fields' known obstacles and
keep-out zones, which `spec.md` says v1 doesn't map.

### 3. Detour only in transit

Use a second behaviour tree, or a planner parameter, for transit goals only. Spray rows and turns
keep using `/follow_path` and `wait_until_clear`. `job_manager` should still wait first, and only ask
for a detour after the obstacle has stayed for a while, because a moving obstacle makes the planner
weave.

### 4. Test again

Repeat step 6 sections 3 and 4 for transits, plus:

- A box in the transit path: the robot goes round it, with at least the inflation radius to spare.
- A box near the boundary: the detour stays inside the keep-out mask.
- A bag played back from a crest: no phantom obstacles left in the costmap after the robot has
  passed.
