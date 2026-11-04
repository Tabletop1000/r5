# R5 Dashboard

Operator interface for the R5 robot. See [requirements.md](requirements.md) and [spec.md](spec.md).
Work still needed on the robot itself is tracked in [TODO_ROBOT.md](TODO_ROBOT.md).

The browser app talks to ROS through rosbridge. A small server on the robot sits in front of
rosbridge: it serves the app over HTTPS, checks the password, and only then passes the browser's
WebSocket through. rosbridge itself listens on localhost only.

```text
browser ──HTTPS / WSS──► r5_dashboard_server (:8443) ──ws://127.0.0.1:9090──► rosbridge ──► ROS 2
```

## Repository layout

| Path | Contents |
| --- | --- |
| `web/` | The browser app: TypeScript, React, Vite, roslib |
| `server/r5_dashboard_server/` | ROS 2 (ament_python) package: HTTPS server, login, WebSocket proxy, the `robot_status`, `motor_test` and `field_manager` nodes, launch file |
| `server/r5_dashboard_interfaces/` | ROS 2 messages, services and actions between the dashboard and the robot |

## Build the web app

Needs Node.js 22.

```bash
cd web
npm install
npm test          # unit tests
npm run build     # writes web/dist
```

`web/.npmrc` sets `legacy-peer-deps`, because npm 10.9 crashes resolving Vite's optional peer
dependencies.

## Install on the robot

The dashboard is part of the `r5` repository, so the robot gets it with `git pull`, and its
dependencies with the usual `rosdep` command (see the `r5` repository's `doc/development.md`).
`r5_bringup/r5.launch.py` starts it. These steps are needed once, or when the web app changes:

1. Build the web app on the laptop and copy it to the robot, which has no Node.js 22. From this
   folder:

   ```bash
   scripts/deploy_web.sh jdutra@r4
   ```

   Then rebuild the package on the robot, because `colcon build` copies `web/dist` into it:

   ```bash
   cd ~/git/r5
   colcon build --packages-select r5_dashboard_server
   ```

2. Set the dashboard password (at least 8 characters). This also logs out every device:

   ```bash
   ros2 run r5_dashboard_server set_password
   ```

3. If the robot's firewall is on (`sudo ufw status`), open the dashboard ports. Keep 9090
   (rosbridge) closed; it only listens on localhost anyway:

   ```bash
   sudo ufw allow 8443/tcp
   sudo ufw allow 8080/tcp
   ```

4. Check the robot's clock is set at boot, because alert times and session expiry use it:
   `timedatectl` should show `System clock synchronized: yes`. The Raspberry Pi has no battery-backed
   clock, so without internet it starts at the last shutdown time. A DS3231 clock module fixes that.

5. Back up the data directory now and then, from the laptop:

   ```bash
   rsync -a jdutra@r4:.r5/ ~/r5-backup/
   ```

6. Start the robot with `ros2 launch r5_bringup r5.launch.py`, or run the dashboard alone with
   `ros2 launch r5_dashboard_server dashboard.launch.py`.

7. Browse to `https://<robot>:8443`. The browser warns about the certificate the first time;
   accept it once. `http://<robot>:8080` redirects there.

### Launch arguments

| Argument | Default | |
| --- | --- | --- |
| `https_port` | `8443` | Use 443 for a plain `https://<robot>` address; needs root or `CAP_NET_BIND_SERVICE` |
| `http_port` | `8080` | Redirects to HTTPS. `0` turns it off. |
| `rosbridge_port` | `9090` | Bound to 127.0.0.1 only |
| `data_dir` | `~/.r5` | Password hash, sessions, TLS certificate, usage counters |
| `use_sim_time` | `false` | Set `true` with the Gazebo simulation |
| `fix_requirement` | `rtk_fixed` | Worst GPS fix accepted for recording a boundary and for running a spray job: `rtk_fixed`, `rtk_float` or `any`. Use `any` with the simulator or the development M9N receiver. |
| `sprayer_stub` | `false` | Start `sprayer_stub`, which logs sprayer commands. Use `true` until the sprayer exists. |

### What the launch file starts

| Node | Does |
| --- | --- |
| `rosbridge_websocket`, `rosapi` | The ROS link, on localhost only |
| `server` | HTTPS, login, web app, WebSocket proxy |
| `robot_status` | Operating hours, distance, service schedule (`/r5/stats`); active alerts from `/diagnostics_agg` and its own checks (`/r5/alerts`); services to record a service and acknowledge an alert |
| `motor_test` | `/r5/motor_test` action: drives one side or both slowly for 2 s and checks the wheel speeds. Speed and time are fixed on the robot. |
| `job_manager` | Plans area spray jobs with Fields2Cover and runs them with Nav2: `FollowPath` for rows and turns, `NavigateToPose` to the start and home. Switches the sprayer by position along each row, estimates the tank, drives home to refill, and handles pause, resume and the software stop (`/r5/stop`), which holds zero velocity on `twist_mux`'s top-priority input for 2 s. Its parameters (boom width, tank, pump, speeds, turn radius, Nav2 goal tolerance, checks) are listed at the top of `job_manager_node.py`. |
| `sprayer_stub` | Optional stand-in for the sprayer: logs `/r5/sprayer/enable` and `/r5/sprayer/flow_setpoint` |
| `field_manager` | Stores fields and the home point; records a boundary from `/fix` while the robot is driven. Recording can also be started and stopped by holding controller button 3 (Y on an Xbox controller) for a second (`joy_record_button`, `joy_hold_s` parameters). |

### Files in the data directory

| File | Contents |
| --- | --- |
| `dashboard_password.json` | scrypt hash of the password (never the password itself) |
| `sessions.json` | Hashes of login session tokens; sessions last 30 days |
| `tls/cert.pem`, `tls/key.pem` | Self-signed certificate, made on first start. Delete both to make a new one. |
| `stats.json` | Operating time, distance, last service. Saved every minute and on shutdown. |
| `service_log.json` | Every service recorded with "Service done" |
| `job_settings.json` | Last-used spray settings for each field |
| `fields/<id>.geojson` | One file per field: boundary, keep-out zones and settings. Opens directly in QGIS. |
| `home.geojson` | The home point |
| `map.json` | Optional: map tile sources (see below) |

### Importing and exporting fields

On the Manage Fields page, **Export** saves a field as GeoJSON, the same format the robot stores,
which opens in QGIS. **Import file** (under Add new field) reads a GeoJSON or KML polygon, for
example from QGIS or Google Earth, and opens it in the editor to check and save:

- Coordinates must be latitude/longitude (WGS 84, EPSG:4326). Files in other coordinate systems
  are refused with a message saying so.
- The largest polygon becomes the boundary. Holes in it, and smaller polygons, become keep-out
  zones. The name comes from the file's `name` property or the file name.

### When the WiFi link drops

Everything runs on the robot, so jobs and recordings carry on. The dashboard pings the robot every
few seconds. After 8 s with no reply it shows "Disconnected — robot continues on its own", even if
the browser hasn't noticed the link has gone, and keeps trying to reconnect. When the link is back
it reconnects by itself and shows the current state.

### Map tiles

The map uses the OpenStreetMap street map by default, which needs internet on the device viewing
the dashboard. Without internet the map shows a plain 10/50/100 m grid, and fields, the robot and
the home point still appear on it.

To use other tiles, such as open aerial imagery for your area, create `map.json` in the data
directory. The format is documented in `server/r5_dashboard_server/r5_dashboard_server/mapconfig.py`.
Only use imagery whose licence allows it (spec NF-12). With more than one layer, a selector appears on
the map. Restart the server after changing it.

All are readable only by the robot's user. To reset a forgotten password, run `set_password` again,
or change it from the Robot page.

## Develop on the laptop

The `r5-laptop` image has everything the dashboard needs, and the simulation starts the dashboard
with the robot. See the Simulation section of the `r5` repository's `doc/development.md`: build the
web app, then `ros2 launch r5_simulation simulation.launch.py`, and browse to
`https://localhost:8443`.

### Live-reloading web app

With the simulation running, start Vite on the laptop, which passes `/api` and `/ws` through to the
dashboard server:

```bash
cd web
npm run dev     # http://localhost:5173
```

Set `R5_DASHBOARD_SERVER` to point Vite at another server, for example the real robot:
`R5_DASHBOARD_SERVER=https://r4:8443 npm run dev`.

### Server tests

In the `r5-laptop` container:

```bash
cd src/r5_dashboard/server/r5_dashboard_server
python3 -m pytest -p no:cacheprovider test
```

## ROS interfaces the dashboard uses

Topic names and message types are in `web/src/ros/topics.ts`. Ones without a publisher on the
robot yet show "Not available"; [TODO_ROBOT.md](TODO_ROBOT.md) lists them.

| Shown | Interface | Source |
| --- | --- | --- |
| Usage, service schedule | `/r5/stats` | `robot_status` |
| Alerts (strip and Robot page) | `/r5/alerts`, `/r5/alerts/acknowledge` | `robot_status`, which reads `/diagnostics_agg` |
| Battery | `/battery_state` | Not published yet |
| WiFi signal | `/r5/wifi_signal_dbm` | `r5_monitor` |
| Round trip | `/rosapi/get_time` | rosapi |
| Versions | `/r5/versions` | `r5_monitor` (`r5` and ROS; motor node firmware not yet) |
| GPS | `/fix`, `/navpvt` | u-blox driver (`/navpvt` only on the real receiver) |
| IMU | `/bno055/imu`, `/bno055/calib_status`, `/bno055/temp` | BNO055 driver (the simulator has only `imu`) |
| Wheels | `/joint_states` | joint_state_broadcaster |
| Motor test | `/r5/motor_test` | `motor_test` |
| Fields, home | `/r5/fields/*`, `/r5/home/*` services, `/r5/fields/revision` | `field_manager` |
| Spray jobs | `/r5/job/defaults`, `plan`, `start`, `pause`, `resume` services; `/r5/job/status`, `/r5/job/plan` topics; `/r5/stop` | `job_manager` (uses Nav2 `/follow_path`, `/navigate_to_pose`, `/speed_limit` and robot_localization `/fromLL`) |
| Sprayer | `/r5/sprayer/enable`, `/r5/sprayer/flow_setpoint` | Published by `job_manager`; placeholder until the sprayer exists |
| Boundary recording | `/r5/recording/start`, `stop`, `clear`, `/r5/recording/status` | `field_manager` (reads `/fix`, `/navpvt`, `/joy`) |
| Current action, STOP | `/r5/job/status`, `/r5/stop`, `/odometry/local` | `job_manager` |
