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
| `docker/` | Laptop image with ROS 2 Jazzy, rosbridge and the server's Python dependencies |

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

1. Build the web app first (above). `colcon build` copies `web/dist` into the package, so rebuild
   the package whenever the web app changes.
2. Put this repository inside the robot's workspace, for example by linking it into `src`:

   ```bash
   ln -s ~/git/r5_dashboard ~/git/r5/src/r5_dashboard
   cd ~/git/r5
   rosdep install --from-paths src --ignore-src -r -y --skip-keys "ros_gz_sim ros_gz_bridge gz_ros2_control"
   colcon build --packages-select r5_dashboard_interfaces r5_dashboard_server
   source install/setup.bash
   ```

   Link the whole repository, not just the package folder, so the package can find `web/dist`.

3. Set the dashboard password (at least 8 characters). This also logs out every device:

   ```bash
   ros2 run r5_dashboard_server set_password
   ```

4. Start rosbridge, the server and the dashboard's robot-side nodes:

   ```bash
   ros2 launch r5_dashboard_server dashboard.launch.py
   ```

5. Browse to `https://<robot>:8443`. The browser warns about the certificate the first time;
   accept it once. `http://<robot>:8080` redirects there.

### Launch arguments

| Argument | Default | |
| --- | --- | --- |
| `https_port` | `8443` | Use 443 for a plain `https://<robot>` address; needs root or `CAP_NET_BIND_SERVICE` |
| `http_port` | `8080` | Redirects to HTTPS. `0` turns it off. |
| `rosbridge_port` | `9090` | Bound to 127.0.0.1 only |
| `data_dir` | `~/.r5` | Password hash, sessions, TLS certificate, usage counters |
| `use_sim_time` | `false` | Set `true` with the Gazebo simulation |
| `record_fix_requirement` | `rtk_fixed` | Worst GPS fix accepted when recording a boundary: `rtk_fixed`, `rtk_float` or `any`. Use `any` with the simulator or the development M9N receiver. |

### What the launch file starts

| Node | Does |
| --- | --- |
| `rosbridge_websocket`, `rosapi` | The ROS link, on localhost only |
| `server` | HTTPS, login, web app, WebSocket proxy |
| `robot_status` | Operating hours, distance, service schedule (`/r5/stats`); active alerts from `/diagnostics_agg` and its own checks (`/r5/alerts`); services to record a service and acknowledge an alert |
| `motor_test` | `/r5/motor_test` action: drives one side or both slowly for 2 s and checks the wheel speeds. Speed and time are fixed on the robot. |
| `field_manager` | Stores fields and the home point; records a boundary from `/fix` while the robot is driven. Recording can also be started and stopped by holding controller button 3 (Y on an Xbox controller) for a second (`joy_record_button`, `joy_hold_s` parameters). |

### Files in the data directory

| File | Contents |
| --- | --- |
| `dashboard_password.json` | scrypt hash of the password (never the password itself) |
| `sessions.json` | Hashes of login session tokens; sessions last 30 days |
| `tls/cert.pem`, `tls/key.pem` | Self-signed certificate, made on first start. Delete both to make a new one. |
| `stats.json` | Operating time, distance, last service. Saved every minute and on shutdown. |
| `service_log.json` | Every service recorded with "Service done" |
| `fields/<id>.geojson` | One file per field: boundary, keep-out zones and settings. Opens directly in QGIS. |
| `home.geojson` | The home point |
| `map.json` | Optional: map tile sources (see below) |

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

### Run the server against the simulation

Build the image once:

```bash
docker build -t r5-dashboard-dev docker
```

Start the simulation in the `r5-laptop` container as usual (see the `r5` repository's
`doc/development.md`; `headless:=true` works). Then, in a second terminal:

```bash
docker run -it --rm --net=host --ipc=host \
  --user $(id -u):$(id -g) -e HOME=/tmp \
  -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  -v ~/r5_ws/r5_dashboard:/home/jdutra/r5_ws/r5_dashboard \
  -v ~/.r5-dev:/data \
  --security-opt label=disable \
  r5-dashboard-dev bash

# inside the container
colcon --log-base /tmp/log build --base-paths server --build-base /tmp/build --install-base /tmp/install
source /tmp/install/setup.bash
ros2 run r5_dashboard_server set_password --data-dir /data   # first time only
ros2 launch r5_dashboard_server dashboard.launch.py data_dir:=/data use_sim_time:=true record_fix_requirement:=any
```

| Option | Purpose |
| --- | --- |
| `--user`, `-e HOME=/tmp` | Files created in the mounted folders belong to you, not root |
| `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` | The simulation container runs as root. Fast DDS shared memory doesn't work between different users, so topics are discovered but no data arrives without this. |

### Live-reloading web app

With the server running, start Vite, which passes `/api` and `/ws` through to it:

```bash
cd web
npm run dev     # http://localhost:5173
```

Set `R5_DASHBOARD_SERVER` to point Vite at another server, for example the real robot:
`R5_DASHBOARD_SERVER=https://r4:8443 npm run dev`.

### Server tests

```bash
docker run --rm -e PYTHONDONTWRITEBYTECODE=1 --user $(id -u):$(id -g) \
  -v ~/r5_ws/r5_dashboard:/w --security-opt label=disable r5-dashboard-dev \
  bash -c "cd /w/server/r5_dashboard_server && python3 -m pytest -p no:cacheprovider test"
```

### Launching the robot sim
```bash
docker run -it --rm --net=host --ipc=host -v ~/r5_ws/r5:/home/jdutra/git/r5 --security-opt label=disable r5-laptop bash
# inside:
source install/setup.bash
ros2 launch r5_simulation simulation.launch.py headless:=true
```

## ROS interfaces the dashboard uses

Topic names and message types are in `web/src/ros/topics.ts`. Ones without a publisher on the
robot yet show "Not available"; [TODO_ROBOT.md](TODO_ROBOT.md) lists them.

| Shown | Interface | Source |
| --- | --- | --- |
| Usage, service schedule | `/r5/stats` | `robot_status` |
| Alerts (strip and Robot page) | `/r5/alerts`, `/r5/alerts/acknowledge` | `robot_status`, which reads `/diagnostics_agg` |
| Battery | `/battery_state` | Not published yet |
| WiFi signal | `/r5/wifi_signal_dbm` | Not published yet |
| Round trip | `/rosapi/get_time` | rosapi |
| Versions | `/r5/versions` | Not published yet |
| GPS | `/fix`, `/navpvt` | u-blox driver (`/navpvt` only on the real receiver) |
| IMU | `/bno055/imu`, `/bno055/calib_status`, `/bno055/temp` | BNO055 driver (the simulator has only `imu`) |
| Wheels | `/joint_states` | joint_state_broadcaster |
| Motor test | `/r5/motor_test` | `motor_test` |
| Fields, home | `/r5/fields/*`, `/r5/home/*` services, `/r5/fields/revision` | `field_manager` |
| Boundary recording | `/r5/recording/start`, `stop`, `clear`, `/r5/recording/status` | `field_manager` (reads `/fix`, `/navpvt`, `/joy`) |
| Current action | — | Job manager, not built yet |
