# R5 Dashboard

Operator interface for the R5 robot. See [requirements.md](requirements.md) and [spec.md](spec.md).

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
| `server/r5_dashboard_server/` | ROS 2 (ament_python) package: HTTPS server, login, WebSocket proxy, launch file |
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
   colcon build --packages-select r5_dashboard_server
   source install/setup.bash
   ```

   Link the whole repository, not just the package folder, so the package can find `web/dist`.

3. Set the dashboard password (at least 8 characters). This also logs out every device:

   ```bash
   ros2 run r5_dashboard_server set_password
   ```

4. Start rosbridge and the server:

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
| `data_dir` | `~/.r5` | Password hash, sessions and TLS certificate |

### Files in the data directory

| File | Contents |
| --- | --- |
| `dashboard_password.json` | scrypt hash of the password (never the password itself) |
| `sessions.json` | Hashes of login session tokens; sessions last 30 days |
| `tls/cert.pem`, `tls/key.pem` | Self-signed certificate, made on first start. Delete both to make a new one. |

All are readable only by the robot's user. To reset a forgotten password, run `set_password` again.

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
ros2 launch r5_dashboard_server dashboard.launch.py data_dir:=/data
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

## Topics the status strip reads

| Shown | Topic | Status |
| --- | --- | --- |
| Battery | `/battery_state` (`sensor_msgs/BatteryState`) | No publisher on the robot yet |
| Alerts | `/diagnostics_agg` (`diagnostic_msgs/DiagnosticArray`) | Needs `diagnostic_aggregator` running on the robot |
| Current action | — | Comes from the job manager, not built yet |

Items without data show "Not available". Names are in `web/src/ros/topics.ts`.
