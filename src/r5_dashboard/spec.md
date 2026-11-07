# R5 Dashboard — Specification

Status: **Draft 0.5** (2026-11-04). Derived from [requirements.md](requirements.md) and
[resource/wireframe.png](resource/wireframe.png). Items marked **[Proposed]** go beyond the requirements
and need sign-off. Open questions are collected in [section 10](#10-open-questions).

## 1. Purpose and scope

The R5 Dashboard is the operator interface for the R5 agricultural robot, which works properties of
1 to 5 acres (about 0.4 to 2 ha). It lets an operator manage fields, start area-spray jobs and check
the robot's health.

| In scope for v1 | Out of scope for v1 |
| --- | --- |
| Field management (create, edit, view, delete) | Spot Spray page (placeholder only) |
| Area Spray job setup and launch | Mow page (placeholder only) |
| Robot status and diagnostics page | Native Android and iOS apps |
| Laptop browsers | Multiple robots, multiple users with accounts |
| | Spraying the headland |
| | Design for outdoor field use (sunlight, gloves) |

### 1.1 Users and environment

- Users are comfortable with technology but are not engineers. The UI uses plain language and
  farming units (litres, hectares, km/h), not ROS terms.
- The main use is from an office or similar, over the robot's long-range WiFi link.
- v1 targets laptops. The layout and code must not stop a later move to tablets and phones (see 7.2).

## 2. System context

```text
 ┌──────────── Laptop / tablet / phone ────────────┐
 │  Browser: R5 Dashboard (static web app)          │
 └──────────────┬───────────────────────────────────┘
                │ HTTPS + secure WebSocket, after password login
      long-range WiFi
                │
 ┌──────────────┴──────────── Robot (ROS 2 Jazzy) ──────────────────────────┐
 │  r5_dashboard_server  ── login, serves web app, proxies the WebSocket,     │
 │                          owns field storage, stats, job manager            │
 │  rosbridge_server     ── WebSocket bridge, listening on localhost only     │
 │  Nav2 + coverage server (Fields2Cover)                                     │
 │  r5_localisation (EKFs + navsat_transform)   r5_sensors (IMU, GNSS)        │
 │  ros2_control / motor node firmware                                        │
 └───────────────────────────────────────────────────────────────────────────┘
```

### 2.1 Key decisions

| # | Decision | Reason |
| --- | --- | --- |
| D1 | Browser web app, written in TypeScript with React and Vite. | Web app is preferred. The same code can later be wrapped with Capacitor for Android and iOS. |
| D2 | Talk to ROS through `rosbridge_suite` over WebSocket, using `roslibjs`. | Standard, works from any browser, supports topics, services and actions. No ROS on the client. |
| D3 | **The robot is the source of truth** for fields, job state and statistics. The web app holds no durable data. | The robot must run without a connection, so it needs its fields locally. Any device that connects sees the same data. |
| D4 | The robot serves the web app itself (static files over HTTP). **[Proposed]** | One thing to deploy. Works on a property with no internet. |
| D5 | Fields are stored in WGS84 latitude/longitude as GeoJSON, never in the `map` frame. | The `map` origin is reset by the first GPS fix at each startup (see `r5_localisation` README), so `map` coordinates are not stable between runs. GeoJSON also opens directly in QGIS. |
| D6 | Map display with MapLibre GL JS; drawing with a polygon-editing plugin (e.g. `terra-draw`). | QGIS is a desktop application and cannot be embedded in a web app. Using GeoJSON keeps QGIS available as an offline tool for inspecting or preparing fields (import/export, see FM-9). |
| D7 | A password is checked by `r5_dashboard_server`, which then proxies the WebSocket to rosbridge. rosbridge itself is not reachable from the network. | rosbridge has no practical login of its own. Putting one server in front gives one place to check the password for both the web app and the ROS link. See 7.3. |

## 3. Layout and navigation

- **NAV-1** A side menu on the **left** is always visible with: Manage Fields, Spray, Mow, Robot.
  (The wireframe draws it on the right; the left is correct.)
- **NAV-2** Only one page is shown at a time. The current page is highlighted in the menu.
- **NAV-3** A status strip is always visible showing: connection state, battery %, current action,
  and active alert count. **[Proposed]**
- **NAV-4** A **Stop** button is always visible while the robot is moving or running a job. It cancels
  the active job and commands zero velocity. **[Proposed — strongly recommended for safety]**
- **NAV-5** Spray in the menu opens Area Spray. Spot Spray is reached from a tab or toggle on the
  Spray page and shows "Coming soon" in v1.
- **NAV-6** Mow shows "Coming soon" in v1.

## 4. Pages

### 4.1 Manage Fields

Wireframe: a gallery of field thumbnails ("New", "River paddock", "East field", "Rice_02"), and an
"Add new field" area with **Start recording / Save** and **Draw on map**.

#### Field model

| Property | Type | Notes |
| --- | --- | --- |
| `id` | UUID | Set by the robot |
| `name` | text, 1–60 chars, unique | |
| `boundary` | polygon, WGS84 | Outer edge of the field. Must not self-intersect. |
| `headland_width_m` | number | Width of the headland (turnrow): the strip at the ends of the rows where the robot turns. Fields2Cover generates it inside the boundary from this width. Default comes from robot config (placeholder: 2.0 m); editable per field. |
| `keep_out` | list of polygons, WGS84 | Known obstacles and no-go zones. Each must lie inside the boundary. |
| `source` | `drawn` \| `recorded` \| `imported` | |
| `area_m2` | number | Calculated, not entered |
| `created`, `modified` | timestamps | |
| `notes` | text, optional | |

#### Requirements

- **FM-1** The page shows every saved field as a card with a thumbnail of its outline, its name and its area
  (ha). A "New" card starts field creation.
- **FM-2** Selecting a field shows it on a satellite map with its boundary, headland and keep-out zones,
  and offers Edit, Rename, Delete and "Spray this field".
- **FM-3 Draw on map.** The user clicks points on the map to draw the boundary, closes the polygon,
  then optionally draws keep-out zones and sets the headland width. The map centres on the robot's
  current position when it is known.
- **FM-4 Record by driving.**
  1. User presses **Start recording**. The robot begins logging its GNSS position.
  2. User drives the robot around the field edge with an Xbox controller connected to the robot
     (see 5.3).
  3. The dashboard draws the recorded track live on the map, with the current fix quality.
  4. User presses **Stop**, then **Save**. The track is closed into a polygon and simplified
     (points removed while keeping within 0.1 m of the track — tunable).
  5. The user can then adjust it as in FM-5 before giving it a name.
  - Points are only recorded when the fix quality is RTK fixed (configurable). The UI warns while
    the fix is worse, and shows how many points were skipped.
  - Recording happens on the robot, so it continues if the link drops during the drive.
  - Recording can also be started and stopped from a controller button, because the person driving
    is next to the robot and may not have the dashboard to hand. The dashboard picks up the
    recording in progress when it connects. **[Proposed]**
  - Requires an RTK receiver. The development M9N (about 2 m accuracy) is enough to test the flow,
    and the RTK-fixed filter is switched off for that.
  - Keep-out zones can be recorded the same way, as a second step.
- **FM-5 Edit.** The user can move, add and delete boundary and keep-out vertices, add or remove keep-out
  zones, change the headland width and rename the field.
- **FM-6 Validation.** Saving is refused, with a clear message, if the boundary self-intersects, has
  fewer than 3 points, or a keep-out zone crosses the boundary. A warning (not a block) is shown if
  the area is outside 0.1–3 ha.
- **FM-7 Delete** asks for confirmation and is refused while a job is using that field.
- **FM-8 Storage.** Fields are saved on the robot in non-volatile storage, one GeoJSON file per field
  (see 6.3). A save is only reported as successful once it is written to disk.
- **FM-9 Import/export.** **[Proposed]** Export a field as GeoJSON and import a GeoJSON or KML polygon,
  so fields can be prepared or checked in QGIS or Google Earth.
- **FM-10 Home location.** The robot has one home point, the garage, where it goes to be refilled
  (AS-9) and when the link is lost (AS-7). It is shown on the map on this page and set on the Settings
  page with "Set to current location". It is stored on the robot like a field. A spray job cannot
  start until home is set.
- **FM-11 Headland display.** The map shows the headland as a shaded strip inside the boundary, so the
  user can see where the robot will turn before saving.
- **FM-12** In v1 the headland is not sprayed. The robot only turns there. The area shown on field
  cards is the whole field; the sprayable area (field minus headland and keep-out zones) is shown in
  the Area Spray summary (AS-4).

### 4.2 Spray — Area Spray

Wireframe: a field dropdown, a map showing the planned coverage path, inputs for sprayer width, speed,
tank capacity and flow rate, and an **Upload** button.

#### Inputs

| Input | Unit | Default | Notes |
| --- | --- | --- | --- |
| Field | — | — | Dropdown of saved fields |
| Application rate | L/m² | — | Entered to 3 decimal places, as typical rates are small (e.g. 0.020 L/m² = 200 L/ha) |
| Boom width | m | robot config (placeholder 1.5) | Also used as the swath width for the coverage path |
| Travel speed | km/h | robot config (1.0) | At most the robot's top speed, 1.08 km/h with Nav2's current 0.3 m/s limit. |
| Tank capacity | L | robot config (placeholder 50) | |
| Row direction | degrees, or Auto | Auto | **[Proposed]** Auto lets Fields2Cover pick the angle that gives the fewest turns |

#### Requirements

- **AS-1** The flow rate is calculated, not typed:
  `flow (L/min) = rate (L/m²) × boom width (m) × speed (km/h) × 1000 / 60`. It is shown read-only
  (the wireframe's "Flow rate" box) and checked against the pump's maximum flow (placeholder
  10 L/min, see 5.5). If it is too high, the dashboard suggests the highest speed that works.
- **AS-2** Inputs are checked for range before planning. Last-used values are remembered per field.
- **AS-3 Preview.** When the field or the inputs change, the dashboard asks the robot to plan the
  coverage path (without moving) and draws it on the map: headland, swaths and turns.
- **AS-4 Job summary.** Next to the preview: area sprayed (excluding the headland, FM-12), path length, estimated time, total
  liquid needed, and number of tank refills. If refills are needed, the summary says so and shows
  how the trips home add to the time.
- **AS-5 Start (wireframe "Upload").** Sends the plan to the robot and starts the job. Before starting,
  the dashboard checks and shows any failure of: link up, localisation ready (fix quality), battery
  above a minimum, no critical alerts, and the robot within a set distance of the path start. The
  user confirms in a dialog that shows the summary.
- **AS-6 Monitoring.** While the job runs: robot position and heading on the map, completed vs remaining
  path, % complete, time remaining, liquid used, and Pause / Resume / Stop. **[Proposed]**
- **AS-7 Link loss.** What a running job does when no dashboard has been connected for a set time
  is a setting (Settings page): keep working (the default), return home, keep working for a set time
  and then return home, or stop. Returning home and stopping turn the sprayer off and wait for
  Resume, which first drives back to where the robot stopped. The dashboard shows "Disconnected"
  with what the robot will do and the time since last contact, and resynchronises job state on
  reconnect.
- **AS-8** Only one job can run at a time. Starting is disabled while a job is active.
- **AS-9 Refilling.** The robot does not refill itself. When the tank runs low:
  1. The robot turns the sprayer off, saves where it stopped on the path, and drives home (FM-10).
  2. It raises an alert, "Tank empty — robot returned home for refill", shown in the status strip and
     on the Robot page. A user who was not connected sees it on their next connection.
  3. The user refills the tank and presses **Tank refilled — resume**. The robot drives back to where
     it stopped and carries on.
  - Until a tank level sensor exists, the remaining liquid is estimated from flow rate × spraying time,
    and the robot heads home with a reserve left (placeholder 5% of tank). The user confirms a full
    tank when starting and resuming.
  - Trips home use Nav2 normal navigation, outside the field's coverage path. The path between fields
    and home must be clear; obstacles there are not mapped in v1.
- **AS-10 Speeds.** Rows are sprayed at the job's speed. Turns between rows and driving to the
  field, home and back use the turn and transit speeds on the Settings page. None goes above the
  maximum speed setting.
- **AS-11 Low battery and slopes.** A running job warns, returns home or stops when the battery is
  low, as set on the Settings page, and stops when the robot tilts more than the slope limit. Both
  raise an alert and wait for Resume.

#### Spot Spray

- **SS-1** Placeholder page, "Coming soon". No functionality in v1.

### 4.3 Mow

- **MW-1** Placeholder page, "Coming soon". No functionality in v1. The field, preview and job
  components from Area Spray should be reusable here later.

### 4.4 Robot

Wireframe: a 3D view of the robot, a list of Temperature, Battery, Operation time, Next service, and
buttons Test motors, Test GPS, Test IMU.

#### Status

Several of these sources don't exist yet and will be added to the robot later. Until then the
dashboard shows "Not available" for that item rather than a blank or zero.

| Item | Shown as | Source (added later where missing) |
| --- | --- | --- |
| Operating time | hours, total | Persistent counter on the robot (6.4) |
| Distance covered | km, total | Persistent odometer on the robot, from `/odometry/local` |
| Next maintenance | km remaining until next service | Every 100 km of distance covered (configurable) |
| Temperature | °C, with warning colour | e.g. BNO055 temperature, motor driver, main computer |
| Battery | %, voltage, charging | `sensor_msgs/BatteryState` (publisher does not exist yet) |
| Link strength | bars and dBm, plus round-trip time | Robot-side WiFi RSSI; client-measured latency |
| Current action | plain text, e.g. "Spraying East field — 42%" | Job manager / Nav2 action status |
| Firmware version | text | Motor node firmware, robot software version |
| Alerts | list with severity and time | `/diagnostics_agg` |

- **RB-1** The page shows all items in the table above, updated at least every 2 s.
- **RB-2** Alerts show severity (info, warning, error), message, and time. The user can acknowledge
  an alert. Errors also appear in the status strip (NAV-3).
- **RB-3** A 3D model of the robot, loaded from the existing URDF (`r5_description`). **[Proposed — nice
  to have]**; a static image is acceptable for v1.
- **RB-4 Maintenance.** Service is due every 100 km of distance covered (interval in robot config).
  The page shows km remaining. A warning alert is raised at 10 km remaining and when overdue; it does
  not stop the robot. The user records a completed service with "Service done", which resets the
  countdown and is logged with the date and odometer reading.

#### Sensors

- **RB-5 GNSS:** fix type (none / 3D / RTK float / RTK fixed), satellites, horizontal accuracy,
  latitude/longitude, update rate, and RTK correction age (RI-8).
- **RB-6 IMU:** heading, roll, pitch, yaw rate, calibration status, temperature.
- **RB-7 Wheels:** speed of each wheel, motor current if available.
- **RB-8 Test GPS / Test IMU** run a check (data arriving, at the expected rate, values in range,
  fix quality / calibration) and report pass or fail with the reason.

#### Motor test

- **RB-9** The user can spin each motor, or each side, at a low fixed speed for a short fixed time
  (e.g. 20% for 2 s), forward or reverse, and see the measured wheel speed and current.
- **RB-10 Safety.** Before a motor test the dashboard shows a warning ("Robot may move. Lift the wheels
  or clear the area.") which the user must confirm. Tests are refused while a job is running.
  Commands are limited in speed and time on the robot, not just in the UI, and stop automatically
  if the link drops.

## 5. Robot interface

All names below are placeholders until the robot side is built. Packages marked *new* do not exist in
the `r5` workspace yet.

### 5.1 Existing interfaces used

| Interface | Type | Use |
| --- | --- | --- |
| `/fix` | `sensor_msgs/NavSatFix` | Position, fix quality, field recording |
| `/odometry/global` | `nav_msgs/Odometry` | Robot position on the map |
| `/odometry/local` | `nav_msgs/Odometry` | Odometer, wheel speed |
| `bno055/*` | IMU topics | IMU status |
| `/fromLL`, `/toLL` | `robot_localization` services | Convert between WGS84 and `map` |
| `/diagnostics_agg` | `diagnostic_msgs/DiagnosticArray` | Alerts (needs an aggregator configured) |

### 5.2 Coverage planning and execution

- **RI-1** Coverage paths are planned with Fields2Cover (F2C). How F2C is brought in (Nav2's
  `opennav_coverage` server, a source build, or calling the F2C library directly) is decided during
  implementation. The job manager below hides that choice, so the dashboard is not affected by it.
  *Decided in step 4:* the job manager calls F2C's Python bindings from the `ros-jazzy-fields2cover`
  apt package (`opennav_coverage` has no Jazzy release), and drives the result with Nav2's
  `FollowPath` and `NavigateToPose`.
- **RI-2** The dashboard does not call the coverage server directly. It calls a robot-side
  **job manager** (*new*, part of `r5_dashboard_server`), which:
  1. loads the field from storage by `id`;
  2. converts it from WGS84 to the current `map` frame (`/fromLL`);
  3. calls the coverage planner with the boundary, keep-out zones as holes, headland width and swath width;
  4. returns the path as WGS84 for preview, or runs it;
  5. switches the sprayer on along swaths and off on turns and headlands (5.5);
  6. reports progress, and keeps running if the dashboard disconnects.
- **RI-3** Proposed job manager interfaces:

| Name | Kind | Purpose |
| --- | --- | --- |
| `/r5/fields/list`, `get`, `save`, `delete` | services | Field storage |
| `/r5/fields/record` | action | Start/stop GNSS recording; feedback is the live track |
| `/r5/job/plan` | service | Plan only, for preview (AS-3) |
| `/r5/job/run` | action | Run a job; feedback is progress |
| `/r5/job/pause`, `resume` | services | `resume` also confirms a refill (AS-9) |
| `/r5/home/get`, `set` | services | Home location (FM-10) |
| `/r5/stop` | service | Emergency software stop (NAV-4) |
| `/r5/status` | topic, 1 Hz | Summary for the status strip |
| `/r5/stats` | topic | Operating time, distance, service due |
| `/r5/motor_test` | action | Bounded motor test (RB-9, RB-10) |

Field storage could alternatively be a small REST API on the same server. Services are proposed so
everything goes through one connection.

### 5.3 Manual driving

- **RI-4** The robot is driven by hand with an Xbox controller connected directly to the robot
  (USB dongle or Bluetooth), using the standard `joy` and `teleop_twist_joy` nodes. Driving does not
  go through the dashboard or the WiFi link.
- **RI-5** A held enable (deadman) button is needed to move; releasing it stops the robot. Manual speed
  is capped on the robot.
- **RI-6** Controller input overrides an autonomous job: moving the sticks with the enable button held
  pauses the job (via a Nav2 velocity mux such as `twist_mux`). **[Proposed]**
- **RI-7** The dashboard shows whether a controller is connected and whether manual control is active.
  **[Proposed]**

### 5.4 GNSS hardware

- **RI-8** Development uses the u-blox M9N (non-RTK). Production uses a u-blox ZED-F9 RTK receiver,
  with corrections from a separate base station on the property, so no internet is needed for RTK.
  The base station is outside the dashboard's scope. On the Robot page, the GNSS panel (RB-5) shows
  whether corrections are arriving and how old they are, so a base station fault shows up as a clear
  message rather than only as a worse fix.
  The dashboard reads fix quality from `/fix` and works with either; the RTK-only rules (FM-4) are a
  setting.

### 5.5 Sprayer (placeholder)

The sprayer hardware is not finished. Until it is, the robot side uses these placeholder interfaces
and values, kept in config so they can be replaced without dashboard changes:

| Item | Placeholder |
| --- | --- |
| Sprayer on/off | `/r5/sprayer/enable`, `std_msgs/Bool` |
| Flow setpoint | `/r5/sprayer/flow_setpoint`, `std_msgs/Float32`, L/min |
| Pump maximum flow | 10 L/min |
| Tank capacity | 50 L |
| Boom width | 1.5 m |
| Tank level sensor | none (estimated, AS-9) |

In simulation a stub node accepts these and logs the commands.

## 6. Data and storage

- **6.1** All durable data lives on the robot, under one directory (e.g. `~/.r5/`), so it can be
  backed up by copying it.
- **6.2** Writes are atomic (write to a temporary file, then rename), so a power cut cannot leave a
  half-written field.
- **6.3** Fields: `fields/<id>.geojson`, a GeoJSON `FeatureCollection` with the boundary feature and
  keep-out features, plus field properties.
- **6.4** Statistics (operating hours, distance, last service) are saved at least once a minute and on
  clean shutdown.
- **6.5** Job settings (last-used spray values per field) and robot defaults (boom width, tank size,
  maximum pump flow, default headland width, service interval) are kept in YAML.
- **6.7** The home location is stored as `home.geojson`. The service log and the hashed dashboard
  password are stored in the same directory; the password file is readable only by the robot's user.
- **6.6** Browser storage is only used for UI conveniences (map zoom, last page).

## 7. Non-functional requirements

### 7.1 Performance and connection

- **NF-1** Live data (position, status) updates on screen within 1 s of the robot publishing it over a
  healthy link.
- **NF-2** The dashboard loads in under 5 s over the robot WiFi link. The app bundle should stay small
  (target < 3 MB, excluding map tiles).
- **NF-3** The dashboard reconnects automatically and shows connection state at all times (NAV-3).
  Data older than 5 s is shown as stale.
- **NF-4** Bandwidth is limited by throttling high-rate topics on the robot (rosbridge `throttle_rate`).

### 7.2 Platform

- **NF-5** v1 supports current Chrome, Firefox and Edge on laptops (1280×720 and up).
- **NF-6** Future-proofing for phone and tablet: responsive layout, controls at least 44 px for
  touch, no hover-only actions, no desktop-only browser APIs. The side menu becomes a bottom bar or drawer
  on narrow screens.
- **NF-7** Units are metric throughout.

### 7.3 Safety and security

- **NF-8** Anything that makes the robot move (start job, motor test, resume) needs an explicit
  confirmation.
- **NF-9** The robot enforces its own limits. The dashboard is never the only safeguard.
- **NF-10 Login.** A password is needed to use the dashboard, because it gives control of the robot.
  - One password per robot, set when the robot is first set up. It can be changed from the Robot page
    (needs the current password) and reset only from a terminal on the robot.
  - Stored only as a salted hash (e.g. Argon2 or bcrypt).
  - After login the browser gets a session token; the WebSocket is refused without a valid token
    (D7). Sessions last 30 days on a device unless the user logs out (configurable).
  - Repeated wrong passwords are slowed down (e.g. 5 failures → 1 minute wait).
- **NF-10a Encryption.** The dashboard is served over HTTPS so the password and commands are not
  sent in clear text. The robot generates a self-signed certificate on first setup; each device
  accepts it once. Plain HTTP requests are redirected to HTTPS.

### 7.4 Maps

- **NF-11** Satellite imagery is a nice-to-have. The map works without it: fields, headlands, the
  path, home and the robot are drawn over a plain base map (e.g. OpenStreetMap) or, with no internet,
  a plain grid.
- **NF-12** If imagery is used, it must be open data. The imagery source is a setting (tile URL and
  attribution text), so it can be chosen per country. Free sources depend on the country, for example
  USGS imagery (USA, public domain) or LINZ aerial imagery (New Zealand, CC BY 4.0). Commercial
  services such as Esri or Google are not used, because of their licence terms.

## 8. Technology summary

| Area | Choice |
| --- | --- |
| Front end | TypeScript, React, Vite |
| Maps | MapLibre GL JS, terra-draw (drawing), Turf.js (area, validation) |
| ROS link | rosbridge_suite + roslibjs |
| 3D robot (optional) | three.js + urdf-loader |
| Robot server | ROS 2 Jazzy Python package `r5_dashboard_server` (new), in the `r5` workspace |
| Coverage | Fields2Cover (integration route decided during implementation) |
| Manual driving | Xbox controller via `joy` + `teleop_twist_joy` on the robot |
| Mobile (later) | Capacitor wrapping the same web app |
| Testing | Vitest for logic, Playwright for UI, launch tests on the robot side; Gazebo simulation (`r5_simulation`) as the test robot |

## 9. Delivery plan (suggested)

1. **Foundation:** app shell, side menu, login, rosbridge connection and status strip, talking to the
   Gazebo sim.
2. **Robot page:** status, sensors, alerts, motor test.
3. **Field management:** drawing, storage on the robot, editing, then recording by driving.
4. **Area Spray:** planning preview, job summary, start, monitoring.
5. **Hardening:** link-loss testing, tablet layout check, import/export.

## 10. Open questions

### 10.1 Open

None at present.

### 10.2 Resolved

| # | Answer (2026-11-04) | Where applied |
| --- | --- | --- |
| Q1 | Fields2Cover will be used. The integration route is left to implementation. | RI-1, section 8 |
| Q2 | M9N is for development only. Production uses a u-blox ZED-F9 RTK receiver. | FM-4, RI-8 |
| Q4 | Manual driving uses an Xbox controller. | FM-4, 5.3 |
| Q5 | Users enter application rate in L/m². Flow rate is calculated. | Area Spray inputs, AS-1 |
| Q8 | Missing status sources (battery, temperature, RSSI, firmware) will be added to the robot later. | Robot status table |
| Q3 | Headland is the turnrow at the ends of the rows where the robot turns. Set as a width per field. | Field model, FM-11 |
| Q6 | Sprayer not finished; placeholder values and interfaces are used. | 5.5, Area Spray inputs, AS-1 |
| Q7 | When the tank runs low the robot returns home and alerts the user. No self-refill. | AS-9, FM-10 |
| Q9 | Maintenance every 100 km. | Robot status table, RB-4 |
| Q10 | Menu on the left. | NAV-1 |
| Q11 | A password is required to log into the robot. | D7, NF-10 |
| Q12 | Imagery is a nice-to-have, open data only; go without if none exists. | NF-11, NF-12 |
| — | HTTPS with a self-signed certificate is confirmed. | NF-10a |
| Q3a | The headland is left unsprayed in v1. | FM-12, AS-4, scope |
| Q13 | Designing for outdoor field use is out of scope. | Scope |
| Q2a | RTK corrections come from a separate base station. | RI-8, RB-5 |
