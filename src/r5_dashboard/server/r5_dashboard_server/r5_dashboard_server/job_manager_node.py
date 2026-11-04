"""job_manager node: plans and runs area spray jobs (spec 4.2, 5.2, AS-1 to AS-9, NAV-4).

Services   /r5/job/defaults (GetJobDefaults), /r5/job/plan (PlanJob), /r5/job/start (StartJob)
           /r5/job/pause (Trigger), /r5/job/resume (ResumeJob), /r5/stop (Trigger)
Publishes  /r5/job/status (JobStatus, latched, on change and 1 Hz)
           /r5/job/plan   (CoveragePlan of the current job, latched)
           /r5/sprayer/enable (std_msgs/Bool), /r5/sprayer/flow_setpoint (std_msgs/Float32, L/min)
           /speed_limit (nav2_msgs/SpeedLimit) while a job runs
Uses       Nav2 /follow_path and /navigate_to_pose; robot_localization /fromLL
Reads      /fix, /navpvt (optional), /odometry/global, /odometry/local, /battery_state, /r5/alerts

The job runs here, on the robot, and carries on if the dashboard's link drops (spec AS-7).
"""

from __future__ import annotations

import json
import math
import threading
import time
from dataclasses import asdict
from pathlib import Path

import rclpy
import rclpy.executors
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from builtin_interfaces.msg import Time
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav2_msgs.action import FollowPath, NavigateToPose
from nav2_msgs.msg import SpeedLimit
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from robot_localization.srv import FromLL
from sensor_msgs.msg import BatteryState, NavSatFix
from std_msgs.msg import Bool, Float32
from std_srvs.srv import Trigger

from r5_dashboard_interfaces.msg import AlertArray, Check, CoveragePlan as PlanMsg, JobStatus, LatLon, PathSection
from r5_dashboard_interfaces.msg import SpraySettings as SettingsMsg
from r5_dashboard_interfaces.srv import GetJobDefaults, PlanJob, ResumeJob, StartJob

from .auth import write_private_file
from .coverage import SWATH, PlanError, RobotLimits, SpraySettings, plan_coverage
from .fields import FIX_NAMES, REQUIREMENTS, FieldStore, fix_quality
from .job import Affine2D, Tank, along_track, angle_difference, extend_line, flow_for_speed, heading, sprayer_should_be_on
from .shutdown import init_without_signal_handlers

try:  # Only on the real robot.
    from ublox_msgs.msg import NavPVT
except ImportError:  # pragma: no cover
    NavPVT = None

ACTIVE = {
    JobStatus.GOING_TO_START,
    JobStatus.SPRAYING,
    JobStatus.TURNING,
    JobStatus.PAUSED,
    JobStatus.RETURNING_HOME,
    JobStatus.WAITING_FOR_REFILL,
    JobStatus.RETURNING_TO_WORK,
}
FRESH_S = 3.0


class JobStopped(Exception):
    pass


def to_time(seconds: float) -> Time:
    sec = int(seconds)
    return Time(sec=sec, nanosec=int((seconds - sec) * 1e9))


def settings_from_msg(m: SettingsMsg) -> SpraySettings:
    return SpraySettings(
        field_id=m.field_id,
        application_rate_l_per_m2=m.application_rate_l_per_m2,
        boom_width_m=m.boom_width_m,
        speed_kmh=m.speed_kmh,
        tank_capacity_l=m.tank_capacity_l,
        auto_angle=m.auto_angle,
        row_angle_deg=m.row_angle_deg,
    )


def settings_to_msg(s: SpraySettings) -> SettingsMsg:
    return SettingsMsg(**asdict(s))


def check(level: int, label: str, detail: str) -> Check:
    return Check(level=level, label=label, detail=detail)


class JobManagerNode(Node):
    def __init__(self):
        super().__init__("job_manager")
        p = lambda name, default: self.declare_parameter(name, default).value  # noqa: E731
        data_dir = Path(p("data_dir", "~/.r5")).expanduser()
        self.store = FieldStore(data_dir)
        self.settings_path = data_dir / "job_settings.json"
        self.defaults = SpraySettings(
            field_id="",
            application_rate_l_per_m2=float(p("default_rate_l_per_m2", 0.02)),
            boom_width_m=float(p("boom_width_m", 1.5)),
            speed_kmh=float(p("default_speed_kmh", 1.0)),
            tank_capacity_l=float(p("tank_capacity_l", 50.0)),
        )
        self.limits = RobotLimits(
            max_speed_kmh=float(p("max_speed_kmh", 1.08)),
            pump_max_flow_l_min=float(p("pump_max_flow_l_min", 10.0)),
            tank_reserve_fraction=float(p("tank_reserve_fraction", 0.05)),
            turn_radius_m=float(p("turn_radius_m", 1.0)),
            goal_tolerance_m=float(p("goal_tolerance_m", 1.5)),
            travel_speed_mps=float(p("travel_speed_mps", 0.3)),
        )
        self.fix_requirement = str(p("job_fix_requirement", "rtk_fixed"))
        if self.fix_requirement not in REQUIREMENTS:
            raise ValueError(f"job_fix_requirement must be one of {sorted(REQUIREMENTS)}")
        self.min_battery = float(p("min_battery_percent", 30.0))
        self.max_start_distance = float(p("max_start_distance_m", 50.0))
        self.sprayer_placeholder = bool(p("sprayer_is_placeholder", True))
        self.controller_id = str(p("controller_id", "FollowPath"))
        follow_name = str(p("follow_path_action", "/follow_path"))
        navigate_name = str(p("navigate_action", "/navigate_to_pose"))

        group = ReentrantCallbackGroup()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(JobStatus, "/r5/job/status", latched)
        self.plan_pub = self.create_publisher(PlanMsg, "/r5/job/plan", latched)
        self.spray_pub = self.create_publisher(Bool, "/r5/sprayer/enable", latched)
        self.flow_pub = self.create_publisher(Float32, "/r5/sprayer/flow_setpoint", 10)
        self.speed_pub = self.create_publisher(SpeedLimit, "/speed_limit", 10)
        self.cmd_pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)

        self.follow = ActionClient(self, FollowPath, follow_name, callback_group=group)
        self.navigate = ActionClient(self, NavigateToPose, navigate_name, callback_group=group)
        self.cancel_clients = [
            self.create_client(CancelGoal, f"{name}/_action/cancel_goal", callback_group=group)
            for name in (follow_name, navigate_name)
        ]
        self.from_ll = self.create_client(FromLL, str(p("from_ll_service", "/fromLL")), callback_group=group)

        self.fix: NavSatFix | None = None
        self.fix_stamp = 0.0
        self.carrier: int | None = None
        self.carrier_stamp = 0.0
        self.pose: tuple[float, float] | None = None
        self.yaw = 0.0
        self.pose_stamp = 0.0
        self.speed = 0.0
        self.battery: float | None = None
        self.alert_errors: list[str] = []
        self.create_subscription(NavSatFix, "/fix", self.on_fix, qos_profile_sensor_data, callback_group=group)
        if NavPVT is not None:
            self.create_subscription(NavPVT, "/navpvt", self.on_navpvt, qos_profile_sensor_data, callback_group=group)
        self.create_subscription(Odometry, "/odometry/global", self.on_global, 10, callback_group=group)
        self.create_subscription(Odometry, "/odometry/local", self.on_local, 10, callback_group=group)
        self.create_subscription(BatteryState, "/battery_state", self.on_battery, 10, callback_group=group)
        self.create_subscription(AlertArray, "/r5/alerts", self.on_alerts, latched, callback_group=group)

        self.create_service(GetJobDefaults, "/r5/job/defaults", self.on_defaults, callback_group=group)
        self.create_service(PlanJob, "/r5/job/plan", self.on_plan, callback_group=group)
        self.create_service(StartJob, "/r5/job/start", self.on_start, callback_group=group)
        self.create_service(Trigger, "/r5/job/pause", self.on_pause, callback_group=group)
        self.create_service(ResumeJob, "/r5/job/resume", self.on_resume, callback_group=group)
        self.create_service(Trigger, "/r5/stop", self.on_stop, callback_group=group)

        self.lock = threading.RLock()
        self.status = JobStatus(state=JobStatus.IDLE, current_action="Idle")
        self.plan_id = 0
        self.plan_cache: dict = {}
        self.thread: threading.Thread | None = None
        self.stop_requested = threading.Event()
        self.pause_requested = threading.Event()
        self.resume_event = threading.Event()
        self.resume_refilled = False
        self.active_goal = None
        self.sprayer_on = False
        self.create_timer(1.0, self.publish_status, callback_group=group)
        self.publish_status()
        self.publish_sprayer(False, 0.0)
        self.get_logger().info(
            f"Spray jobs need {FIX_NAMES[REQUIREMENTS[self.fix_requirement]]} or better; "
            f"top speed {self.limits.max_speed_kmh:.2f} km/h, tank {self.defaults.tank_capacity_l:.0f} L"
        )

    # ---- Inputs ----

    def on_fix(self, msg: NavSatFix) -> None:
        self.fix, self.fix_stamp = msg, time.monotonic()

    def on_navpvt(self, msg) -> None:
        self.carrier, self.carrier_stamp = (msg.flags >> 6) & 0b11, time.monotonic()

    def on_global(self, msg: Odometry) -> None:
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose, self.pose_stamp = (p.x, p.y), time.monotonic()

    def on_local(self, msg: Odometry) -> None:
        self.speed = abs(msg.twist.twist.linear.x)

    def on_battery(self, msg: BatteryState) -> None:
        self.battery = msg.percentage * 100 if math.isfinite(msg.percentage) else None

    def on_alerts(self, msg: AlertArray) -> None:
        self.alert_errors = [f"{a.source}: {a.message}" for a in msg.alerts if a.severity >= 2]

    def fix_quality(self) -> int:
        if self.fix is None or time.monotonic() - self.fix_stamp > FRESH_S:
            return -1
        carrier = self.carrier if time.monotonic() - self.carrier_stamp < FRESH_S else None
        return fix_quality(self.fix.status.status, carrier)

    # ---- Settings ----

    def load_settings(self) -> dict:
        try:
            data = json.loads(self.settings_path.read_text())
            return data if isinstance(data, dict) else {}
        except (OSError, ValueError):
            return {}

    def remember_settings(self, s: SpraySettings) -> None:
        data = self.load_settings()
        data[s.field_id] = asdict(s)
        try:
            write_private_file(self.settings_path, json.dumps(data, indent=2) + "\n")
        except OSError as exc:
            self.get_logger().warning(f"Couldn't save job settings: {exc}")

    def on_defaults(self, request, response):
        saved = self.load_settings().get(request.field_id)
        settings = SpraySettings(**{**asdict(self.defaults), "field_id": request.field_id})
        if saved:
            try:
                settings = SpraySettings(**{**saved, "field_id": request.field_id})
            except TypeError:
                pass
        response.settings = settings_to_msg(settings)
        response.max_speed_kmh = self.limits.max_speed_kmh
        response.pump_max_flow_l_min = self.limits.pump_max_flow_l_min
        response.tank_reserve_fraction = self.limits.tank_reserve_fraction
        return response

    # ---- Planning and checks ----

    def make_plan(self, s: SpraySettings):
        """Plan, reusing a recent identical plan: Start uses the one the preview just made."""
        fld = self.store.get(s.field_id) if s.field_id else None
        if fld is None:
            raise PlanError("Choose a field.")
        home = self.store.get_home()
        key = (fld.id, fld.modified_s, tuple(asdict(s).values()), home)
        with self.lock:
            cached = self.plan_cache.get(key)
        if cached is not None:
            return fld, cached
        started = time.monotonic()
        plan = plan_coverage(fld, s, self.limits, home)
        self.get_logger().info(f"Planned {fld.name}: {plan.swath_count} rows in {time.monotonic() - started:.1f} s")
        with self.lock:
            self.plan_cache = {k: v for k, v in list(self.plan_cache.items())[-7:]}
            self.plan_cache[key] = plan
        return fld, plan

    def plan_to_msg(self, fld, plan, s: SpraySettings, plan_id: int) -> PlanMsg:
        proj = plan.projection

        def ll(pts):
            return [LatLon(latitude=a, longitude=b) for a, b in (proj.to_latlon(p) for p in pts)]

        return PlanMsg(
            plan_id=plan_id,
            field_id=fld.id,
            field_name=fld.name,
            settings=settings_to_msg(s),
            sections=[PathSection(kind=PathSection.SWATH if x.kind == SWATH else PathSection.TURN, points=ll(x.points)) for x in plan.sections],
            row_angle_deg=plan.row_angle_deg,
            swath_count=plan.swath_count,
            area_sprayed_m2=plan.area_sprayed_m2,
            spray_length_m=plan.spray_length_m,
            path_length_m=plan.path_length_m,
            duration_s=plan.duration_s,
            liquid_needed_l=plan.liquid_needed_l,
            refills=plan.refills,
            flow_l_min=plan.flow_l_min,
            warnings=plan.warnings,
        )

    def checks(self, plan=None) -> list[Check]:
        """Pre-start checks (spec AS-5). The dashboard adds its own link check."""
        out: list[Check] = []
        with self.lock:
            busy = self.status.state in ACTIVE
        out.append(check(Check.FAIL if busy else Check.OK, "Other jobs", "A job is already running." if busy else "None running"))

        home = self.store.get_home()
        out.append(check(Check.OK if home else Check.FAIL, "Home point", "Set" if home else "Set the home point on the Manage Fields page."))

        q = self.fix_quality()
        need = REQUIREMENTS[self.fix_requirement]
        out.append(
            check(
                Check.OK if q >= need else Check.FAIL,
                "GPS",
                f"{FIX_NAMES.get(q, 'No data')} (needs {FIX_NAMES[need]} or better)" if q >= 0 else "No position from /fix",
            )
        )
        fresh = self.pose is not None and time.monotonic() - self.pose_stamp < FRESH_S
        out.append(check(Check.OK if fresh else Check.FAIL, "Localisation", "Ready" if fresh else "No /odometry/global"))

        nav_ok = self.follow.server_is_ready() and self.navigate.server_is_ready()
        out.append(check(Check.OK if nav_ok else Check.FAIL, "Navigation", "Ready" if nav_ok else "Nav2 isn't running"))
        ll_ok = self.from_ll.service_is_ready()
        out.append(check(Check.OK if ll_ok else Check.FAIL, "Map transform", "Ready" if ll_ok else "navsat_transform /fromLL not available"))

        if self.battery is None:
            out.append(check(Check.WARNING, "Battery", "Level not reported"))
        else:
            ok = self.battery >= self.min_battery
            out.append(check(Check.OK if ok else Check.FAIL, "Battery", f"{self.battery:.0f}% (needs {self.min_battery:.0f}%)"))

        out.append(
            check(
                Check.FAIL if self.alert_errors else Check.OK,
                "Alerts",
                "; ".join(self.alert_errors[:3]) if self.alert_errors else "No errors",
            )
        )

        if plan is not None and self.fix is not None and q >= 0:
            start = plan.sections[0].points[0]
            here = plan.projection.to_xy((self.fix.latitude, self.fix.longitude))
            dist = math.dist(start, here)
            ok = dist <= self.max_start_distance
            out.append(
                check(
                    Check.OK if ok else Check.FAIL,
                    "Distance to start",
                    f"{dist:.0f} m" + ("" if ok else f" (must be within {self.max_start_distance:.0f} m)"),
                )
            )
        if self.sprayer_placeholder:
            out.append(check(Check.WARNING, "Sprayer", "Placeholder interface: no sprayer hardware is fitted yet"))
        return out

    def on_plan(self, request, response):
        s = settings_from_msg(request.settings)
        try:
            fld, plan = self.make_plan(s)
        except PlanError as exc:
            response.success, response.message = False, str(exc)
            response.checks = self.checks()
            return response
        response.success, response.message = True, "Planned."
        response.plan = self.plan_to_msg(fld, plan, s, 0)
        response.checks = self.checks(plan)
        response.ready = all(c.level != Check.FAIL for c in response.checks)
        return response

    # ---- Commands ----

    def on_start(self, request, response):
        s = settings_from_msg(request.settings)
        try:
            fld, plan = self.make_plan(s)
        except PlanError as exc:
            response.success, response.message = False, str(exc)
            return response
        response.checks = self.checks(plan)
        if any(c.level == Check.FAIL for c in response.checks):
            response.success, response.message = False, "Some checks failed."
            return response
        if not request.tank_full_confirmed:
            response.success, response.message = False, "Confirm the tank is full."
            return response
        with self.lock:
            if self.thread and self.thread.is_alive():
                response.success, response.message = False, "A job is already running."
                return response
            self.remember_settings(s)
            self.plan_id += 1
            self.stop_requested.clear()
            self.pause_requested.clear()
            self.resume_event.clear()
            self.plan_pub.publish(self.plan_to_msg(fld, plan, s, self.plan_id))
            self.thread = threading.Thread(target=self.run_job, args=(fld, plan, s, self.plan_id), daemon=True)
            self.thread.start()
        response.success, response.message = True, "Job started."
        return response

    def on_pause(self, _request, response):
        with self.lock:
            active = self.status.state in ACTIVE and self.status.state not in (JobStatus.PAUSED, JobStatus.WAITING_FOR_REFILL)
        if not active:
            response.success, response.message = False, "Nothing to pause."
            return response
        self.pause_requested.set()
        self.cancel_active_goal()
        response.success, response.message = True, "Pausing."
        return response

    def on_resume(self, request, response):
        with self.lock:
            state = self.status.state
        if state == JobStatus.WAITING_FOR_REFILL and not request.tank_refilled:
            response.success, response.message = False, "Confirm the tank has been refilled."
            return response
        if state not in (JobStatus.PAUSED, JobStatus.WAITING_FOR_REFILL):
            response.success, response.message = False, "The job isn't paused."
            return response
        self.resume_refilled = request.tank_refilled
        self.pause_requested.clear()
        self.resume_event.set()
        response.success, response.message = True, "Resuming."
        return response

    def on_stop(self, _request, response):
        """Software stop (spec NAV-4): end any job, cancel navigation, sprayer off, zero velocity."""
        job_running = self.thread is not None and self.thread.is_alive()
        self.stop_requested.set()
        self.resume_event.set()
        self.cancel_active_goal()
        for client in self.cancel_clients:  # all goals, including ones this node didn't send
            if client.service_is_ready():
                client.call_async(CancelGoal.Request())
        self.publish_sprayer(False, 0.0)
        for _ in range(5):
            self.publish_zero_velocity()
        self.get_logger().warning("Stop requested from the dashboard")
        if not job_running:  # a running job reports its own stop, with how much was sprayed
            self.set_status(current_action="Stopped", message="Stopped from the dashboard.")
        response.success, response.message = True, "Stopped."
        return response

    # ---- Low-level helpers ----

    def publish_zero_velocity(self) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        self.cmd_pub.publish(msg)

    def publish_sprayer(self, on: bool, flow: float) -> None:
        if on != self.sprayer_on:
            self.get_logger().info(f"Sprayer {'on' if on else 'off'}")
        self.sprayer_on = on
        self.spray_pub.publish(Bool(data=on))
        self.flow_pub.publish(Float32(data=float(flow if on else 0.0)))

    def set_speed_limit(self, mps: float) -> None:
        msg = SpeedLimit(percentage=False, speed_limit=float(mps))
        msg.header.stamp = self.get_clock().now().to_msg()
        self.speed_pub.publish(msg)

    def cancel_active_goal(self) -> None:
        goal = self.active_goal
        if goal is not None:
            goal.cancel_goal_async()

    def wait_future(self, future, timeout: float):
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout):
            raise RuntimeError("timed out")
        return future.result()

    def map_transform(self, plan) -> Affine2D:
        """Field-local metres → map frame, using navsat_transform's /fromLL at three points."""
        if not self.from_ll.wait_for_service(timeout_sec=3.0):
            raise RuntimeError("navsat_transform's /fromLL service isn't available")
        step = 100.0
        pts = []
        for xy in ((0.0, 0.0), (step, 0.0), (0.0, step)):
            lat, lon = plan.projection.to_latlon(xy)
            req = FromLL.Request(ll_point=GeoPoint(latitude=lat, longitude=lon, altitude=0.0))
            res = self.wait_future(self.from_ll.call_async(req), 5.0)
            pts.append((res.map_point.x, res.map_point.y))
        t = Affine2D.from_points(pts[0], pts[1], pts[2], step)
        if t.scale_error() > 0.02:
            self.get_logger().warning(f"Map transform scale is off by {t.scale_error():.1%}")
        return t

    def pose_msg(self, xy, yaw: float) -> PoseStamped:
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y = xy
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return ps

    def path_msg(self, points) -> NavPath:
        path = NavPath()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        for i, xy in enumerate(points):
            j = min(i + 1, len(points) - 1)
            k = i if j != i else i - 1
            path.poses.append(self.pose_msg(xy, heading(points[k], points[j])))
        return path

    # ---- Status ----

    def set_status(self, **fields) -> None:
        with self.lock:
            for k, v in fields.items():
                setattr(self.status, k, v)
        self.publish_status()

    def publish_status(self) -> None:
        with self.lock:
            self.status.sprayer_on = self.sprayer_on
            self.status_pub.publish(self.status)

    # ---- Running a job ----

    def run_job(self, fld, plan, s: SpraySettings, plan_id: int) -> None:
        tank = Tank(s.tank_capacity_l, self.limits.tank_reserve_fraction)
        total_spray = plan.spray_length_m
        progress = {"done": 0.0}
        speed_mps = s.speed_kmh / 3.6
        self.set_status(
            state=JobStatus.GOING_TO_START,
            field_id=fld.id,
            field_name=fld.name,
            plan_id=plan_id,
            section_index=0,
            swaths_done=0,
            swath_count=plan.swath_count,
            progress=0.0,
            area_done_m2=0.0,
            liquid_used_l=0.0,
            tank_remaining_l=tank.remaining,
            tank_capacity_l=tank.capacity,
            remaining_s=plan.duration_s,
            message="",
            current_action=f"Going to the start of {fld.name}",
            started=to_time(time.time()),
        )
        self.get_logger().info(f"Spray job started: {fld.name}, {plan.swath_count} rows")
        try:
            transform = self.map_transform(plan)
            sections = [(x.kind, [transform.apply(p) for p in x.points], x.length) for x in plan.sections]
            home_ll = self.store.get_home()
            home = transform.apply(plan.projection.to_xy(home_ll)) if home_ll else None
            self.set_speed_limit(speed_mps)

            first = sections[0][1]
            self.navigate_to(first[0], heading(first[0], first[1]), JobStatus.GOING_TO_START, f"Going to the start of {fld.name}")

            swaths_done = 0
            for index, (kind, pts, length) in enumerate(sections):
                self.set_status(section_index=index)
                if kind == SWATH:
                    need = length * s.boom_width_m * s.application_rate_l_per_m2
                    if tank.needs_refill_for(need):
                        self.refill(fld, home, pts, tank)
                    self.run_swath(fld, pts, length, s, tank, progress, total_spray, swaths_done, plan)
                    swaths_done += 1
                    progress["done"] += length
                    self.set_status(swaths_done=swaths_done)
                else:
                    pct = 100 * progress["done"] / total_spray
                    self.follow_section(pts, JobStatus.TURNING, f"Turning — {fld.name} {pct:.0f}%")
            area = plan.area_sprayed_m2
            area_text = f"{area / 10_000:.2f} ha" if area >= 1000 else f"{area:.0f} m²"
            self.finish(JobStatus.FINISHED, f"Finished {fld.name}", f"Sprayed {area_text} with {tank.used_total:.1f} L.")
        except JobStopped:
            with self.lock:
                done = self.status.progress
            self.finish(JobStatus.STOPPED, "Stopped", f"Stopped from the dashboard. {done:.0%} of {fld.name} was sprayed.")
        except Exception as exc:  # noqa: BLE001 - any failure must leave the robot safe
            self.get_logger().error(f"Spray job failed: {exc}")
            self.finish(JobStatus.FAILED, "Job failed", f"The job failed: {exc}")

    def finish(self, state: int, action: str, message: str) -> None:
        self.publish_sprayer(False, 0.0)
        self.set_speed_limit(0.0)  # 0 removes the limit
        self.publish_zero_velocity()
        self.active_goal = None
        self.set_status(state=state, current_action=action, message=message, remaining_s=0.0)
        self.get_logger().info(message)

    def refill(self, fld, home, row_points, tank: Tank) -> None:
        """Spec AS-9: sprayer off, drive home, wait for the refill, come back to this row."""
        if home is None:
            raise RuntimeError("the tank needs refilling but no home point is set")
        self.publish_sprayer(False, 0.0)
        self.navigate_to(home, 0.0, JobStatus.RETURNING_HOME, "Returning home to refill", any_heading=True)
        self.set_status(
            state=JobStatus.WAITING_FOR_REFILL,
            current_action="Waiting for a refill at home",
            message="Tank empty — robot returned home for refill. Refill it, then press Resume.",
        )
        self.get_logger().info("Waiting for refill")
        while True:
            self.resume_event.clear()
            self.resume_event.wait()
            if self.stop_requested.is_set():
                raise JobStopped
            if self.resume_refilled:
                break
        tank.refill()
        self.set_status(tank_remaining_l=tank.remaining, message="")
        self.navigate_to(row_points[0], heading(row_points[0], row_points[1]), JobStatus.RETURNING_TO_WORK, f"Returning to {fld.name}")

    def run_swath(self, fld, pts, length, s: SpraySettings, tank: Tank, progress, total, swaths_done, plan) -> None:
        start, end = pts[0], pts[-1]
        lead = self.limits.goal_tolerance_m + 0.5
        lead_out = min(lead, max(0.0, fld.headland_width_m - 0.3))
        path = extend_line(start, end, lead, lead_out)
        row_heading = heading(start, end)
        last = {"t": time.monotonic()}

        def monitor() -> bool:
            """Sprayer by position along the row; returns True once past the row's end."""
            now = time.monotonic()
            dt, last["t"] = now - last["t"], now
            if self.pose is None:
                return False
            along, offset = along_track(self.pose, start, end)
            on = sprayer_should_be_on(along, offset, length, s.boom_width_m, angle_difference(self.yaw, row_heading))
            flow = flow_for_speed(s.application_rate_l_per_m2, s.boom_width_m, self.speed) if on else 0.0
            self.publish_sprayer(on, flow)
            tank.use(flow, dt)
            done = progress["done"] + min(max(along, 0.0), length)
            remaining_spray = total - done
            remaining_s = remaining_spray / (s.speed_kmh / 3.6) + max(0.0, plan.path_length_m - plan.spray_length_m) * (remaining_spray / total) / self.limits.travel_speed_mps
            self.set_status(
                progress=done / total,
                area_done_m2=done * s.boom_width_m,
                liquid_used_l=tank.used_total,
                tank_remaining_l=tank.remaining,
                remaining_s=remaining_s,
                current_action=f"Spraying {fld.name} — {100 * done / total:.0f}%",
            )
            return along >= length + 0.2

        self.follow_section(path, JobStatus.SPRAYING, f"Spraying {fld.name}", monitor=monitor)
        self.publish_sprayer(False, 0.0)

    # ---- Navigation with pause and stop ----

    def check_pause(self, state: int, action: str) -> None:
        """If a pause was requested: everything off, wait for resume. Raises JobStopped on stop."""
        if self.stop_requested.is_set():
            raise JobStopped
        if not self.pause_requested.is_set():
            return
        self.publish_sprayer(False, 0.0)
        self.publish_zero_velocity()
        self.set_status(state=JobStatus.PAUSED, current_action="Paused")
        while self.pause_requested.is_set():
            self.resume_event.clear()
            self.resume_event.wait()
            if self.stop_requested.is_set():
                raise JobStopped
        self.set_status(state=state, current_action=action, message="")

    def run_goal(self, client: ActionClient, goal, state: int, action: str, monitor=None) -> None:
        """Send a goal and wait, handling pause/stop. Retries a failed goal once, then pauses."""
        failures = 0
        while True:
            self.check_pause(state, action)
            self.set_status(state=state, current_action=action)
            if not client.wait_for_server(timeout_sec=5.0):
                raise RuntimeError("Nav2 isn't running")
            handle = self.wait_future(client.send_goal_async(goal), 10.0)
            if not handle.accepted:
                raise RuntimeError("Nav2 refused the goal")
            self.active_goal = handle
            result_future = handle.get_result_async()
            finished_early = False
            while not result_future.done():
                if self.stop_requested.is_set() or self.pause_requested.is_set():
                    handle.cancel_goal_async()
                    break
                if monitor and monitor():
                    finished_early = True
                    handle.cancel_goal_async()
                    break
                time.sleep(0.1)
            if not (finished_early or self.stop_requested.is_set() or self.pause_requested.is_set()):
                status = result_future.result().status
            else:
                try:
                    self.wait_future(result_future, 5.0)
                except RuntimeError:
                    pass
                status = GoalStatus.STATUS_SUCCEEDED if finished_early else None
            self.active_goal = None
            if finished_early or status == GoalStatus.STATUS_SUCCEEDED:
                return
            if self.stop_requested.is_set():
                raise JobStopped
            if self.pause_requested.is_set():
                continue  # check_pause waits, then the goal is sent again
            failures += 1
            if failures >= 2:
                self.get_logger().warning("Navigation failed twice; pausing the job")
                self.publish_sprayer(False, 0.0)
                self.pause_requested.set()
                self.set_status(message="Navigation failed. Check the robot, then press Resume to try again.")
                failures = 0

    def follow_section(self, points, state: int, action: str, monitor=None) -> None:
        goal = FollowPath.Goal(path=self.path_msg(points), controller_id=self.controller_id)
        self.run_goal(self.follow, goal, state, action, monitor)

    def navigate_to(self, xy, yaw: float, state: int, action: str, any_heading: bool = False) -> None:
        if self.pose is not None and math.dist(self.pose, xy) < 1.0 and any_heading:
            return
        goal = NavigateToPose.Goal(pose=self.pose_msg(xy, yaw))
        self.run_goal(self.navigate, goal, state, action)


def main(args=None):
    init_without_signal_handlers(args)
    node = JobManagerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.stop_requested.set()
        node.resume_event.set()
        node.publish_sprayer(False, 0.0)
        node.publish_zero_velocity()
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
