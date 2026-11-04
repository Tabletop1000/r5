"""field_manager node: field storage, home point and boundary recording (spec 4.1, FM-4, FM-10).

Services   /r5/fields/list, /r5/fields/save, /r5/fields/delete
           /r5/home/get, /r5/home/set
           /r5/recording/start, /r5/recording/stop, /r5/recording/clear   (std_srvs/Trigger)
Publishes  /r5/recording/status  (RecordingStatus, latched, on change and 1 Hz)
           /r5/fields/revision   (std_msgs/UInt32, latched; goes up whenever fields or home change)
Reads      /fix, /navpvt (u-blox, optional), /joy (optional: hold a button to start/stop recording)

Recording runs here on the robot, so it carries on if the dashboard's link drops (spec FM-4).
"""

from __future__ import annotations

import time
from pathlib import Path

import rclpy
import rclpy.executors
from builtin_interfaces.msg import Time
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Joy, NavSatFix
from std_msgs.msg import UInt32
from std_srvs.srv import Trigger

from r5_dashboard_interfaces.msg import Field as FieldMsg
from r5_dashboard_interfaces.msg import GeoPolygon, JobStatus, LatLon, RecordingStatus
from r5_dashboard_interfaces.srv import DeleteField, GetHome, ListFields, SaveField, SetHome

from .fields import (
    FIX_NAMES,
    RECORDING,
    Field,
    FieldInvalid,
    FieldStore,
    Recorder,
    fix_quality,
)
from .shutdown import init_without_signal_handlers

try:  # Only on the real robot, where the u-blox driver is installed.
    from ublox_msgs.msg import NavPVT
except ImportError:  # pragma: no cover
    NavPVT = None

NAVPVT_FRESH_S = 2.0
# JobStatus states while a job is under way.
JOB_ACTIVE = {1, 2, 3, 4, 5, 6, 7}


def to_time(seconds: float) -> Time:
    sec = int(seconds)
    return Time(sec=sec, nanosec=int((seconds - sec) * 1e9))


def ring_to_msg(points) -> list[LatLon]:
    return [LatLon(latitude=lat, longitude=lon) for lat, lon in points]


def ring_from_msg(points) -> list[tuple[float, float]]:
    return [(p.latitude, p.longitude) for p in points]


def field_to_msg(f: Field) -> FieldMsg:
    return FieldMsg(
        id=f.id,
        name=f.name,
        boundary=ring_to_msg(f.boundary),
        keep_out=[GeoPolygon(points=ring_to_msg(z)) for z in f.keep_out],
        headland_width_m=f.headland_width_m,
        source=f.source,
        notes=f.notes,
        area_m2=f.area_m2,
        created=to_time(f.created_s),
        modified=to_time(f.modified_s),
    )


def field_from_msg(m: FieldMsg) -> Field:
    return Field(
        id=m.id,
        name=m.name,
        boundary=ring_from_msg(m.boundary),
        keep_out=[ring_from_msg(z.points) for z in m.keep_out],
        headland_width_m=m.headland_width_m,
        source=m.source or "drawn",
        notes=m.notes,
    )


class FieldManagerNode(Node):
    def __init__(self):
        super().__init__("field_manager")
        p = self.declare_parameter
        data_dir = Path(p("data_dir", "~/.r5").value).expanduser()
        required = str(p("record_fix_requirement", "rtk_fixed").value)
        self.joy_button = int(p("joy_record_button", 3).value)
        self.joy_hold_s = float(p("joy_hold_s", 1.0).value)

        self.store = FieldStore(data_dir)
        self.recorder = Recorder(
            required=required,
            min_spacing_m=float(p("record_min_spacing_m", 0.2).value),
            tolerance_m=float(p("record_tolerance_m", 0.1).value),
        )
        self.revision = 0
        self.carrier: int | None = None
        self.carrier_stamp = 0.0
        self.joy_pressed_at: float | None = None
        self.joy_handled = False

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(RecordingStatus, "/r5/recording/status", latched)
        self.revision_pub = self.create_publisher(UInt32, "/r5/fields/revision", latched)

        self.create_service(ListFields, "/r5/fields/list", self.on_list)
        self.create_service(SaveField, "/r5/fields/save", self.on_save)
        self.create_service(DeleteField, "/r5/fields/delete", self.on_delete)
        self.create_service(GetHome, "/r5/home/get", self.on_get_home)
        self.create_service(SetHome, "/r5/home/set", self.on_set_home)
        self.create_service(Trigger, "/r5/recording/start", self.on_start)
        self.create_service(Trigger, "/r5/recording/stop", self.on_stop)
        self.create_service(Trigger, "/r5/recording/clear", self.on_clear)

        self.create_subscription(NavSatFix, "/fix", self.on_fix, qos_profile_sensor_data)
        if NavPVT is not None:
            self.create_subscription(NavPVT, "/navpvt", self.on_navpvt, qos_profile_sensor_data)
        if self.joy_button >= 0:
            self.create_subscription(Joy, "/joy", self.on_joy, 10)
        self.job_field: str | None = None  # field of the job under way, if any
        self.create_subscription(JobStatus, "/r5/job/status", self.on_job, latched)

        self.create_timer(1.0, self.publish_status)
        self.publish_status()
        self.publish_revision()
        _, problems = self.store.list()
        for name in problems:
            self.get_logger().error(f"Couldn't read field file {name}; it is ignored.")
        self.get_logger().info(
            f"Fields in {self.store.dir}. Recording needs {FIX_NAMES[self.recorder.min_quality]} or better"
            + (f"; hold controller button {self.joy_button} to start/stop." if self.joy_button >= 0 else ".")
        )

    def on_job(self, msg: JobStatus) -> None:
        self.job_field = msg.field_id if msg.state in JOB_ACTIVE else None

    # ---- Fields ----

    def on_list(self, _request, response):
        fields, _ = self.store.list()
        response.fields = [field_to_msg(f) for f in fields]
        return response

    def on_save(self, request, response):
        try:
            saved = self.store.save(field_from_msg(request.field))
        except FieldInvalid as exc:
            response.success, response.message = False, str(exc)
            return response
        except OSError as exc:
            self.get_logger().error(f"Saving field failed: {exc}")
            response.success, response.message = False, f"The robot couldn't write the field to disk: {exc}"
            return response
        self.get_logger().info(f"Saved field “{saved.name}” ({saved.id})")
        response.success, response.message, response.field = True, "Saved.", field_to_msg(saved)
        self.publish_revision()
        return response

    def on_delete(self, request, response):
        if self.job_field == request.id:
            response.success, response.message = False, "A spray job is using this field. Stop it first."
            return response
        try:
            deleted = self.store.delete(request.id)
        except FieldInvalid as exc:
            response.success, response.message = False, str(exc)
            return response
        response.success = deleted
        response.message = "Deleted." if deleted else "That field doesn't exist."
        if deleted:
            self.get_logger().info(f"Deleted field {request.id}")
            self.publish_revision()
        return response

    def on_get_home(self, _request, response):
        home = self.store.get_home()
        response.is_set = home is not None
        if home:
            response.position = LatLon(latitude=home[0], longitude=home[1])
        return response

    def on_set_home(self, request, response):
        try:
            self.store.set_home((request.position.latitude, request.position.longitude))
        except FieldInvalid as exc:
            response.success, response.message = False, str(exc)
            return response
        response.success, response.message = True, "Home set."
        self.publish_revision()
        return response

    def publish_revision(self) -> None:
        self.revision += 1
        self.revision_pub.publish(UInt32(data=self.revision))

    # ---- Recording ----

    def on_start(self, _request, response):
        if self.job_field is not None:
            response.success, response.message = False, "A spray job is running."
        elif self.recorder.state == RECORDING:
            response.success, response.message = False, "Already recording."
        else:
            self.recorder.start()
            self.get_logger().info("Boundary recording started")
            response.success, response.message = True, "Recording started."
        self.publish_status()
        return response

    def on_stop(self, _request, response):
        if self.recorder.state != RECORDING:
            response.success, response.message = False, "Not recording."
        else:
            self.recorder.stop()
            self.get_logger().info(f"Boundary recording stopped: {self.recorder.message}")
            response.success, response.message = True, self.recorder.message
        self.publish_status()
        return response

    def on_clear(self, _request, response):
        if self.recorder.state == RECORDING:
            response.success, response.message = False, "Stop the recording first."
        else:
            self.recorder.clear()
            response.success, response.message = True, "Cleared."
        self.publish_status()
        return response

    def on_navpvt(self, msg) -> None:
        self.carrier = (msg.flags >> 6) & 0b11
        self.carrier_stamp = time.monotonic()

    def on_fix(self, msg: NavSatFix) -> None:
        carrier = self.carrier if time.monotonic() - self.carrier_stamp < NAVPVT_FRESH_S else None
        if self.recorder.add_fix((msg.latitude, msg.longitude), fix_quality(msg.status.status, carrier)):
            self.publish_status()

    def on_joy(self, msg: Joy) -> None:
        pressed = len(msg.buttons) > self.joy_button and msg.buttons[self.joy_button] == 1
        now = time.monotonic()
        if not pressed:
            self.joy_pressed_at, self.joy_handled = None, False
            return
        if self.joy_pressed_at is None:
            self.joy_pressed_at = now
        if not self.joy_handled and now - self.joy_pressed_at >= self.joy_hold_s:
            self.joy_handled = True  # once per press
            if self.job_field is not None and self.recorder.state != RECORDING:
                self.get_logger().info("Controller record button ignored: a spray job is running")
            elif self.recorder.state == RECORDING:
                self.recorder.stop()
                self.get_logger().info(f"Recording stopped from the controller: {self.recorder.message}")
            else:
                self.recorder.start()
                self.get_logger().info("Recording started from the controller")
            self.publish_status()

    def publish_status(self) -> None:
        r = self.recorder
        self.status_pub.publish(
            RecordingStatus(
                state=r.state,
                points=ring_to_msg(r.points),
                skipped_points=r.skipped,
                length_m=r.length_m,
                required_fix=FIX_NAMES[r.min_quality],
                current_fix=FIX_NAMES[r.current_quality],
                message=r.message,
                started=to_time(r.started_s),
            )
        )


def main(args=None):
    init_without_signal_handlers(args)
    node = FieldManagerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
