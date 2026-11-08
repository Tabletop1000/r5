// field_manager: field storage, home point and boundary recording (spec 4.1, FM-4, FM-10).
//
// Services   /r5/fields/list, /r5/fields/save, /r5/fields/delete
//            /r5/home/get, /r5/home/set
//            /r5/recording/start, /r5/recording/stop, /r5/recording/clear   (std_srvs/Trigger)
// Publishes  /r5/recording/status  (RecordingStatus, latched, on change and 1 Hz)
//            /r5/fields/revision   (std_msgs/UInt32, latched; goes up whenever fields or home
//                                   change)
// Reads      /fix, /navpvt (u-blox, when built with ublox_msgs), /joy (optional: hold a button to
//            start or stop recording), /r5/job/status
//
// Recording runs here on the robot, so it carries on if the dashboard's link drops (spec FM-4).

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "r5_dashboard_interfaces/msg/field.hpp"
#include "r5_dashboard_interfaces/msg/job_status.hpp"
#include "r5_dashboard_interfaces/msg/recording_status.hpp"
#include "r5_dashboard_interfaces/srv/delete_field.hpp"
#include "r5_dashboard_interfaces/srv/get_home.hpp"
#include "r5_dashboard_interfaces/srv/list_fields.hpp"
#include "r5_dashboard_interfaces/srv/save_field.hpp"
#include "r5_dashboard_interfaces/srv/set_home.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_srvs/srv/trigger.hpp"
#ifdef R5_HAVE_UBLOX_MSGS
#include "ublox_msgs/msg/nav_pvt.hpp"
#endif

#include "r5_dashboard/fields.hpp"
#include "r5_dashboard/files.hpp"

namespace r5_dashboard
{

namespace
{

namespace msg = r5_dashboard_interfaces::msg;
namespace srv = r5_dashboard_interfaces::srv;
using Trigger = std_srvs::srv::Trigger;
using fields::Field;
using fields::FieldInvalid;
using geo::LatLon;
using SteadyClock = std::chrono::steady_clock;

constexpr auto kNavPvtFresh = std::chrono::seconds(2);

builtin_interfaces::msg::Time to_time(double seconds)
{
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<int32_t>(seconds);
  t.nanosec = static_cast<uint32_t>((seconds - t.sec) * 1e9);
  return t;
}

std::vector<msg::LatLon> ring_to_msg(const std::vector<LatLon> & points)
{
  std::vector<msg::LatLon> out;
  for (const auto & p : points) {
    msg::LatLon m;
    m.latitude = p.lat;
    m.longitude = p.lon;
    out.push_back(m);
  }
  return out;
}

std::vector<LatLon> ring_from_msg(const std::vector<msg::LatLon> & points)
{
  std::vector<LatLon> out;
  for (const auto & p : points) {
    out.push_back({p.latitude, p.longitude});
  }
  return out;
}

msg::Field field_to_msg(const Field & f)
{
  msg::Field m;
  m.id = f.id;
  m.name = f.name;
  m.boundary = ring_to_msg(f.boundary);
  for (const auto & zone : f.keep_out) {
    msg::GeoPolygon polygon;
    polygon.points = ring_to_msg(zone);
    m.keep_out.push_back(polygon);
  }
  m.headland_width_m = f.headland_width_m;
  m.source = f.source;
  m.notes = f.notes;
  m.area_m2 = f.area_m2;
  m.created = to_time(f.created_s);
  m.modified = to_time(f.modified_s);
  return m;
}

Field field_from_msg(const msg::Field & m)
{
  Field f;
  f.id = m.id;
  f.name = m.name;
  f.boundary = ring_from_msg(m.boundary);
  for (const auto & zone : m.keep_out) {
    f.keep_out.push_back(ring_from_msg(zone.points));
  }
  f.headland_width_m = m.headland_width_m;
  f.source = m.source.empty() ? "drawn" : m.source;
  f.notes = m.notes;
  return f;
}

void answer(Trigger::Response & response, bool success, const std::string & message)
{
  response.success = success;
  response.message = message;
}

}  // namespace

class FieldManagerNode : public rclcpp::Node
{
public:
  explicit FieldManagerNode(const rclcpp::NodeOptions & options)
  : Node("field_manager", options),
    store_(expand_user(declare_parameter<std::string>("data_dir", "~/.r5"))),
    recorder_(
      declare_parameter<std::string>("record_fix_requirement", "rtk_fixed"),
      declare_parameter("record_min_spacing_m", 0.2),
      declare_parameter("record_tolerance_m", 0.1)),
    joy_button_(static_cast<int>(declare_parameter<int64_t>("joy_record_button", 3))),
    joy_hold_s_(declare_parameter("joy_hold_s", 1.0))
  {
    const auto latched = rclcpp::QoS(1).transient_local();
    status_pub_ = create_publisher<msg::RecordingStatus>("/r5/recording/status", latched);
    revision_pub_ = create_publisher<std_msgs::msg::UInt32>("/r5/fields/revision", latched);

    list_srv_ = create_service<srv::ListFields>(
      "/r5/fields/list",
      [this](const std::shared_ptr<srv::ListFields::Request>,
      std::shared_ptr<srv::ListFields::Response> response) {
        for (const auto & f : store_.list().fields) {
          response->fields.push_back(field_to_msg(f));
        }
      });
    save_srv_ = create_service<srv::SaveField>(
      "/r5/fields/save",
      [this](const std::shared_ptr<srv::SaveField::Request> request,
      std::shared_ptr<srv::SaveField::Response> response) {on_save(*request, *response);});
    delete_srv_ = create_service<srv::DeleteField>(
      "/r5/fields/delete",
      [this](const std::shared_ptr<srv::DeleteField::Request> request,
      std::shared_ptr<srv::DeleteField::Response> response) {on_delete(*request, *response);});
    get_home_srv_ = create_service<srv::GetHome>(
      "/r5/home/get",
      [this](const std::shared_ptr<srv::GetHome::Request>,
      std::shared_ptr<srv::GetHome::Response> response) {
        const auto home = store_.get_home();
        response->is_set = home.has_value();
        if (home) {
          response->position.latitude = home->lat;
          response->position.longitude = home->lon;
        }
      });
    set_home_srv_ = create_service<srv::SetHome>(
      "/r5/home/set",
      [this](const std::shared_ptr<srv::SetHome::Request> request,
      std::shared_ptr<srv::SetHome::Response> response) {on_set_home(*request, *response);});
    start_srv_ = create_service<Trigger>(
      "/r5/recording/start",
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> r) {
        on_start(*r);
      });
    stop_srv_ = create_service<Trigger>(
      "/r5/recording/stop",
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> r) {
        on_stop(*r);
      });
    clear_srv_ = create_service<Trigger>(
      "/r5/recording/clear",
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> r) {
        on_clear(*r);
      });

    fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/fix", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::NavSatFix & m) {on_fix(m);});
#ifdef R5_HAVE_UBLOX_MSGS
    navpvt_sub_ = create_subscription<ublox_msgs::msg::NavPVT>(
      "/navpvt", rclcpp::SensorDataQoS(), [this](const ublox_msgs::msg::NavPVT & m) {
        carrier_ = (m.flags >> 6) & 0b11;
        carrier_stamp_ = SteadyClock::now();
      });
#endif
    if (joy_button_ >= 0) {
      joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, [this](const sensor_msgs::msg::Joy & m) {on_joy(m);});
    }
    job_sub_ = create_subscription<msg::JobStatus>(
      "/r5/job/status", latched, [this](const msg::JobStatus & m) {
        const bool active = m.state >= msg::JobStatus::GOING_TO_START &&
        m.state <= msg::JobStatus::RETURNING_TO_WORK;
        job_field_ = active ? std::optional<std::string>(m.field_id) : std::nullopt;
      });

    timer_ = create_wall_timer(std::chrono::seconds(1), [this] {publish_status();});
    publish_status();
    publish_revision();
    for (const auto & name : store_.list().problems) {
      RCLCPP_ERROR(get_logger(), "Couldn't read field file %s; it is ignored.", name.c_str());
    }
    const std::string joy = joy_button_ >= 0 ?
      "; hold controller button " + std::to_string(joy_button_) + " to start/stop." : ".";
    RCLCPP_INFO(get_logger(), "Fields in %s. Recording needs %s or better%s",
      store_.dir().c_str(), fields::fix_name(recorder_.min_quality()), joy.c_str());
  }

private:
  // ---- Fields ----

  void on_save(const srv::SaveField::Request & request, srv::SaveField::Response & response)
  {
    Field saved;
    try {
      saved = store_.save(field_from_msg(request.field));
    } catch (const FieldInvalid & e) {
      response.success = false;
      response.message = e.what();
      return;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Saving field failed: %s", e.what());
      response.success = false;
      response.message = std::string("The robot couldn't write the field to disk: ") + e.what();
      return;
    }
    RCLCPP_INFO(get_logger(), "Saved field “%s” (%s)", saved.name.c_str(), saved.id.c_str());
    response.success = true;
    response.message = "Saved.";
    response.field = field_to_msg(saved);
    publish_revision();
  }

  void on_delete(const srv::DeleteField::Request & request, srv::DeleteField::Response & response)
  {
    if (job_field_ == request.id) {
      response.success = false;
      response.message = "A spray job is using this field. Stop it first.";
      return;
    }
    bool deleted = false;
    try {
      deleted = store_.remove(request.id);
    } catch (const FieldInvalid & e) {
      response.success = false;
      response.message = e.what();
      return;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Deleting field failed: %s", e.what());
      response.success = false;
      response.message = std::string("The robot couldn't delete the field: ") + e.what();
      return;
    }
    response.success = deleted;
    response.message = deleted ? "Deleted." : "That field doesn't exist.";
    if (deleted) {
      RCLCPP_INFO(get_logger(), "Deleted field %s", request.id.c_str());
      publish_revision();
    }
  }

  void on_set_home(const srv::SetHome::Request & request, srv::SetHome::Response & response)
  {
    try {
      store_.set_home({request.position.latitude, request.position.longitude});
    } catch (const FieldInvalid & e) {
      response.success = false;
      response.message = e.what();
      return;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Saving the home point failed: %s", e.what());
      response.success = false;
      response.message = std::string("The robot couldn't save the home point: ") + e.what();
      return;
    }
    response.success = true;
    response.message = "Home set.";
    publish_revision();
  }

  void publish_revision()
  {
    std_msgs::msg::UInt32 m;
    m.data = ++revision_;
    revision_pub_->publish(m);
  }

  // ---- Recording ----

  void on_start(Trigger::Response & response)
  {
    if (job_field_) {
      answer(response, false, "A spray job is running.");
    } else if (recorder_.state() == fields::kRecording) {
      answer(response, false, "Already recording.");
    } else {
      recorder_.start();
      RCLCPP_INFO(get_logger(), "Boundary recording started");
      answer(response, true, "Recording started.");
    }
    publish_status();
  }

  void on_stop(Trigger::Response & response)
  {
    if (recorder_.state() != fields::kRecording) {
      answer(response, false, "Not recording.");
    } else {
      recorder_.stop();
      RCLCPP_INFO(get_logger(), "Boundary recording stopped: %s", recorder_.message().c_str());
      answer(response, true, recorder_.message());
    }
    publish_status();
  }

  void on_clear(Trigger::Response & response)
  {
    if (recorder_.state() == fields::kRecording) {
      answer(response, false, "Stop the recording first.");
    } else {
      recorder_.clear();
      answer(response, true, "Cleared.");
    }
    publish_status();
  }

  void on_fix(const sensor_msgs::msg::NavSatFix & m)
  {
    std::optional<int> carrier;
    if (carrier_ && SteadyClock::now() - carrier_stamp_ < kNavPvtFresh) {
      carrier = carrier_;
    }
    if (recorder_.add_fix({m.latitude, m.longitude}, fields::fix_quality(m.status.status, carrier))) {
      publish_status();
    }
  }

  void on_joy(const sensor_msgs::msg::Joy & m)
  {
    const bool pressed = static_cast<int>(m.buttons.size()) > joy_button_ &&
      m.buttons[static_cast<std::size_t>(joy_button_)] == 1;
    const auto now = SteadyClock::now();
    if (!pressed) {
      joy_pressed_at_.reset();
      joy_handled_ = false;
      return;
    }
    if (!joy_pressed_at_) {
      joy_pressed_at_ = now;
    }
    const double held = std::chrono::duration<double>(now - *joy_pressed_at_).count();
    if (joy_handled_ || held < joy_hold_s_) {
      return;
    }
    joy_handled_ = true;  // Once per press.
    if (job_field_ && recorder_.state() != fields::kRecording) {
      RCLCPP_INFO(get_logger(), "Controller record button ignored: a spray job is running");
    } else if (recorder_.state() == fields::kRecording) {
      recorder_.stop();
      RCLCPP_INFO(get_logger(), "Recording stopped from the controller: %s",
        recorder_.message().c_str());
    } else {
      recorder_.start();
      RCLCPP_INFO(get_logger(), "Recording started from the controller");
    }
    publish_status();
  }

  void publish_status()
  {
    msg::RecordingStatus m;
    m.state = static_cast<uint8_t>(recorder_.state());
    m.points = ring_to_msg(recorder_.points());
    m.skipped_points = recorder_.skipped();
    m.length_m = recorder_.length_m();
    m.required_fix = fields::fix_name(recorder_.min_quality());
    m.current_fix = fields::fix_name(recorder_.current_quality());
    m.message = recorder_.message();
    m.started = to_time(recorder_.started_s());
    status_pub_->publish(m);
  }

  fields::FieldStore store_;
  fields::Recorder recorder_;
  const int joy_button_;
  const double joy_hold_s_;
  uint32_t revision_ = 0;
  std::optional<int> carrier_;
  SteadyClock::time_point carrier_stamp_;
  std::optional<SteadyClock::time_point> joy_pressed_at_;
  bool joy_handled_ = false;
  std::optional<std::string> job_field_;  // The field of the job under way, if any.

  rclcpp::Publisher<msg::RecordingStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr revision_pub_;
  rclcpp::Service<srv::ListFields>::SharedPtr list_srv_;
  rclcpp::Service<srv::SaveField>::SharedPtr save_srv_;
  rclcpp::Service<srv::DeleteField>::SharedPtr delete_srv_;
  rclcpp::Service<srv::GetHome>::SharedPtr get_home_srv_;
  rclcpp::Service<srv::SetHome>::SharedPtr set_home_srv_;
  rclcpp::Service<Trigger>::SharedPtr start_srv_;
  rclcpp::Service<Trigger>::SharedPtr stop_srv_;
  rclcpp::Service<Trigger>::SharedPtr clear_srv_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
#ifdef R5_HAVE_UBLOX_MSGS
  rclcpp::Subscription<ublox_msgs::msg::NavPVT>::SharedPtr navpvt_sub_;
#endif
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<msg::JobStatus>::SharedPtr job_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace r5_dashboard

RCLCPP_COMPONENTS_REGISTER_NODE(r5_dashboard::FieldManagerNode)
