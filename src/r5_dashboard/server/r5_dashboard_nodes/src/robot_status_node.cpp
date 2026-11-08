// robot_status: usage counters, service schedule, alerts and settings for the dashboard.
//
// Publishes  /r5/stats     (r5_dashboard_interfaces/RobotStats, 1 Hz)
//            /r5/alerts    (r5_dashboard_interfaces/AlertArray, on change and 1 Hz)
//            /r5/settings  (r5_dashboard_interfaces/RobotSettings, latched, on change)
// Services   /r5/stats/service_done  (std_srvs/Trigger)  record a completed service
//            /r5/alerts/acknowledge  (r5_dashboard_interfaces/AcknowledgeAlert)
//            /r5/settings/set        (r5_dashboard_interfaces/SetSettings)
// Reads      /odometry/local, /diagnostics_agg, /battery_state, /r5/job/status

#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "r5_dashboard_interfaces/msg/alert_array.hpp"
#include "r5_dashboard_interfaces/msg/job_status.hpp"
#include "r5_dashboard_interfaces/msg/robot_settings.hpp"
#include "r5_dashboard_interfaces/msg/robot_stats.hpp"
#include "r5_dashboard_interfaces/srv/acknowledge_alert.hpp"
#include "r5_dashboard_interfaces/srv/set_settings.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "r5_dashboard/files.hpp"
#include "r5_dashboard/settings.hpp"
#include "r5_dashboard/stats.hpp"

namespace r5_dashboard
{

namespace
{

namespace msg = r5_dashboard_interfaces::msg;
namespace srv = r5_dashboard_interfaces::srv;
using settings::Json;
using stats::kError;
using stats::kInfo;
using stats::kWarning;

const char * const kMaintenanceAlert = "robot:maintenance";
const char * const kNoDiagnosticsAlert = "robot:no_diagnostics";
const char * const kRefillAlert = "job:refill";
const char * const kJobProblemAlert = "job:problem";
const char * const kJobFailedAlert = "job:failed";
const char * const kBatteryAlert = "robot:battery_low";
// Alerts for a job that stopped or went home by itself and waits for Resume, by message start.
const std::pair<const char *, const char *> kJobWaitingAlerts[] = {
  {settings::kLinkLost, "job:link_lost"},
  {settings::kBatteryLow, "job:battery_low"},
  {settings::kTooSteep, "job:too_steep"},
};

// Seconds since the epoch, for times shown to the user.
double wall_s()
{
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch())
         .count();
}

// Seconds on a clock that never jumps, for intervals.
double steady_s()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
         .count();
}

builtin_interfaces::msg::Time to_time(double seconds)
{
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<int32_t>(seconds);
  t.nanosec = static_cast<uint32_t>((seconds - t.sec) * 1e9);
  return t;
}

bool starts_with(const std::string & s, const std::string & prefix)
{
  return s.rfind(prefix, 0) == 0;
}

std::string format(const char * fmt, double value)
{
  char buffer[128];
  std::snprintf(buffer, sizeof(buffer), fmt, value);
  return buffer;
}

}  // namespace

class RobotStatusNode : public rclcpp::Node
{
public:
  explicit RobotStatusNode(const rclcpp::NodeOptions & options)
  : Node("robot_status", options),
    interval_km_(declare_parameter("service_interval_km", 100.0)),
    warn_km_(declare_parameter("service_warning_km", 10.0)),
    diag_timeout_s_(declare_parameter("diagnostics_timeout_s", 10.0)),
    data_dir_(expand_user(declare_parameter<std::string>("data_dir", "~/.r5"))),
    store_(data_dir_),
    settings_store_(data_dir_),
    battery_episode_(10.0),  // The same rule as job_manager's.
    started_(steady_s()),
    last_tick_(started_)
  {
    const double save_period_s = declare_parameter("save_period_s", 60.0);

    try {
      stats_ = store_.load();
    } catch (const stats::StatsFileDamaged & e) {
      RCLCPP_ERROR(get_logger(), "%s", e.what());
      stats_ = store_.load();  // Now empty.
    }
    auto loaded = settings_store_.load();
    settings_ = std::move(loaded.values);
    if (!loaded.ignored.empty()) {
      std::string keys;
      for (const auto & k : loaded.ignored) {
        keys += (keys.empty() ? "" : ", ") + k;
      }
      RCLCPP_WARN(get_logger(), "Using defaults for settings that couldn't be read: %s",
        keys.c_str());
    }

    const auto latched = rclcpp::QoS(1).transient_local();
    stats_pub_ = create_publisher<msg::RobotStats>("/r5/stats", latched);
    alerts_pub_ = create_publisher<msg::AlertArray>("/r5/alerts", latched);
    settings_pub_ = create_publisher<msg::RobotSettings>("/r5/settings", latched);

    // Remapped to a lower-rate copy in dashboard.launch.py.
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/local", 10, [this](const nav_msgs::msg::Odometry & m) {
        stats_.odometer.update(m.pose.pose.position.x, m.pose.pose.position.y);
      });
    diag_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics_agg", 10,
      [this](const diagnostic_msgs::msg::DiagnosticArray & m) {on_diagnostics(m);});
    battery_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
      "/battery_state", 10, [this](const sensor_msgs::msg::BatteryState & m) {on_battery(m);});
    job_sub_ = create_subscription<msg::JobStatus>(
      "/r5/job/status", latched, [this](const msg::JobStatus & m) {on_job(m);});

    service_done_srv_ = create_service<std_srvs::srv::Trigger>(
      "/r5/stats/service_done",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {on_service_done(*response);});
    acknowledge_srv_ = create_service<srv::AcknowledgeAlert>(
      "/r5/alerts/acknowledge",
      [this](const std::shared_ptr<srv::AcknowledgeAlert::Request> request,
      std::shared_ptr<srv::AcknowledgeAlert::Response> response) {
        response->success = alerts_.acknowledge(request->id);
        response->message =
        response->success ? "Acknowledged." : "That alert is no longer active.";
        if (response->success) {
          publish_alerts();
        }
      });
    settings_srv_ = create_service<srv::SetSettings>(
      "/r5/settings/set",
      [this](const std::shared_ptr<srv::SetSettings::Request> request,
      std::shared_ptr<srv::SetSettings::Response> response) {
        on_set_settings(request->values_json, *response);
      });

    tick_timer_ = create_wall_timer(std::chrono::seconds(1), [this] {tick();});
    save_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(save_period_s)),
      [this] {save();});
    publish_settings();
    tick();
  }

  ~RobotStatusNode() override
  {
    save();
  }

private:
  // ---- Inputs ----

  void on_diagnostics(const diagnostic_msgs::msg::DiagnosticArray & m)
  {
    last_diagnostics_ = steady_s();
    std::vector<stats::DiagnosticEntry> entries;
    for (const auto & s : m.status) {
      entries.push_back({s.name, static_cast<int>(s.level), s.message});
    }
    if (alerts_.update_diagnostics(entries, wall_s())) {
      publish_alerts();
    }
  }

  // Spray job events that need the user (spec AS-9).
  void on_job(const msg::JobStatus & m)
  {
    const double now = wall_s();
    bool changed = alerts_.set_condition(
      kRefillAlert, m.state == msg::JobStatus::WAITING_FOR_REFILL, kWarning, "Spray job",
      "Tank empty — robot returned home for refill (" + m.field_name + ").", now);
    const bool paused = m.state == msg::JobStatus::PAUSED;
    changed |= alerts_.set_condition(
      kJobProblemAlert, paused && starts_with(m.message, "Navigation failed"), kError,
      "Spray job", m.message, now);
    // Stopped or went home by itself (link loss, low battery, slope), and waiting for Resume.
    for (const auto & [prefix, alert_id] : kJobWaitingAlerts) {
      changed |= alerts_.set_condition(
        alert_id, paused && starts_with(m.message, prefix), kWarning, "Spray job", m.message, now);
    }
    changed |= alerts_.set_condition(
      kJobFailedAlert, m.state == msg::JobStatus::FAILED, kError, "Spray job", m.message, now);
    if (changed) {
      publish_alerts();
    }
  }

  void on_battery(const sensor_msgs::msg::BatteryState & m)
  {
    if (!std::isfinite(m.percentage)) {
      return;
    }
    const double percent = m.percentage * 100.0;
    const double low = settings_["low_battery_percent"].get<double>();
    const bool was_low = battery_episode_.active();
    battery_episode_.update(percent < low, percent >= low + 5.0, steady_s());
    if (battery_episode_.active() != was_low) {
      alerts_.set_condition(
        kBatteryAlert, battery_episode_.active(), kWarning, "Battery",
        format("Battery low: %.0f%%", percent) + " (the low level is " + format("%g", low) +
        "%). Charge it soon.",
        wall_s());
      publish_alerts();
    }
  }

  void on_service_done(std_srvs::srv::Trigger::Response & response)
  {
    const stats::ServiceRecord record{wall_s(), stats_.odometer.distance_m};
    stats_.last_service = record;
    try {
      store_.log_service(record);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Couldn't add to the service log: %s", e.what());
    }
    save();
    RCLCPP_INFO(get_logger(), "Service recorded at %.1f km", record.distance_m / 1000.0);
    tick();
    response.success = true;
    response.message = "Service recorded.";
  }

  void on_set_settings(const std::string & values_json, srv::SetSettings::Response & response)
  {
    Json changes;
    try {
      changes = Json::parse(values_json);
      settings_ = settings_store_.update(settings_, changes);
    } catch (const Json::parse_error &) {
      response.success = false;
      response.message = "The settings weren't valid JSON.";
      return;
    } catch (const settings::SettingsError & e) {
      response.success = false;
      response.message = e.what();
      return;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Couldn't save settings: %s", e.what());
      response.success = false;
      response.message = std::string("Couldn't save the settings: ") + e.what();
      return;
    }
    RCLCPP_INFO(get_logger(), "Settings changed: %s", changes.dump().c_str());
    publish_settings();
    response.success = true;
    response.message = "Settings saved.";
  }

  void publish_settings()
  {
    msg::RobotSettings m;
    m.schema_json = settings::schema_json();
    m.values_json = settings_.dump();
    settings_pub_->publish(m);
  }

  // ---- Periodic ----

  void tick()
  {
    const double now = steady_s();
    stats_.add_operating_time(now - last_tick_);
    last_tick_ = now;

    const auto status = stats::service_status(stats_, interval_km_, warn_km_);
    const std::string message = status.overdue ?
      format("Service overdue by %.1f km.", -status.km_to_service) :
      format("Service due in %.1f km.", status.km_to_service);
    alerts_.set_condition(
      kMaintenanceAlert, status.due_soon || status.overdue, kWarning, "Maintenance", message,
      wall_s());

    const double diag_age = now - last_diagnostics_.value_or(started_);
    const bool missing = diag_age > diag_timeout_s_;
    if (missing) {
      alerts_.update_diagnostics({}, wall_s());  // Old diagnostics can't be trusted.
    }
    alerts_.set_condition(
      kNoDiagnosticsAlert, missing, kInfo, "Diagnostics",
      "No diagnostics from the robot. Is diagnostic_aggregator running?", wall_s());

    publish_stats(status);
    publish_alerts();
  }

  void publish_stats(const stats::ServiceStatus & status)
  {
    msg::RobotStats m;
    m.operating_hours = stats_.operating_s / 3600.0;
    m.distance_km = stats_.odometer.distance_m / 1000.0;
    m.service_interval_km = interval_km_;
    m.km_to_service = status.km_to_service;
    m.service_due_soon = status.due_soon;
    m.service_overdue = status.overdue;
    if (stats_.last_service) {
      m.has_last_service = true;
      m.last_service_time = to_time(stats_.last_service->time_s);
      m.last_service_distance_km = stats_.last_service->distance_m / 1000.0;
    }
    stats_pub_->publish(m);
  }

  void publish_alerts()
  {
    msg::AlertArray m;
    for (const auto & a : alerts_.alerts()) {
      msg::Alert alert;
      alert.id = a.id;
      alert.severity = static_cast<uint8_t>(a.severity);
      alert.source = a.source;
      alert.message = a.message;
      alert.since = to_time(a.since_s);
      alert.acknowledged = a.acknowledged;
      m.alerts.push_back(alert);
    }
    alerts_pub_->publish(m);
  }

  void save()
  {
    try {
      store_.save(stats_);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Couldn't save usage counters: %s", e.what());
    }
  }

  const double interval_km_;
  const double warn_km_;
  const double diag_timeout_s_;
  const std::filesystem::path data_dir_;
  stats::StatsStore store_;
  settings::SettingsStore settings_store_;
  stats::UsageStats stats_;
  Json settings_;
  stats::AlertTracker alerts_;
  stats::Episode battery_episode_;
  const double started_;
  double last_tick_;
  std::optional<double> last_diagnostics_;

  rclcpp::Publisher<msg::RobotStats>::SharedPtr stats_pub_;
  rclcpp::Publisher<msg::AlertArray>::SharedPtr alerts_pub_;
  rclcpp::Publisher<msg::RobotSettings>::SharedPtr settings_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<msg::JobStatus>::SharedPtr job_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_done_srv_;
  rclcpp::Service<srv::AcknowledgeAlert>::SharedPtr acknowledge_srv_;
  rclcpp::Service<srv::SetSettings>::SharedPtr settings_srv_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
  rclcpp::TimerBase::SharedPtr save_timer_;
};

}  // namespace r5_dashboard

RCLCPP_COMPONENTS_REGISTER_NODE(r5_dashboard::RobotStatusNode)
