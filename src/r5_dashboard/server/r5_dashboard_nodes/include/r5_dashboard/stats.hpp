#ifndef R5_DASHBOARD__STATS_HPP_
#define R5_DASHBOARD__STATS_HPP_

#include <filesystem>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

// Usage counters, service schedule and alert tracking (spec 4.4, RB-2, RB-4, 6.4), without ROS so
// they can be unit tested. robot_status_node.cpp connects them to ROS.
namespace r5_dashboard::stats
{

// Odometry from the local EKF wobbles slightly while standing still. Distance is only added once
// the robot has moved this far from the last counted point, so the wobble isn't counted.
constexpr double kMinStepM = 0.05;
// A bigger jump between two messages is a reset or a glitch, not driving.
constexpr double kMaxStepM = 2.0;
// Longer gaps between operating-time ticks (a suspended computer, a clock change) aren't counted.
constexpr double kMaxTickS = 10.0;

// Counts distance driven from a stream of (x, y) positions.
class Odometer
{
public:
  Odometer() = default;
  explicit Odometer(double distance_m)
  : distance_m(distance_m) {}

  void update(double x, double y);

  double distance_m = 0.0;

private:
  std::optional<std::pair<double, double>> anchor_;
};

struct ServiceRecord
{
  double time_s = 0.0;
  double distance_m = 0.0;
};

// Everything that must survive a restart. Saved to stats.json.
struct UsageStats
{
  double operating_s = 0.0;
  Odometer odometer;
  std::optional<ServiceRecord> last_service;

  void add_operating_time(double dt_s);
};

// stats.json couldn't be read. It has been moved aside, and the counters restart at zero.
class StatsFileDamaged : public std::runtime_error
{
public:
  explicit StatsFileDamaged(const std::filesystem::path & backup);
};

// Loads and saves UsageStats, and appends to the service log.
class StatsStore
{
public:
  explicit StatsStore(const std::filesystem::path & data_dir);

  // Throws StatsFileDamaged after moving a damaged file aside; loading again then gives zeros.
  UsageStats load() const;
  // Throw std::filesystem::filesystem_error.
  void save(const UsageStats & stats) const;
  void log_service(const ServiceRecord & record) const;

private:
  std::filesystem::path stats_path_;
  std::filesystem::path service_log_path_;
};

struct ServiceStatus
{
  double km_to_service = 0.0;
  bool due_soon = false;
  bool overdue = false;
};

// Service is due every interval_km of driving, counted from the last service (or from zero).
ServiceStatus service_status(const UsageStats & stats, double interval_km, double warn_km);

// ---- Alerts ----

enum Severity : int { kInfo = 0, kWarning = 1, kError = 2 };

// diagnostic_msgs/DiagnosticStatus levels.
enum DiagLevel : int { kDiagOk = 0, kDiagWarn = 1, kDiagError = 2, kDiagStale = 3 };

struct AlertState
{
  std::string id;
  int severity = kInfo;
  std::string source;
  std::string message;
  double since_s = 0.0;
  bool acknowledged = false;
};

struct DiagnosticEntry
{
  std::string name;
  int level = kDiagOk;
  std::string message;
};

// Drops aggregator group entries ("/Sensors") that only repeat their members ("/Sensors/IMU").
std::vector<DiagnosticEntry> leaf_entries(const std::vector<DiagnosticEntry> & entries);

// Turns diagnostics and the robot's own checks into a list of active alerts.
//
// An alert keeps its start time and acknowledgement while it stays active. It is dropped when the
// problem clears. If it gets worse (warning to error) it needs acknowledging again.
class AlertTracker
{
public:
  // Most severe first, then oldest.
  std::vector<AlertState> alerts() const;

  // Replaces all diagnostics-based alerts. Returns true if anything changed.
  bool update_diagnostics(const std::vector<DiagnosticEntry> & entries, double now_s);
  // Raises or clears one of the robot's own alerts. Returns true if anything changed.
  bool set_condition(
    const std::string & id, bool active, int severity, const std::string & source,
    const std::string & message, double now_s);
  bool acknowledge(const std::string & id);

private:
  bool set(const std::string & id, int severity, const std::string & source,
    const std::string & message, double now_s);
  bool clear(const std::string & id);

  std::map<std::string, AlertState> alerts_;
};

// A condition, such as a low battery, that counts once it has held for hold_s.
//
// It ends only when the clear condition holds, which is set a little apart from the start
// condition so a reading near the limit doesn't start a new episode every few seconds. Episodes
// are numbered, so the code acting on them can tell a new one from the one it has handled.
class Episode
{
public:
  explicit Episode(double hold_s)
  : hold_s_(hold_s) {}

  void update(bool over, bool clear, double now_s);
  bool active() const {return active_;}
  int number() const {return number_;}

private:
  double hold_s_;
  std::optional<double> since_;
  bool active_ = false;
  int number_ = 0;
};

}  // namespace r5_dashboard::stats

#endif  // R5_DASHBOARD__STATS_HPP_
