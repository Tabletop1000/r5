#include "r5_dashboard/stats.hpp"

#include <algorithm>
#include <cmath>
#include <tuple>

#include <nlohmann/json.hpp>

#include "r5_dashboard/files.hpp"

namespace r5_dashboard::stats
{

namespace fs = std::filesystem;
using Json = nlohmann::ordered_json;

void Odometer::update(double x, double y)
{
  if (!std::isfinite(x) || !std::isfinite(y)) {
    return;
  }
  if (!anchor_) {
    anchor_ = std::make_pair(x, y);
    return;
  }
  const double step = std::hypot(x - anchor_->first, y - anchor_->second);
  if (step > kMaxStepM) {
    anchor_ = std::make_pair(x, y);
  } else if (step >= kMinStepM) {
    distance_m += step;
    anchor_ = std::make_pair(x, y);
  }
}

void UsageStats::add_operating_time(double dt_s)
{
  if (dt_s > 0.0 && dt_s <= kMaxTickS) {
    operating_s += dt_s;
  }
}

StatsFileDamaged::StatsFileDamaged(const fs::path & backup)
: std::runtime_error(
    "stats.json couldn't be read and was moved to " + backup.string() +
    ". Counters restart at zero.")
{
}

StatsStore::StatsStore(const fs::path & data_dir)
: stats_path_(data_dir / "stats.json"), service_log_path_(data_dir / "service_log.json")
{
}

UsageStats StatsStore::load() const
{
  try {
    const auto text = read_file(stats_path_);
    if (!text) {
      return {};
    }
    const Json data = Json::parse(*text);
    UsageStats stats;
    stats.operating_s = data.value("operating_s", 0.0);
    stats.odometer = Odometer(data.value("distance_m", 0.0));
    if (data.contains("last_service") && !data["last_service"].is_null()) {
      const Json & last = data["last_service"];
      stats.last_service =
        ServiceRecord{last.at("time_s").get<double>(), last.at("distance_m").get<double>()};
    }
    return stats;
  } catch (const std::exception &) {
    // Keep the damaged file for inspection rather than overwrite it.
    fs::path backup = stats_path_;
    backup.replace_extension(".damaged.json");
    std::error_code ec;
    fs::rename(stats_path_, backup, ec);
    throw StatsFileDamaged(backup);
  }
}

void StatsStore::save(const UsageStats & stats) const
{
  Json data = {
    {"operating_s", stats.operating_s},
    {"distance_m", stats.odometer.distance_m},
    {"last_service", nullptr}};
  if (stats.last_service) {
    data["last_service"] = {
      {"time_s", stats.last_service->time_s}, {"distance_m", stats.last_service->distance_m}};
  }
  write_private_file(stats_path_, data.dump(2) + "\n");
}

void StatsStore::log_service(const ServiceRecord & record) const
{
  Json log = Json::array();
  try {
    const auto text = read_file(service_log_path_);
    if (text) {
      log = Json::parse(*text);
    }
  } catch (const std::exception &) {
    log = Json::array();
  }
  if (!log.is_array()) {
    log = Json::array();
  }
  log.push_back({{"time_s", record.time_s}, {"distance_km", record.distance_m / 1000.0}});
  write_private_file(service_log_path_, log.dump(2) + "\n");
}

ServiceStatus service_status(const UsageStats & stats, double interval_km, double warn_km)
{
  const double since_m = stats.last_service ? stats.last_service->distance_m : 0.0;
  const double driven_km = (stats.odometer.distance_m - since_m) / 1000.0;
  const double remaining = interval_km - driven_km;
  return {remaining, 0.0 < remaining && remaining <= warn_km, remaining <= 0.0};
}

std::vector<DiagnosticEntry> leaf_entries(const std::vector<DiagnosticEntry> & entries)
{
  std::vector<DiagnosticEntry> out;
  for (const auto & e : entries) {
    const std::string prefix = e.name + "/";
    const bool group = std::any_of(entries.begin(), entries.end(), [&](const auto & other) {
          return other.name != e.name && other.name.rfind(prefix, 0) == 0;
        });
    if (!group) {
      out.push_back(e);
    }
  }
  return out;
}

namespace
{

// The last part of "/Sensors/IMU", ignoring a trailing "/".
std::string source_name(const std::string & diag_name)
{
  std::string trimmed = diag_name;
  while (!trimmed.empty() && trimmed.back() == '/') {
    trimmed.pop_back();
  }
  const auto slash = trimmed.rfind('/');
  const std::string last = slash == std::string::npos ? trimmed : trimmed.substr(slash + 1);
  return last.empty() ? diag_name : last;
}

}  // namespace

std::vector<AlertState> AlertTracker::alerts() const
{
  std::vector<AlertState> out;
  for (const auto & [id, alert] : alerts_) {
    out.push_back(alert);
  }
  std::sort(out.begin(), out.end(), [](const AlertState & a, const AlertState & b) {
      return std::make_tuple(-a.severity, a.since_s, a.id) <
      std::make_tuple(-b.severity, b.since_s, b.id);
    });
  return out;
}

bool AlertTracker::set(
  const std::string & id, int severity, const std::string & source, const std::string & message,
  double now_s)
{
  const auto it = alerts_.find(id);
  if (it == alerts_.end()) {
    alerts_[id] = AlertState{id, severity, source, message, now_s, false};
    return true;
  }
  AlertState & current = it->second;
  const bool changed = current.severity != severity || current.message != message;
  if (severity > current.severity) {
    current.acknowledged = false;
  }
  current.severity = severity;
  current.message = message;
  return changed;
}

bool AlertTracker::clear(const std::string & id)
{
  return alerts_.erase(id) > 0;
}

bool AlertTracker::update_diagnostics(const std::vector<DiagnosticEntry> & entries, double now_s)
{
  bool changed = false;
  std::vector<std::string> active;
  for (const auto & entry : leaf_entries(entries)) {
    if (entry.level == kDiagOk) {
      continue;
    }
    const int severity = entry.level == kDiagError ? kError : kWarning;
    std::string message = entry.message;
    if (entry.level == kDiagStale) {
      message = entry.message.empty() ? "No recent data" : "No recent data (" + entry.message + ")";
    } else if (message.empty()) {
      message = "Problem reported";
    }
    const std::string id = "diag:" + entry.name;
    active.push_back(id);
    changed |= set(id, severity, source_name(entry.name), message, now_s);
  }
  for (auto it = alerts_.begin(); it != alerts_.end(); ) {
    const bool stale = it->first.rfind("diag:", 0) == 0 &&
      std::find(active.begin(), active.end(), it->first) == active.end();
    if (stale) {
      it = alerts_.erase(it);
      changed = true;
    } else {
      ++it;
    }
  }
  return changed;
}

bool AlertTracker::set_condition(
  const std::string & id, bool active, int severity, const std::string & source,
  const std::string & message, double now_s)
{
  return active ? set(id, severity, source, message, now_s) : clear(id);
}

bool AlertTracker::acknowledge(const std::string & id)
{
  const auto it = alerts_.find(id);
  if (it == alerts_.end()) {
    return false;
  }
  it->second.acknowledged = true;
  return true;
}

void Episode::update(bool over, bool clear, double now_s)
{
  if (active_) {
    if (clear) {
      active_ = false;
      since_.reset();
    }
    return;
  }
  if (!over) {
    since_.reset();
    return;
  }
  if (!since_) {
    since_ = now_s;
  }
  if (now_s - *since_ >= hold_s_) {
    active_ = true;
    ++number_;
  }
}

}  // namespace r5_dashboard::stats
