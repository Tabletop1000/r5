#ifndef R5_DASHBOARD__SETTINGS_HPP_
#define R5_DASHBOARD__SETTINGS_HPP_

#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

// Robot settings that the user changes on the dashboard's Settings page.
//
// Each setting is described once, in settings(). The Settings page builds its form from these
// descriptions, which robot_status publishes on /r5/settings. To add a setting, add it there and
// read it where it's needed, from the values on /r5/settings. The page needs no change.
//
// job_manager is still Python and keeps its own copy of these descriptions in settings.py, for the
// defaults and checks; keep the two the same until it's ported.
//
// The values are kept in <data_dir>/settings.json. A value that's missing or no longer valid there
// falls back to its default.
namespace r5_dashboard::settings
{

// Keeps keys in the order they're added, as Python's json did.
using Json = nlohmann::ordered_json;

// 0.3 m/s: linear.x.max_velocity in r5's diffbot_controllers.yaml and Nav2's desired_linear_vel.
constexpr double kHardwareMaxSpeedKmh = 1.08;

inline constexpr const char * kCommsContinue = "continue";
inline constexpr const char * kCommsReturnHome = "return_home";
inline constexpr const char * kCommsDelayedReturnHome = "delayed_return_home";
inline constexpr const char * kCommsStop = "stop";

inline constexpr const char * kBatteryWarn = "warn";
inline constexpr const char * kBatteryReturnHome = "return_home";
inline constexpr const char * kBatteryStop = "stop";

inline constexpr const char * kUnitsMetric = "metric";
inline constexpr const char * kUnitsUs = "us";

inline constexpr const char * kDeviceTimeZone = "device";

// job_manager starts its job status messages with these when it stops or goes home by itself.
// robot_status raises an alert for each while the job waits for Resume.
inline constexpr const char * kLinkLost = "The link to the dashboard was lost";
inline constexpr const char * kBatteryLow = "The battery is low";
inline constexpr const char * kTooSteep = "The robot is on a slope steeper than the limit";

using Choices = std::vector<std::pair<std::string, std::string>>;

struct Setting
{
  std::string key;
  std::string label;
  std::string group;
  std::string kind;  // "number", "text" or "choice"
  Json default_value;
  std::string help;
  std::string unit;
  std::optional<double> minimum;
  std::optional<double> maximum;
  std::optional<double> step;
  std::optional<int> max_length;
  // (value, label) pairs, for kind "choice".
  Choices choices;
  // (key, value): the page only shows this setting while that setting has that value.
  std::optional<std::pair<std::string, std::string>> shown_when;
  // "speed" for a value in km/h that the page shows in the chosen units. The value, minimum and
  // maximum are always stored in km/h.
  std::string quantity;

  Json to_json() const;
};

// A value the user tried to set isn't allowed. The message is shown to them.
class SettingsError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

// IANA time zones under zoneinfo_dir, as browsers know them, after "this device's time zone".
Choices time_zone_choices(const std::filesystem::path & zoneinfo_dir = "/usr/share/zoneinfo");

const std::vector<Setting> & all();
const Setting * find(const std::string & key);

Json defaults();
std::string schema_json();

// The value to store, or SettingsError with a message for the user.
Json check_value(const Setting & setting, const Json & value);
// Checks every value first, so a bad one means nothing is changed.
Json check_changes(const Json & changes);

struct Loaded
{
  Json values;
  // Keys whose saved value was ignored because it's no longer valid, or the file's path if it
  // couldn't be read at all.
  std::vector<std::string> ignored;
};

class SettingsStore
{
public:
  explicit SettingsStore(const std::filesystem::path & data_dir);

  Loaded load() const;
  // Checks and saves the changes. Returns the new values. Throws SettingsError, or
  // std::filesystem::filesystem_error if they can't be saved.
  Json update(const Json & current, const Json & changes) const;

private:
  std::filesystem::path path_;
};

}  // namespace r5_dashboard::settings

#endif  // R5_DASHBOARD__SETTINGS_HPP_
