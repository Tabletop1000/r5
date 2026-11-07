#ifndef R5_MONITOR__SYSTEM_INFO_HPP_
#define R5_MONITOR__SYSTEM_INFO_HPP_

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

// Reading the computer's state, without ROS, so it can be tested on any machine.
namespace r5_monitor
{

// The hottest thermal zone under `thermal_dir` (normally /sys/class/thermal) in °C, or empty if
// there are none.
std::optional<double> read_temperature_c(const std::filesystem::path & thermal_dir);

struct WifiSignal
{
  std::string interface;
  double dbm = 0.0;
};

// The first interface in the text of /proc/net/wireless, or the one named `interface` if it isn't
// empty. Empty if it isn't listed.
std::optional<WifiSignal> parse_wireless(const std::string & text, const std::string & interface);
std::optional<WifiSignal> read_wifi(const std::filesystem::path & wireless,
  const std::string & interface);

// `git describe --always --dirty --tags` of the first repository holding one of `paths`, or
// "unknown". Gives up after timeout_s.
std::string git_version(const std::vector<std::filesystem::path> & paths, double timeout_s = 5.0);

}  // namespace r5_monitor

#endif  // R5_MONITOR__SYSTEM_INFO_HPP_
