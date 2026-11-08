#include "r5_dashboard/motor_test.hpp"

#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <vector>

namespace r5_dashboard::motor_test
{

WheelSpeeds wheel_speeds(int side, bool forward, double speed_mps)
{
  const double v = forward ? speed_mps : -speed_mps;
  switch (side) {
    case kLeft: return {v, 0.0};
    case kRight: return {0.0, v};
    case kBoth: return {v, v};
    default: throw std::invalid_argument("Unknown side " + std::to_string(side));
  }
}

Twist twist_for(const WheelSpeeds & w, double wheel_separation_m)
{
  return {(w.left + w.right) / 2.0, (w.right - w.left) / wheel_separation_m};
}

std::string side_name(int side)
{
  switch (side) {
    case kLeft: return "Left";
    case kRight: return "Right";
    case kBoth: return "Both";
    default: throw std::invalid_argument("Unknown side " + std::to_string(side));
  }
}

Verdict evaluate(int side, const WheelSpeeds & commanded,
  const std::optional<WheelSpeeds> & measured)
{
  if (!measured) {
    return {false, "No wheel speed readings arrived during the test (check /joint_states)."};
  }
  struct Wheel
  {
    const char * name;
    double cmd;
    double meas;
  };
  const Wheel wheels[] = {
    {"Left", commanded.left, measured->left}, {"Right", commanded.right, measured->right}};
  std::string problems;
  char buffer[160];
  for (const auto & w : wheels) {
    if (w.cmd == 0.0) {
      continue;
    }
    if (w.meas * w.cmd <= 0.0) {
      std::snprintf(buffer, sizeof(buffer), "%s wheels didn't turn the commanded way (%+.3f m/s).",
        w.name, w.meas);
    } else if (std::abs(w.meas) < 0.5 * std::abs(w.cmd)) {
      std::snprintf(buffer, sizeof(buffer), "%s wheels reached only %.3f of %.3f m/s.", w.name,
        std::abs(w.meas), std::abs(w.cmd));
    } else {
      continue;
    }
    problems += (problems.empty() ? "" : " ") + std::string(buffer);
  }
  if (!problems.empty()) {
    return {false, problems};
  }
  const std::string what = side == kBoth ? "All wheels" : side_name(side) + "-side wheels";
  return {true, what + " turned as commanded."};
}

}  // namespace r5_dashboard::motor_test
