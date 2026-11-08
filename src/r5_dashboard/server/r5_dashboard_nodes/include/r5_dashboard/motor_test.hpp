#ifndef R5_DASHBOARD__MOTOR_TEST_HPP_
#define R5_DASHBOARD__MOTOR_TEST_HPP_

#include <optional>
#include <string>

// Motor test maths (spec RB-9, RB-10), without ROS so it can be unit tested.
namespace r5_dashboard::motor_test
{

enum Side : int { kLeft = 0, kRight = 1, kBoth = 2 };

struct WheelSpeeds
{
  double left = 0.0;
  double right = 0.0;
};

struct Twist
{
  double linear = 0.0;  // m/s
  double angular = 0.0;  // rad/s
};

struct Verdict
{
  bool ok = false;
  std::string message;
};

// Ground speed of the left and right wheels for a test of one side, or both. Throws
// std::invalid_argument for an unknown side.
WheelSpeeds wheel_speeds(int side, bool forward, double speed_mps);

// The velocity that makes a differential drive give these wheel speeds.
Twist twist_for(const WheelSpeeds & wheels, double wheel_separation_m);

std::string side_name(int side);

// Pass if each tested side turned the right way at no less than half the commanded speed.
Verdict evaluate(int side, const WheelSpeeds & commanded,
  const std::optional<WheelSpeeds> & measured);

}  // namespace r5_dashboard::motor_test

#endif  // R5_DASHBOARD__MOTOR_TEST_HPP_
