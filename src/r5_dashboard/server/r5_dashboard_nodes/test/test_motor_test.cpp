#include <gtest/gtest.h>

#include <stdexcept>

#include "r5_dashboard/motor_test.hpp"

namespace mt = r5_dashboard::motor_test;

TEST(MotorTest, WheelSpeeds)
{
  const auto left = mt::wheel_speeds(mt::kLeft, true, 0.06);
  EXPECT_DOUBLE_EQ(left.left, 0.06);
  EXPECT_DOUBLE_EQ(left.right, 0.0);
  const auto right = mt::wheel_speeds(mt::kRight, false, 0.06);
  EXPECT_DOUBLE_EQ(right.left, 0.0);
  EXPECT_DOUBLE_EQ(right.right, -0.06);
  const auto both = mt::wheel_speeds(mt::kBoth, true, 0.06);
  EXPECT_DOUBLE_EQ(both.left, 0.06);
  EXPECT_DOUBLE_EQ(both.right, 0.06);
  EXPECT_THROW(mt::wheel_speeds(7, true, 0.06), std::invalid_argument);
}

TEST(MotorTest, TwistGivesBackTheWheelSpeeds)
{
  const double b = 0.36;
  for (const mt::WheelSpeeds w : {mt::WheelSpeeds{0.06, 0.0}, mt::WheelSpeeds{0.0, -0.06},
      mt::WheelSpeeds{0.06, 0.06}, mt::WheelSpeeds{-0.06, -0.06}})
  {
    const auto t = mt::twist_for(w, b);
    EXPECT_NEAR(t.linear - t.angular * b / 2, w.left, 1e-12);
    EXPECT_NEAR(t.linear + t.angular * b / 2, w.right, 1e-12);
  }
}

TEST(MotorTest, Evaluate)
{
  EXPECT_TRUE(mt::evaluate(mt::kBoth, {0.06, 0.06}, mt::WheelSpeeds{0.055, 0.058}).ok);
  EXPECT_EQ(mt::evaluate(mt::kBoth, {0.06, 0.06}, mt::WheelSpeeds{0.055, 0.058}).message,
    "All wheels turned as commanded.");
  EXPECT_EQ(mt::evaluate(mt::kLeft, {0.06, 0.0}, mt::WheelSpeeds{0.05, 0.0}).message,
    "Left-side wheels turned as commanded.");

  auto v = mt::evaluate(mt::kLeft, {0.06, 0.0}, mt::WheelSpeeds{0.01, 0.0});
  EXPECT_FALSE(v.ok);
  EXPECT_EQ(v.message, "Left wheels reached only 0.010 of 0.060 m/s.");

  v = mt::evaluate(mt::kRight, {0.0, 0.06}, mt::WheelSpeeds{0.0, -0.05});
  EXPECT_FALSE(v.ok);
  EXPECT_EQ(v.message, "Right wheels didn't turn the commanded way (-0.050 m/s).");

  v = mt::evaluate(mt::kBoth, {0.06, 0.06}, mt::WheelSpeeds{0.0, 0.01});
  EXPECT_EQ(v.message,
    "Left wheels didn't turn the commanded way (+0.000 m/s). "
    "Right wheels reached only 0.010 of 0.060 m/s.");

  v = mt::evaluate(mt::kBoth, {0.06, 0.06}, std::nullopt);
  EXPECT_FALSE(v.ok);
  EXPECT_NE(v.message.find("joint_states"), std::string::npos);
}
