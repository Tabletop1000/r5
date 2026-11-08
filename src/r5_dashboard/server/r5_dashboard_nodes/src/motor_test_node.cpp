// motor_test: bounded motor tests for the Robot page (spec RB-9, RB-10).
//
// Action  /r5/motor_test  (r5_dashboard_interfaces/action/MotorTest)
//
// Speed and duration are parameters on the robot, capped in code, and the goal can't change them.
// The test always ends after duration_s, so a dropped link can't leave the motors running.
//
// A 20 Hz timer runs the test, so it never blocks the container's single thread.

#include <algorithm>
#include <chrono>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "r5_dashboard_interfaces/action/motor_test.hpp"
#include "r5_dashboard_interfaces/msg/job_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "r5_dashboard/motor_test.hpp"

namespace r5_dashboard
{

namespace
{

using MotorTestAction = r5_dashboard_interfaces::action::MotorTest;
using GoalHandle = rclcpp_action::ServerGoalHandle<MotorTestAction>;
using JobStatus = r5_dashboard_interfaces::msg::JobStatus;
using Clock = std::chrono::steady_clock;

// Hard limits, whatever the parameters say.
constexpr double kMaxTestSpeedMps = 0.1;
constexpr double kMaxDurationS = 5.0;
constexpr auto kPeriod = std::chrono::milliseconds(50);  // 20 Hz.
// Wheel speeds older than this don't count.
constexpr auto kFreshFor = std::chrono::milliseconds(500);

double seconds_since(Clock::time_point t)
{
  return std::chrono::duration<double>(Clock::now() - t).count();
}

}  // namespace

class MotorTestNode : public rclcpp::Node
{
public:
  explicit MotorTestNode(const rclcpp::NodeOptions & options)
  : Node("motor_test", options),
    wheel_separation_(declare_parameter("wheel_separation", 0.36)),
    wheel_radius_(declare_parameter("wheel_radius", 0.1)),
    speed_(std::min(declare_parameter("test_speed_mps", 0.06), kMaxTestSpeedMps)),
    duration_(std::min(declare_parameter("duration_s", 2.0), kMaxDurationS)),
    left_joints_(declare_parameter<std::vector<std::string>>(
        "left_wheel_joints", {"front_left_wheel_joint", "rear_left_wheel_joint"})),
    right_joints_(declare_parameter<std::vector<std::string>>(
        "right_wheel_joints", {"front_right_wheel_joint", "rear_right_wheel_joint"})),
    moving_threshold_(declare_parameter("moving_threshold_mps", 0.02)),
    stamped_(declare_parameter("stamped_cmd_vel", true))
  {
    const auto cmd_topic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    if (stamped_) {
      stamped_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(cmd_topic, 10);
    } else {
      twist_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_topic, 10);
    }

    joints_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](const sensor_msgs::msg::JointState & msg) {on_joints(msg);});
    // Remapped to a lower-rate copy in dashboard.launch.py.
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/local", 10, [this](const nav_msgs::msg::Odometry & msg) {
        odom_ = motor_test::Twist{msg.twist.twist.linear.x, msg.twist.twist.angular.z};
      });
    job_sub_ = create_subscription<JobStatus>(
      "/r5/job/status", rclcpp::QoS(1).transient_local(), [this](const JobStatus & msg) {
        job_active_ = msg.state >= JobStatus::GOING_TO_START &&
        msg.state <= JobStatus::RETURNING_TO_WORK;
      });

    server_ = rclcpp_action::create_server<MotorTestAction>(
      this, "/r5/motor_test",
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const MotorTestAction::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](std::shared_ptr<GoalHandle>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](std::shared_ptr<GoalHandle> goal) {on_accepted(std::move(goal));});

    // On Ctrl-C the context shuts down before the node is destroyed, and nothing can be published
    // after that. Stop the motors just before.
    shutdown_callback_ = get_node_base_interface()->get_context()->add_pre_shutdown_callback(
      [this] {stop_motors();});

    RCLCPP_INFO(get_logger(), "Motor tests run at %.3f m/s for %.1f s on %s", speed_, duration_,
      cmd_topic.c_str());
  }

  ~MotorTestNode() override
  {
    get_node_base_interface()->get_context()->remove_pre_shutdown_callback(shutdown_callback_);
    if (test_) {
      stop_motors();
    }
  }

private:
  struct Test
  {
    std::shared_ptr<GoalHandle> goal;
    int side = 0;
    motor_test::WheelSpeeds commanded;
    motor_test::Twist twist;
    Clock::time_point start;
    std::vector<motor_test::WheelSpeeds> samples;
  };

  void on_joints(const sensor_msgs::msg::JointState & msg)
  {
    if (msg.velocity.empty()) {
      return;
    }
    std::map<std::string, double> by_name;
    for (std::size_t i = 0; i < msg.name.size() && i < msg.velocity.size(); ++i) {
      by_name[msg.name[i]] = msg.velocity[i];
    }
    const auto average = [&by_name](const std::vector<std::string> & joints) -> std::optional<double> {
        double sum = 0.0;
        int n = 0;
        for (const auto & j : joints) {
          const auto it = by_name.find(j);
          if (it != by_name.end()) {
            sum += it->second;
            ++n;
          }
        }
        return n ? std::optional<double>(sum / n) : std::nullopt;
      };
    const auto left = average(left_joints_);
    const auto right = average(right_joints_);
    if (left && right) {
      wheels_ = motor_test::WheelSpeeds{*left * wheel_radius_, *right * wheel_radius_};
      wheels_stamp_ = Clock::now();
    }
  }

  void refuse(const std::shared_ptr<GoalHandle> & goal, const std::string & message)
  {
    auto result = std::make_shared<MotorTestAction::Result>();
    result->success = false;
    result->message = message;
    goal->succeed(result);
    RCLCPP_WARN(get_logger(), "Motor test refused: %s", message.c_str());
  }

  void on_accepted(std::shared_ptr<GoalHandle> goal)
  {
    const int side = goal->get_goal()->side;
    const bool forward = goal->get_goal()->forward;
    if (side > motor_test::kBoth) {
      refuse(goal, "Unknown side " + std::to_string(side) + ".");
      return;
    }
    if (test_) {
      refuse(goal, "Another motor test is already running.");
      return;
    }
    if (job_active_) {
      refuse(goal, "A spray job is running. Stop it before testing the motors.");
      return;
    }
    if (odom_ && (std::abs(odom_->linear) > moving_threshold_ || std::abs(odom_->angular) > 0.05)) {
      refuse(goal, "The robot is moving. Motor tests only run while it's standing still.");
      return;
    }

    Test test;
    test.goal = goal;
    test.side = side;
    test.commanded = motor_test::wheel_speeds(side, forward, speed_);
    test.twist = motor_test::twist_for(test.commanded, wheel_separation_);
    test.start = Clock::now();
    test_ = std::move(test);
    RCLCPP_INFO(get_logger(), "Motor test: %s %s", motor_test::side_name(side).c_str(),
      forward ? "forward" : "reverse");
    tick();
    timer_ = create_wall_timer(kPeriod, [this] {tick();});
  }

  void tick()
  {
    if (!test_) {
      return;
    }
    Test & test = *test_;
    const double elapsed = seconds_since(test.start);
    if (test.goal->is_canceling()) {
      auto result = std::make_shared<MotorTestAction::Result>();
      result->success = false;
      result->message = "Stopped by the user.";
      test.goal->canceled(result);
      finish();
      return;
    }
    if (elapsed >= duration_) {
      complete();
      return;
    }

    publish_cmd(test.twist.linear, test.twist.angular);
    std::optional<motor_test::WheelSpeeds> fresh;
    if (wheels_ && Clock::now() - wheels_stamp_ < kFreshFor) {
      fresh = wheels_;
    }
    if (fresh && elapsed > duration_ / 2) {
      test.samples.push_back(*fresh);
    }
    auto feedback = std::make_shared<MotorTestAction::Feedback>();
    feedback->elapsed_s = static_cast<float>(elapsed);
    feedback->duration_s = static_cast<float>(duration_);
    feedback->speeds_valid = fresh.has_value();
    feedback->left_speed = fresh ? static_cast<float>(fresh->left) : 0.0f;
    feedback->right_speed = fresh ? static_cast<float>(fresh->right) : 0.0f;
    test.goal->publish_feedback(feedback);
  }

  void complete()
  {
    Test & test = *test_;
    stop_motors();
    std::optional<motor_test::WheelSpeeds> measured;
    if (!test.samples.empty()) {
      motor_test::WheelSpeeds sum;
      for (const auto & s : test.samples) {
        sum.left += s.left;
        sum.right += s.right;
      }
      const double n = static_cast<double>(test.samples.size());
      measured = motor_test::WheelSpeeds{sum.left / n, sum.right / n};
    }
    const auto verdict = motor_test::evaluate(test.side, test.commanded, measured);
    auto result = std::make_shared<MotorTestAction::Result>();
    result->success = verdict.ok;
    result->message = verdict.message;
    result->commanded_left = static_cast<float>(test.commanded.left);
    result->commanded_right = static_cast<float>(test.commanded.right);
    result->measured_valid = measured.has_value();
    result->measured_left = measured ? static_cast<float>(measured->left) : 0.0f;
    result->measured_right = measured ? static_cast<float>(measured->right) : 0.0f;
    test.goal->succeed(result);
    finish();
  }

  void finish()
  {
    stop_motors();
    timer_->cancel();
    timer_.reset();
    test_.reset();
  }

  void publish_cmd(double linear, double angular)
  {
    if (stamped_) {
      geometry_msgs::msg::TwistStamped msg;
      msg.header.stamp = now();
      msg.twist.linear.x = linear;
      msg.twist.angular.z = angular;
      stamped_pub_->publish(msg);
    } else {
      geometry_msgs::msg::Twist msg;
      msg.linear.x = linear;
      msg.angular.z = angular;
      twist_pub_->publish(msg);
    }
  }

  void stop_motors()
  {
    for (int i = 0; i < 3; ++i) {
      publish_cmd(0.0, 0.0);
    }
  }

  const double wheel_separation_;
  const double wheel_radius_;
  const double speed_;
  const double duration_;
  const std::vector<std::string> left_joints_;
  const std::vector<std::string> right_joints_;
  const double moving_threshold_;
  const bool stamped_;

  bool job_active_ = false;
  std::optional<motor_test::WheelSpeeds> wheels_;
  Clock::time_point wheels_stamp_;
  std::optional<motor_test::Twist> odom_;
  std::optional<Test> test_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr stamped_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<JobStatus>::SharedPtr job_sub_;
  rclcpp_action::Server<MotorTestAction>::SharedPtr server_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::PreShutdownCallbackHandle shutdown_callback_;
};

}  // namespace r5_dashboard

RCLCPP_COMPONENTS_REGISTER_NODE(r5_dashboard::MotorTestNode)
