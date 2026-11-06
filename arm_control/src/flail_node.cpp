#include "arm_control/flail_motion.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <limits>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "urdf/model.hpp"

namespace {

constexpr std::array<const char*, arm_control::kJointCount> kJointNames = {
    "joint_1", "joint_2", "joint_3", "joint_4"};

double seconds_since(std::chrono::steady_clock::time_point start) {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
      .count();
}

std::array<arm_control::JointLimits, arm_control::kJointCount> read_limits(
    const std::string& description) {
  urdf::Model model;
  if (!model.initString(description)) {
    throw std::runtime_error("Could not parse robot_description URDF");
  }
  std::array<arm_control::JointLimits, arm_control::kJointCount> limits{};
  for (std::size_t i = 0; i < kJointNames.size(); ++i) {
    const auto joint = model.getJoint(kJointNames[i]);
    if (!joint || joint->type != urdf::Joint::REVOLUTE || !joint->limits) {
      throw std::runtime_error(std::string("Missing revolute joint limits: ") +
                               kJointNames[i]);
    }
    limits[i] = {joint->limits->lower, joint->limits->upper,
                 joint->limits->velocity};
  }
  return limits;
}

}  // namespace

class FlailNode : public rclcpp::Node {
 public:
  FlailNode() : rclcpp::Node("flail_controller"), started_(std::chrono::steady_clock::now()) {
    const auto description = declare_parameter<std::string>("robot_description", "");
    if (description.empty()) {
      throw std::runtime_error("robot_description must contain the arm URDF");
    }
    arm_control::FlailConfig config;
    config.window_seconds = declare_parameter<double>("window_seconds", config.window_seconds);
    config.reaction_gain = declare_parameter<double>("reaction_gain", config.reaction_gain);
    config.idle_speed = declare_parameter<double>("idle_speed", config.idle_speed);
    config.min_flip_rate = declare_parameter<double>("min_flip_rate", config.min_flip_rate);
    config.max_flip_rate = declare_parameter<double>("max_flip_rate", config.max_flip_rate);
    config.speed_response_seconds =
        declare_parameter<double>("speed_response_seconds", config.speed_response_seconds);
    config.commitment_response_seconds = declare_parameter<double>(
        "commitment_response_seconds", config.commitment_response_seconds);
    const auto seed = declare_parameter<std::int64_t>("seed", 7);
    if (seed < 0 ||
        seed > static_cast<std::int64_t>(std::numeric_limits<std::uint32_t>::max())) {
      throw std::invalid_argument("seed must fit an unsigned 32-bit integer");
    }
    motion_ = std::make_unique<arm_control::FlailMotion>(
        read_limits(description), config, static_cast<std::uint32_t>(seed));
    motion_->initialize({0.0, 0.0, 0.0, 0.0});

    joint_states_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    activity_ = create_publisher<std_msgs::msg::Float64>("flail/activity", 10);
    commitment_ = create_publisher<std_msgs::msg::Float64>("flail/commitment", 10);
    input_ = create_subscription<sensor_msgs::msg::JointState>(
        "flail/slider_input", 10,
        [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) { on_input(*msg); });
    last_step_ = std::chrono::steady_clock::now();
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] { on_timer(); });
  }

 private:
  void on_input(const sensor_msgs::msg::JointState& msg) {
    if (msg.name.size() != msg.position.size()) {
      RCLCPP_WARN(get_logger(), "Ignoring JointState with mismatched names and positions");
      return;
    }
    std::array<double, arm_control::kJointCount> input_positions{};
    std::array<bool, arm_control::kJointCount> seen{};
    for (std::size_t n = 0; n < msg.name.size(); ++n) {
      for (std::size_t i = 0; i < kJointNames.size(); ++i) {
        if (msg.name[n] == kJointNames[i] && std::isfinite(msg.position[n])) {
          input_positions[i] = msg.position[n];
          seen[i] = true;
        }
      }
    }
    if (!std::all_of(seen.begin(), seen.end(), [](bool value) { return value; })) {
      RCLCPP_WARN(get_logger(), "Ignoring incomplete or non-finite slider input");
      return;
    }
    if (!have_input_) {
      previous_input_ = input_positions;
      motion_->initialize(input_positions);
      have_input_ = true;
      return;
    }

    std::size_t changed_joint = 0;
    double largest_change = 0.0;
    for (std::size_t i = 0; i < kJointNames.size(); ++i) {
      const double change = std::abs(input_positions[i] - previous_input_[i]);
      if (change > largest_change) {
        largest_change = change;
        changed_joint = i;
      }
    }
    if (largest_change > 1e-4) {
      motion_->set_actor_target(changed_joint, input_positions[changed_joint],
                                input_positions[changed_joint] - previous_input_[changed_joint],
                                seconds_since(started_));
    }
    previous_input_ = input_positions;
  }

  void on_timer() {
    const auto now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(now - last_step_).count();
    last_step_ = now;
    motion_->step(seconds_since(started_), dt);

    sensor_msgs::msg::JointState state;
    state.header.stamp = get_clock()->now();
    for (std::size_t i = 0; i < kJointNames.size(); ++i) {
      state.name.emplace_back(kJointNames[i]);
      state.position.push_back(motion_->positions()[i]);
    }
    joint_states_->publish(state);

    std_msgs::msg::Float64 activity;
    activity.data = motion_->activity();
    activity_->publish(activity);
    std_msgs::msg::Float64 commitment;
    commitment.data = motion_->commitment();
    commitment_->publish(commitment);
  }

  std::unique_ptr<arm_control::FlailMotion> motion_;
  std::array<double, arm_control::kJointCount> previous_input_{};
  bool have_input_ = false;
  std::chrono::steady_clock::time_point started_;
  std::chrono::steady_clock::time_point last_step_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr activity_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr commitment_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr input_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<FlailNode>());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("flail_controller"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
