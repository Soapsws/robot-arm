#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <random>

namespace arm_control {

constexpr std::size_t kJointCount = 4;

struct JointLimits {
  double lower;
  double upper;
  double max_velocity;
};

struct FlailConfig {
  double window_seconds = 0.4;
  double reaction_gain = 1.0;
  double idle_speed = 0.015;
  double min_flip_rate = 0.15;
  double max_flip_rate = 7.0;
  double speed_response_seconds = 0.2;
  double commitment_response_seconds = 0.12;
};

// ROS-free motion logic. Angles are radians and time is monotonic seconds.
class FlailMotion {
 public:
  FlailMotion(std::array<JointLimits, kJointCount> limits,
              FlailConfig config, std::uint32_t seed);

  void initialize(const std::array<double, kJointCount>& positions);
  void set_actor_target(std::size_t joint, double position, double delta,
                        double now_seconds);
  void step(double now_seconds, double dt_seconds);

  const std::array<double, kJointCount>& positions() const { return positions_; }
  double activity() const { return activity_; }
  double commitment() const { return commitment_; }
  int actor() const { return actor_; }

 private:
  struct Movement {
    double time;
    double delta;
  };

  void update_window(double now_seconds, double dt_seconds);

  std::array<JointLimits, kJointCount> limits_;
  FlailConfig config_;
  std::array<double, kJointCount> positions_{};
  std::array<double, kJointCount> speed_{};
  std::array<int, kJointCount> direction_{};
  std::deque<Movement> movements_;
  std::mt19937 rng_;
  std::uniform_real_distribution<double> unit_{0.0, 1.0};
  double activity_ = 0.0;
  double commitment_ = 0.0;
  double actor_target_ = 0.0;
  int actor_ = -1;
  bool has_moved_ = false;
};

}  // namespace arm_control
