#include "arm_control/flail_motion.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace arm_control {
namespace {

double response_fraction(double dt, double response_seconds) {
  return 1.0 - std::exp(-dt / response_seconds);
}

}  // namespace

FlailMotion::FlailMotion(std::array<JointLimits, kJointCount> limits,
                         FlailConfig config, std::uint32_t seed)
    : limits_(limits), config_(config), rng_(seed) {
  for (const auto& limit : limits_) {
    if (!std::isfinite(limit.lower) || !std::isfinite(limit.upper) ||
        !std::isfinite(limit.max_velocity) || limit.lower >= limit.upper ||
        limit.max_velocity <= 0.0) {
      throw std::invalid_argument("Each joint needs finite position and velocity limits");
    }
  }
  if (!std::isfinite(config_.window_seconds) || config_.window_seconds <= 0.0 ||
      !std::isfinite(config_.reaction_gain) || config_.reaction_gain < 0.0 ||
      !std::isfinite(config_.idle_speed) || config_.idle_speed < 0.0 ||
      !std::isfinite(config_.min_flip_rate) || config_.min_flip_rate < 0.0 ||
      !std::isfinite(config_.max_flip_rate) ||
      config_.max_flip_rate < config_.min_flip_rate ||
      !std::isfinite(config_.speed_response_seconds) ||
      config_.speed_response_seconds <= 0.0 ||
      !std::isfinite(config_.commitment_response_seconds) ||
      config_.commitment_response_seconds <= 0.0) {
    throw std::invalid_argument("Invalid flail configuration");
  }
  for (auto& direction : direction_) {
    direction = unit_(rng_) < 0.5 ? -1 : 1;
  }
}

void FlailMotion::initialize(const std::array<double, kJointCount>& positions) {
  for (std::size_t i = 0; i < kJointCount; ++i) {
    if (!std::isfinite(positions[i])) {
      throw std::invalid_argument("Initial joint positions must be finite");
    }
    positions_[i] = std::clamp(positions[i], limits_[i].lower, limits_[i].upper);
  }
  speed_.fill(0.0);
  movements_.clear();
  actor_ = -1;
  activity_ = 0.0;
  commitment_ = 0.0;
  has_moved_ = false;
}

void FlailMotion::set_actor_target(std::size_t joint, double position,
                                   double delta, double now_seconds) {
  if (joint >= kJointCount || !std::isfinite(position) ||
      !std::isfinite(delta) || !std::isfinite(now_seconds)) {
    throw std::invalid_argument("Invalid actor input");
  }
  if (actor_ != static_cast<int>(joint)) {
    movements_.clear();
    actor_ = static_cast<int>(joint);
    speed_[joint] = 0.0;
  }
  actor_target_ = std::clamp(position, limits_[joint].lower, limits_[joint].upper);
  if (std::abs(delta) > 1e-6) {
    movements_.push_back({now_seconds, delta});
    has_moved_ = true;
  }
}

void FlailMotion::update_window(double now_seconds, double dt_seconds) {
  while (!movements_.empty() &&
         movements_.front().time < now_seconds - config_.window_seconds) {
    movements_.pop_front();
  }
  double travel = 0.0;
  double signed_travel = 0.0;
  for (const auto& movement : movements_) {
    travel += std::abs(movement.delta);
    signed_travel += movement.delta;
  }
  activity_ = travel / config_.window_seconds;
  const double observed_commitment =
      travel > 1e-9 ? std::clamp(std::abs(signed_travel) / travel, 0.0, 1.0) : 0.0;
  commitment_ += response_fraction(dt_seconds, config_.commitment_response_seconds) *
                 (observed_commitment - commitment_);
}

void FlailMotion::step(double now_seconds, double dt_seconds) {
  if (!std::isfinite(now_seconds) || !std::isfinite(dt_seconds) ||
      dt_seconds <= 0.0) {
    throw std::invalid_argument("Motion step needs finite positive time");
  }
  // A paused VM or debugger must not create a large position jump.
  const double dt = std::min(dt_seconds, 0.05);
  update_window(now_seconds, dt);
  const double requested_speed =
      has_moved_ ? std::max(config_.idle_speed, config_.reaction_gain * activity_)
                 : 0.0;
  const double flip_rate = config_.max_flip_rate -
      (config_.max_flip_rate - config_.min_flip_rate) * commitment_;
  const double flip_probability = 1.0 - std::exp(-flip_rate * dt);
  const double response_seconds =
      config_.speed_response_seconds * (1.0 + commitment_);
  const double speed_fraction = response_fraction(dt, response_seconds);

  for (std::size_t i = 0; i < kJointCount; ++i) {
    const auto& limit = limits_[i];
    if (static_cast<int>(i) == actor_) {
      const double error = actor_target_ - positions_[i];
      const double travel = limit.max_velocity * dt;
      positions_[i] += std::clamp(error, -travel, travel);
      continue;
    }

    const double target_speed = std::min(requested_speed, limit.max_velocity);
    speed_[i] += speed_fraction * (target_speed - speed_[i]);
    if (unit_(rng_) < flip_probability) {
      direction_[i] *= -1;
    }
    double next = positions_[i] + direction_[i] * speed_[i] * dt;
    // Reflect overshoot so a high-commitment reactor keeps moving at an edge.
    if (next > limit.upper) {
      next = limit.upper - (next - limit.upper);
      direction_[i] = -1;
    } else if (next < limit.lower) {
      next = limit.lower + (limit.lower - next);
      direction_[i] = 1;
    }
    positions_[i] = std::clamp(next, limit.lower, limit.upper);
  }
}

}  // namespace arm_control
