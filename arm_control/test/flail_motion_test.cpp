#include "arm_control/flail_motion.hpp"

#include <array>
#include <cmath>
#include <stdexcept>

namespace {

void require(bool condition) {
  if (!condition) {
    throw std::runtime_error("flail motion test failed");
  }
}

}  // namespace

int main() {
  using arm_control::FlailMotion;
  using arm_control::JointLimits;
  std::array<JointLimits, arm_control::kJointCount> limits{};
  limits.fill({-0.1, 0.1, 1.0});
  arm_control::FlailConfig config;
  config.reaction_gain = 2.0;

  FlailMotion model(limits, config, 42);
  model.initialize({0.0, 0.0, 0.0, 0.0});
  model.step(0.02, 0.02);
  for (double position : model.positions()) {
    require(position == 0.0);  // No startup motion before a slider changes.
  }

  // A steady actor move produces high commitment and bounded joint motion.
  for (int n = 1; n <= 5; ++n) {
    const double t = 0.02 + n * 0.05;
    model.set_actor_target(0, n * 0.02, 0.02, t);
    model.step(t, 0.05);
  }
  require(model.activity() > 0.0);
  require(model.commitment() > 0.8);
  require(model.positions()[0] > 0.0);

  // Alternating equal changes have activity but little directional commitment.
  for (int n = 1; n <= 20; ++n) {
    const double t = 1.0 + n * 0.05;
    const double delta = n % 2 == 0 ? -0.02 : 0.02;
    model.set_actor_target(0, 0.08 + delta, delta, t);
    model.step(t, 0.05);
  }
  require(model.activity() > 0.0);
  require(model.commitment() < 0.2);
  for (double position : model.positions()) {
    require(std::isfinite(position));
    require(position >= -0.1 && position <= 0.1);
  }

  // A fixed seed makes the same input sequence reproducible.
  FlailMotion first(limits, config, 17);
  FlailMotion second(limits, config, 17);
  first.initialize({0.0, 0.0, 0.0, 0.0});
  second.initialize({0.0, 0.0, 0.0, 0.0});
  for (int n = 1; n <= 100; ++n) {
    const double t = n * 0.02;
    if (n % 5 == 0) {
      first.set_actor_target(0, 0.05, 0.01, t);
      second.set_actor_target(0, 0.05, 0.01, t);
    }
    first.step(t, 0.02);
    second.step(t, 0.02);
    require(first.positions() == second.positions());
    for (double position : first.positions()) {
      require(position >= -0.1 && position <= 0.1);
    }
  }
}
