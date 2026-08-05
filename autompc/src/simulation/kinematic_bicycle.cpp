#include "autompc/simulation/kinematic_bicycle.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace autompc {
namespace {

void validateOptions(const SimulationOptions& options) {
    if (!std::isfinite(options.dt) || options.dt <= 0.0 ||
        !std::isfinite(options.wheelbase) || options.wheelbase <= 0.0 ||
        !std::isfinite(options.max_velocity) || options.max_velocity < 0.0 ||
        !std::isfinite(options.max_acceleration) ||
            options.max_acceleration < 0.0 ||
        !std::isfinite(options.max_deceleration) ||
            options.max_deceleration < 0.0 ||
        !std::isfinite(options.max_steering) || options.max_steering < 0.0 ||
        !std::isfinite(options.max_steering_rate) ||
            options.max_steering_rate < 0.0 ||
        !std::isfinite(options.max_reverse_velocity) ||
            options.max_reverse_velocity < 0.0) {
        throw std::invalid_argument("invalid kinematic bicycle simulation options");
    }
}

double clampFinite(double value, double lower, double upper) {
    if (!std::isfinite(value)) return lower;
    return std::clamp(value, lower, upper);
}

}  // namespace

KinematicBicycleSimulator::KinematicBicycleSimulator(
    const State& initial, SimulationOptions options)
    : state_(initial), options_(options) {
    validateOptions(options_);
    if (!std::isfinite(state_.x) || !std::isfinite(state_.y) ||
        !std::isfinite(state_.theta) || !std::isfinite(state_.v)) {
        throw std::invalid_argument("initial state must be finite");
    }
    const double minimum_velocity = options_.allow_reverse
        ? -options_.max_reverse_velocity : 0.0;
    state_.v = clampFinite(state_.v, minimum_velocity,
                           options_.max_velocity);
}

State KinematicBicycleSimulator::step(const Control& command) {
    const double minimum_velocity = options_.allow_reverse
        ? -options_.max_reverse_velocity : 0.0;
    const double target_velocity =
        clampFinite(command.velocity, minimum_velocity, options_.max_velocity);
    const double velocity_delta = target_velocity - state_.v;
    const double acceleration_limit = velocity_delta >= 0.0
        ? options_.max_acceleration : options_.max_deceleration;
    const double max_velocity_delta = acceleration_limit * options_.dt;
    const double next_velocity = state_.v +
        std::clamp(velocity_delta, -max_velocity_delta, max_velocity_delta);

    const double target_steering = clampFinite(
        command.steering, -options_.max_steering, options_.max_steering);
    const double steering_delta = target_steering - steering_;
    const double max_steering_delta =
        options_.max_steering_rate * options_.dt;
    const double next_steering = steering_ + std::clamp(
        steering_delta, -max_steering_delta, max_steering_delta);

    // Midpoint integration reduces the visible discretization error when both
    // velocity and steering are changing under their actuator limits.
    const double velocity_midpoint = 0.5 * (state_.v + next_velocity);
    const double steering_midpoint = 0.5 * (steering_ + next_steering);
    State next = state_;
    next.x += velocity_midpoint * std::cos(state_.theta) * options_.dt;
    next.y += velocity_midpoint * std::sin(state_.theta) * options_.dt;
    next.theta += velocity_midpoint / options_.wheelbase *
                  std::tan(steering_midpoint) * options_.dt;
    next.v = next_velocity;

    state_ = next;
    steering_ = next_steering;
    return state_;
}

void KinematicBicycleSimulator::reset(const State& state) {
    if (!std::isfinite(state.x) || !std::isfinite(state.y) ||
        !std::isfinite(state.theta) || !std::isfinite(state.v)) {
        throw std::invalid_argument("state must be finite");
    }
    state_ = state;
    const double minimum_velocity = options_.allow_reverse
        ? -options_.max_reverse_velocity : 0.0;
    state_.v = clampFinite(state_.v, minimum_velocity,
                           options_.max_velocity);
    steering_ = 0.0;
}

}  // namespace autompc
