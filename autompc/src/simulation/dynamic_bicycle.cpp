#include "autompc/simulation/dynamic_bicycle.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace autompc {
namespace {

void validateOptions(const DynamicBicycleOptions& options) {
    if (!std::isfinite(options.dt) || options.dt <= 0.0 ||
        !std::isfinite(options.wheelbase) || options.wheelbase <= 0.0 ||
        !std::isfinite(options.front_axle) || options.front_axle <= 0.0 ||
        !std::isfinite(options.rear_axle) || options.rear_axle <= 0.0 ||
        !std::isfinite(options.mass) || options.mass <= 0.0 ||
        !std::isfinite(options.yaw_inertia) || options.yaw_inertia <= 0.0 ||
        !std::isfinite(options.front_cornering_stiffness) ||
        options.front_cornering_stiffness < 0.0 ||
        !std::isfinite(options.rear_cornering_stiffness) ||
        options.rear_cornering_stiffness < 0.0 ||
        !std::isfinite(options.max_velocity) || options.max_velocity < 0.0 ||
        !std::isfinite(options.max_acceleration) ||
        options.max_acceleration < 0.0 ||
        !std::isfinite(options.max_deceleration) ||
        options.max_deceleration < 0.0 ||
        !std::isfinite(options.max_steering) || options.max_steering < 0.0 ||
        !std::isfinite(options.max_steering_rate) ||
        options.max_steering_rate < 0.0 ||
        !std::isfinite(options.max_reverse_velocity) ||
        options.max_reverse_velocity < 0.0 ||
        std::abs((options.front_axle + options.rear_axle) -
                 options.wheelbase) > 1e-6) {
        throw std::invalid_argument("invalid dynamic bicycle options");
    }
}

double clampFinite(double value, double lower, double upper) {
    if (!std::isfinite(value)) return lower;
    return std::clamp(value, lower, upper);
}

void validateState(const State& state) {
    if (!std::isfinite(state.x) || !std::isfinite(state.y) ||
        !std::isfinite(state.theta) || !std::isfinite(state.v)) {
        throw std::invalid_argument("state must be finite");
    }
}

}  // namespace

DynamicBicycleSimulator::DynamicBicycleSimulator(
    const State& initial, DynamicBicycleOptions options)
    : state_(initial), options_(options) {
    validateOptions(options_);
    validateState(state_);
    const double minimum_velocity = options_.allow_reverse
        ? -options_.max_reverse_velocity : 0.0;
    state_.v = clampFinite(state_.v, minimum_velocity,
                           options_.max_velocity);
}

State DynamicBicycleSimulator::step(const Control& command) {
    const double minimum_velocity = options_.allow_reverse
        ? -options_.max_reverse_velocity : 0.0;
    const double target_velocity = clampFinite(
        command.velocity, minimum_velocity, options_.max_velocity);
    const double velocity_delta = target_velocity - state_.v;
    const double acceleration_limit = velocity_delta >= 0.0
        ? options_.max_acceleration : options_.max_deceleration;
    const double max_velocity_delta = acceleration_limit * options_.dt;
    const double next_velocity = state_.v + std::clamp(
        velocity_delta, -max_velocity_delta, max_velocity_delta);

    const double target_steering = clampFinite(
        command.steering, -options_.max_steering, options_.max_steering);
    const double steering_delta = target_steering - steering_;
    const double max_steering_delta =
        options_.max_steering_rate * options_.dt;
    const double next_steering = steering_ + std::clamp(
        steering_delta, -max_steering_delta, max_steering_delta);

    // Linear tire model around the current operating point.  The speed floor
    // avoids singular slip-angle calculations at standstill while preserving
    // the correct sign for reverse motion.
    const double speed_sign = state_.v < 0.0 ? -1.0 : 1.0;
    const double longitudinal_speed = speed_sign *
        std::max(std::abs(0.5 * (state_.v + next_velocity)), 0.1);
    const double beta_front = (lateral_velocity_ +
                               options_.front_axle * yaw_rate_) /
                              longitudinal_speed;
    const double beta_rear = (lateral_velocity_ -
                              options_.rear_axle * yaw_rate_) /
                             longitudinal_speed;
    const double front_force = options_.front_cornering_stiffness *
        (next_steering - beta_front);
    const double rear_force = -options_.rear_cornering_stiffness * beta_rear;
    const double lateral_acceleration =
        (front_force * std::cos(next_steering) + rear_force) /
        options_.mass;
    const double yaw_acceleration =
        (options_.front_axle * front_force * std::cos(next_steering) -
         options_.rear_axle * rear_force) / options_.yaw_inertia;

    const double next_lateral_velocity = lateral_velocity_ +
        lateral_acceleration * options_.dt;
    const double next_yaw_rate = yaw_rate_ +
        yaw_acceleration * options_.dt;
    const double velocity_midpoint = 0.5 * (state_.v + next_velocity);
    const double lateral_midpoint =
        0.5 * (lateral_velocity_ + next_lateral_velocity);
    const double yaw_rate_midpoint = 0.5 * (yaw_rate_ + next_yaw_rate);
    State next = state_;
    next.x += (velocity_midpoint * std::cos(state_.theta) -
               lateral_midpoint * std::sin(state_.theta)) * options_.dt;
    next.y += (velocity_midpoint * std::sin(state_.theta) +
               lateral_midpoint * std::cos(state_.theta)) * options_.dt;
    next.theta += yaw_rate_midpoint * options_.dt;
    next.v = next_velocity;

    state_ = next;
    steering_ = next_steering;
    lateral_velocity_ = next_lateral_velocity;
    yaw_rate_ = next_yaw_rate;
    return state_;
}

void DynamicBicycleSimulator::reset(const State& state) {
    validateState(state);
    state_ = state;
    const double minimum_velocity = options_.allow_reverse
        ? -options_.max_reverse_velocity : 0.0;
    state_.v = clampFinite(state_.v, minimum_velocity,
                           options_.max_velocity);
    steering_ = 0.0;
    lateral_velocity_ = 0.0;
    yaw_rate_ = 0.0;
}

void DynamicBicycleSimulator::setSteering(double steering) {
    steering_ = clampFinite(steering, -options_.max_steering,
                            options_.max_steering);
}

}  // namespace autompc
