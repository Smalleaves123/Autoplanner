#include "autompc/simulation/differential_drive.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace autompc {
namespace {

void validateOptions(const DifferentialDriveOptions& options) {
    if (!std::isfinite(options.dt) || options.dt <= 0.0 ||
        !std::isfinite(options.track_width) || options.track_width <= 0.0 ||
        !std::isfinite(options.max_linear_velocity) ||
            options.max_linear_velocity < 0.0 ||
        !std::isfinite(options.max_reverse_velocity) ||
            options.max_reverse_velocity < 0.0 ||
        !std::isfinite(options.max_linear_acceleration) ||
            options.max_linear_acceleration < 0.0 ||
        !std::isfinite(options.max_linear_deceleration) ||
            options.max_linear_deceleration < 0.0 ||
        !std::isfinite(options.max_angular_velocity) ||
            options.max_angular_velocity < 0.0 ||
        !std::isfinite(options.max_angular_acceleration) ||
            options.max_angular_acceleration < 0.0 ||
        !std::isfinite(options.max_wheel_velocity) ||
            options.max_wheel_velocity < 0.0) {
        throw std::invalid_argument("invalid differential-drive options");
    }
}

void validateState(const State& state) {
    if (!std::isfinite(state.x) || !std::isfinite(state.y) ||
        !std::isfinite(state.theta) || !std::isfinite(state.v)) {
        throw std::invalid_argument("differential-drive state must be finite");
    }
}

double finiteOrZero(double value) {
    return std::isfinite(value) ? value : 0.0;
}

double rateLimited(double current, double target, double acceleration,
                   double deceleration, double dt) {
    const bool same_direction = current * target >= 0.0;
    const bool increasing_speed =
        same_direction && std::abs(target) > std::abs(current);
    const double limit = (increasing_speed ? acceleration : deceleration) * dt;
    return current + std::clamp(target - current, -limit, limit);
}

void constrainWheelVelocity(double track_width, double max_wheel_velocity,
                            double& linear_velocity,
                            double& angular_velocity) {
    const double half_track = 0.5 * track_width;
    const double left = linear_velocity - angular_velocity * half_track;
    const double right = linear_velocity + angular_velocity * half_track;
    const double peak = std::max(std::abs(left), std::abs(right));
    if (peak > max_wheel_velocity && peak > 0.0) {
        const double scale = max_wheel_velocity / peak;
        linear_velocity *= scale;
        angular_velocity *= scale;
    }
}

}  // namespace

DifferentialDriveSimulator::DifferentialDriveSimulator(
    const State& initial, DifferentialDriveOptions options)
    : state_(initial), options_(options) {
    validateOptions(options_);
    reset(initial);
}

State DifferentialDriveSimulator::step(
    const DifferentialDriveCommand& command) {
    const double minimum_velocity = options_.allow_reverse
        ? -options_.max_reverse_velocity : 0.0;
    double target_linear = std::clamp(
        finiteOrZero(command.linear_velocity), minimum_velocity,
        options_.max_linear_velocity);
    double target_angular = std::clamp(
        finiteOrZero(command.angular_velocity),
        -options_.max_angular_velocity, options_.max_angular_velocity);
    constrainWheelVelocity(options_.track_width, options_.max_wheel_velocity,
                           target_linear, target_angular);

    double next_linear = rateLimited(
        state_.v, target_linear, options_.max_linear_acceleration,
        options_.max_linear_deceleration, options_.dt);
    double next_angular = angular_velocity_ + std::clamp(
        target_angular - angular_velocity_,
        -options_.max_angular_acceleration * options_.dt,
        options_.max_angular_acceleration * options_.dt);
    constrainWheelVelocity(options_.track_width, options_.max_wheel_velocity,
                           next_linear, next_angular);

    const double linear_midpoint = 0.5 * (state_.v + next_linear);
    const double angular_midpoint =
        0.5 * (angular_velocity_ + next_angular);
    const double heading_midpoint =
        state_.theta + 0.5 * angular_midpoint * options_.dt;

    state_.x += linear_midpoint * std::cos(heading_midpoint) * options_.dt;
    state_.y += linear_midpoint * std::sin(heading_midpoint) * options_.dt;
    state_.theta += angular_midpoint * options_.dt;
    state_.v = next_linear;
    angular_velocity_ = next_angular;
    left_wheel_velocity_ = next_linear -
        0.5 * options_.track_width * next_angular;
    right_wheel_velocity_ = next_linear +
        0.5 * options_.track_width * next_angular;
    return state_;
}

void DifferentialDriveSimulator::reset(const State& state) {
    validateState(state);
    state_ = state;
    const double minimum_velocity = options_.allow_reverse
        ? -options_.max_reverse_velocity : 0.0;
    state_.v = std::clamp(state_.v, minimum_velocity,
                          options_.max_linear_velocity);
    state_.v = std::clamp(state_.v, -options_.max_wheel_velocity,
                          options_.max_wheel_velocity);
    angular_velocity_ = 0.0;
    left_wheel_velocity_ = state_.v;
    right_wheel_velocity_ = state_.v;
}

}  // namespace autompc
