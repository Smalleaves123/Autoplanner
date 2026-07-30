#include "robotnav/dwa_local_planner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace robotnav {
namespace {

double normalizeAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

double clampFinite(double value, double lower, double upper) {
    if (!std::isfinite(value)) return lower;
    return std::clamp(value, lower, upper);
}

std::vector<double> sampleRange(double lower, double upper, int samples) {
    if (samples <= 1 || std::abs(upper - lower) < 1e-12) {
        return {0.5 * (lower + upper)};
    }
    std::vector<double> values;
    values.reserve(static_cast<std::size_t>(samples));
    for (int index = 0; index < samples; ++index) {
        const double t = static_cast<double>(index) /
                         static_cast<double>(samples - 1);
        values.push_back(lower + t * (upper - lower));
    }
    return values;
}

void addUnique(std::vector<double>& values, double value) {
    if (std::none_of(values.begin(), values.end(),
                     [value](double existing) {
                         return std::abs(existing - value) < 1e-9;
                     })) {
        values.push_back(value);
    }
}

autompc::TrajectoryPoint lookaheadReference(
    const autompc::Trajectory& trajectory,
    const autompc::State& state,
    double lookahead_distance) {
    std::size_t closest = 0;
    double best_distance = std::numeric_limits<double>::max();
    for (std::size_t index = 0; index < trajectory.size(); ++index) {
        const double distance = std::hypot(
            trajectory[index].x - state.x, trajectory[index].y - state.y);
        if (distance < best_distance) {
            best_distance = distance;
            closest = index;
        }
    }

    double accumulated = 0.0;
    for (std::size_t index = closest + 1; index < trajectory.size(); ++index) {
        accumulated += std::hypot(
            trajectory[index].x - trajectory[index - 1].x,
            trajectory[index].y - trajectory[index - 1].y);
        if (accumulated >= lookahead_distance) return trajectory[index];
    }
    return trajectory.back();
}

autompc::State rolloutStep(const autompc::State& state,
                           double& steering,
                           const autompc::Control& command,
                           const autompc::SimulationOptions& options) {
    const double target_velocity = clampFinite(
        command.velocity, 0.0, options.max_velocity);
    const double velocity_delta = target_velocity - state.v;
    const double acceleration_limit = velocity_delta >= 0.0
        ? options.max_acceleration
        : options.max_deceleration;
    const double max_velocity_delta = acceleration_limit * options.dt;
    const double next_velocity = state.v + std::clamp(
        velocity_delta, -max_velocity_delta, max_velocity_delta);

    const double target_steering = clampFinite(
        command.steering, -options.max_steering, options.max_steering);
    const double steering_delta = target_steering - steering;
    const double max_steering_delta = options.max_steering_rate * options.dt;
    const double next_steering = steering + std::clamp(
        steering_delta, -max_steering_delta, max_steering_delta);

    const double velocity_midpoint = 0.5 * (state.v + next_velocity);
    const double steering_midpoint = 0.5 * (steering + next_steering);
    autompc::State next = state;
    next.x += velocity_midpoint * std::cos(state.theta) * options.dt;
    next.y += velocity_midpoint * std::sin(state.theta) * options.dt;
    next.theta += velocity_midpoint / options.wheelbase *
                  std::tan(steering_midpoint) * options.dt;
    next.theta = normalizeAngle(next.theta);
    next.v = next_velocity;
    steering = next_steering;
    return next;
}

double scoreEndpoint(const autompc::State& endpoint,
                     const autompc::Trajectory& trajectory,
                     const autompc::State& start,
                     const autompc::Control& command,
                     const autompc::Control& nominal_command,
                     const autompc::SimulationOptions& simulation_options,
                     const DwaOptions& options) {
    const double nominal_speed = std::max(
        nominal_command.velocity, trajectory.front().v);
    const auto reference = lookaheadReference(
        trajectory, start,
        std::max(0.5, nominal_speed * options.prediction_time));
    const double trajectory_error =
        std::hypot(endpoint.x - reference.x, endpoint.y - reference.y);
    const double heading_error =
        std::abs(normalizeAngle(reference.theta - endpoint.theta));
    const double velocity_scale =
        simulation_options.max_velocity > 0.0
            ? simulation_options.max_velocity
            : 1.0;
    const double steering_scale =
        simulation_options.max_steering > 0.0
            ? simulation_options.max_steering
            : 1.0;
    const double command_error =
        std::abs(command.velocity - nominal_command.velocity) / velocity_scale +
        std::abs(command.steering - nominal_command.steering) / steering_scale;
    const double speed_reward = endpoint.v / velocity_scale;

    return options.trajectory_weight * trajectory_error +
           options.heading_weight * heading_error +
           options.command_weight * command_error -
           options.speed_weight * speed_reward;
}

}  // namespace

DwaLocalPlanner::DwaLocalPlanner(
    const autoplanner::CollisionChecker& collision_checker,
    autompc::SimulationOptions simulation_options,
    DwaOptions options)
    : collision_checker_(collision_checker),
      simulation_options_(simulation_options),
      options_(options) {}

DwaDecision DwaLocalPlanner::computeCommand(
    const autompc::State& state,
    double current_steering,
    const autompc::Trajectory& trajectory,
    const autompc::Control& nominal_command) const {
    if (trajectory.empty() || simulation_options_.dt <= 0.0 ||
        options_.prediction_time <= 0.0) {
        return {};
    }

    const double min_velocity = std::max(
        0.0, state.v -
                 simulation_options_.max_deceleration * simulation_options_.dt);
    const double max_velocity = std::min(
        simulation_options_.max_velocity,
        state.v + simulation_options_.max_acceleration * simulation_options_.dt);
    const double min_steering = std::max(
        -simulation_options_.max_steering,
        current_steering -
            simulation_options_.max_steering_rate * simulation_options_.dt);
    const double max_steering = std::min(
        simulation_options_.max_steering,
        current_steering +
            simulation_options_.max_steering_rate * simulation_options_.dt);

    auto velocity_samples = sampleRange(
        min_velocity, max_velocity, options_.velocity_samples);
    auto steering_samples = sampleRange(
        min_steering, max_steering, options_.steering_samples);
    addUnique(velocity_samples, clampFinite(
        nominal_command.velocity, min_velocity, max_velocity));
    addUnique(steering_samples, clampFinite(
        nominal_command.steering, min_steering, max_steering));

    const int rollout_steps = std::max(
        1, static_cast<int>(
               std::ceil(options_.prediction_time / simulation_options_.dt)));
    DwaDecision best;
    best.score = std::numeric_limits<double>::max();

    for (const double velocity : velocity_samples) {
        for (const double steering_command : steering_samples) {
            const autompc::Control candidate{velocity, steering_command};
            autompc::State predicted = state;
            double predicted_steering = current_steering;
            bool collision_free = true;
            for (int step = 0; step < rollout_steps; ++step) {
                const auto previous = predicted;
                predicted = rolloutStep(
                    predicted, predicted_steering, candidate,
                    simulation_options_);
                if (!collision_checker_.isPoseSegmentValid(
                        {previous.x, previous.y, previous.theta},
                        {predicted.x, predicted.y, predicted.theta})) {
                    collision_free = false;
                    break;
                }
            }
            if (!collision_free) continue;
            const double score = scoreEndpoint(
                predicted, trajectory, state, candidate, nominal_command,
                simulation_options_, options_);
            if (score < best.score) {
                best.feasible = true;
                best.command = candidate;
                best.score = score;
            }
        }
    }

    return best;
}

}  // namespace robotnav
