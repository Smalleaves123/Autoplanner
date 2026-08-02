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

bool dynamicSegmentValid(const autompc::State& start,
                         const autompc::State& end,
                         double start_seconds,
                         double end_seconds,
                         const DwaDynamicContext& context,
                         const DwaOptions& options,
                         double& minimum_clearance) {
    if (context.obstacles == nullptr || context.obstacles->empty()) {
        return true;
    }
    if (!std::isfinite(context.frame_period_seconds) ||
        context.frame_period_seconds <= 0.0) {
        return false;
    }

    const int samples = std::max(2, options.dynamic_collision_samples);
    for (int index = 0; index <= samples; ++index) {
        const double ratio = static_cast<double>(index) /
                             static_cast<double>(samples);
        const double seconds = start_seconds +
                                ratio * (end_seconds - start_seconds);
        const double frame = static_cast<double>(context.current_frame) +
                             seconds / context.frame_period_seconds;
        const autoplanner::Point2d position{
            start.x + ratio * (end.x - start.x),
            start.y + ratio * (end.y - start.y)};
        const double clearance = predictedObstacleClearance(
            *context.obstacles, position, frame);
        if (std::isfinite(clearance)) {
            minimum_clearance = std::min(minimum_clearance, clearance);
        }
        if (isPredictedCollision(
                *context.obstacles, position, frame,
                context.collision_margin)) {
            return false;
        }
    }
    return true;
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
    const autompc::Control& nominal_command,
    const DwaDynamicContext& dynamic_context) const {
    if (trajectory.empty() || simulation_options_.dt <= 0.0 ||
        options_.prediction_time <= 0.0 ||
        options_.dynamic_collision_samples <= 0) {
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
    best.minimum_dynamic_clearance =
        std::numeric_limits<double>::infinity();

    for (const double velocity : velocity_samples) {
        for (const double steering_command : steering_samples) {
            const autompc::Control candidate{velocity, steering_command};
            autompc::State predicted = state;
            double predicted_steering = current_steering;
            bool collision_free = true;
            double candidate_clearance =
                std::numeric_limits<double>::infinity();
            double elapsed_seconds = 0.0;
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
                const double next_elapsed_seconds = elapsed_seconds +
                    simulation_options_.dt;
                if (!dynamicSegmentValid(
                        previous, predicted, elapsed_seconds,
                        next_elapsed_seconds, dynamic_context, options_,
                        candidate_clearance)) {
                    collision_free = false;
                    ++best.dynamic_collision_rejections;
                    break;
                }
                elapsed_seconds = next_elapsed_seconds;
            }
            if (!collision_free) continue;
            best.minimum_dynamic_clearance = std::min(
                best.minimum_dynamic_clearance, candidate_clearance);
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
