#include "robotnav/mppi_local_planner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include "robotnav/dynamic_obstacle_prediction.h"

namespace robotnav {
namespace {

double normalizeAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

double clampFinite(double value, double lower, double upper) {
    if (!std::isfinite(value)) return lower;
    return std::clamp(value, lower, upper);
}

autompc::TrajectoryPoint closestReference(
    const autompc::Trajectory& trajectory,
    const autompc::State& state) {
    autompc::TrajectoryPoint best = trajectory.front();
    double best_distance = std::numeric_limits<double>::max();
    for (const auto& point : trajectory) {
        const double dx = point.x - state.x;
        const double dy = point.y - state.y;
        const double squared_distance = dx * dx + dy * dy;
        if (squared_distance < best_distance) {
            best_distance = squared_distance;
            best = point;
        }
    }
    return best;
}

autompc::State rolloutStep(const autompc::State& state,
                           double& steering,
                           const autompc::Control& command,
                           const autompc::SimulationOptions& options,
                           double dt) {
    const double minimum_velocity = options.allow_reverse
        ? -options.max_reverse_velocity : 0.0;
    const double target_velocity = clampFinite(
        command.velocity, minimum_velocity, options.max_velocity);
    const double velocity_delta = target_velocity - state.v;
    const double acceleration_limit = velocity_delta >= 0.0
        ? options.max_acceleration
        : options.max_deceleration;
    const double max_velocity_delta = std::max(0.0, acceleration_limit) * dt;
    const double next_velocity = state.v + std::clamp(
        velocity_delta, -max_velocity_delta, max_velocity_delta);

    const double target_steering = clampFinite(
        command.steering, -options.max_steering, options.max_steering);
    const double steering_delta = target_steering - steering;
    const double max_steering_delta =
        std::max(0.0, options.max_steering_rate) * dt;
    const double next_steering = steering + std::clamp(
        steering_delta, -max_steering_delta, max_steering_delta);

    const double velocity_midpoint = 0.5 * (state.v + next_velocity);
    const double steering_midpoint = 0.5 * (steering + next_steering);
    autompc::State next = state;
    next.x += velocity_midpoint * std::cos(state.theta) * dt;
    next.y += velocity_midpoint * std::sin(state.theta) * dt;
    if (options.wheelbase > 0.0) {
        next.theta += velocity_midpoint / options.wheelbase *
                      std::tan(steering_midpoint) * dt;
    }
    next.theta = normalizeAngle(next.theta);
    next.v = next_velocity;
    steering = next_steering;
    return next;
}

bool dynamicSegmentValid(const autompc::State& start,
                         const autompc::State& end,
                         double start_seconds,
                         double end_seconds,
                         const DynamicObstacleContext& context,
                         const MppiOptions& options,
                         double& minimum_clearance) {
    if (context.obstacles == nullptr || context.obstacles->empty()) {
        return true;
    }
    if (!std::isfinite(context.frame_period_seconds) ||
        context.frame_period_seconds <= 0.0 ||
        !std::isfinite(context.collision_margin) ||
        context.collision_margin < 0.0) {
        return false;
    }

    const int samples = std::max(2, options.dynamic_collision_samples);
    for (int index = 0; index <= samples; ++index) {
        const double ratio = static_cast<double>(index) /
                             static_cast<double>(samples);
        const double seconds = start_seconds +
                                ratio * (end_seconds - start_seconds);
        const double frame = context.predictionFrameAfter(seconds);
        if (!std::isfinite(frame)) return false;
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

double normalizedSquared(double value, double scale) {
    const double safe_scale = scale > 1e-9 ? scale : 1.0;
    return (value / safe_scale) * (value / safe_scale);
}

}  // namespace

MppiLocalPlanner::MppiLocalPlanner(
    const autoplanner::CollisionChecker& collision_checker,
    autompc::SimulationOptions simulation_options,
    MppiOptions options)
    : collision_checker_(collision_checker),
      simulation_options_(simulation_options),
      options_(options) {}

MppiDecision MppiLocalPlanner::computeCommand(
    const autompc::State& state,
    double current_steering,
    const autompc::Trajectory& trajectory,
    const autompc::Control& nominal_command,
    const DynamicObstacleContext& dynamic_context) const {
    MppiDecision decision;
    decision.score = std::numeric_limits<double>::max();
    decision.minimum_dynamic_clearance =
        std::numeric_limits<double>::infinity();
    if (trajectory.empty() ||
        !std::isfinite(simulation_options_.dt) ||
        simulation_options_.dt <= 0.0 ||
        !std::isfinite(simulation_options_.wheelbase) ||
        simulation_options_.wheelbase <= 0.0 ||
        !std::isfinite(options_.prediction_time) ||
        options_.prediction_time <= 0.0 || options_.horizon <= 0 ||
        options_.rollouts <= 0 ||
        !std::isfinite(options_.temperature) ||
        options_.temperature <= 0.0 ||
        !std::isfinite(options_.velocity_noise) ||
        options_.velocity_noise < 0.0 ||
        !std::isfinite(options_.steering_noise) ||
        options_.steering_noise < 0.0 ||
        options_.dynamic_collision_samples <= 0 ||
        !std::isfinite(options_.dynamic_clearance) ||
        options_.dynamic_clearance < 0.0 ||
        !std::isfinite(options_.dynamic_obstacle_margin) ||
        options_.dynamic_obstacle_margin < 0.0 ||
        !std::isfinite(options_.warm_start_blend) ||
        options_.warm_start_blend < 0.0 ||
        options_.warm_start_blend > 1.0 ||
        !std::isfinite(options_.target_effective_sample_ratio) ||
        options_.target_effective_sample_ratio <= 0.0 ||
        options_.target_effective_sample_ratio > 1.0 ||
        !std::isfinite(options_.sampling_adaptation_gain) ||
        options_.sampling_adaptation_gain < 0.0 ||
        !std::isfinite(options_.minimum_noise_scale) ||
        options_.minimum_noise_scale <= 0.0 ||
        !std::isfinite(options_.maximum_noise_scale) ||
        options_.maximum_noise_scale < options_.minimum_noise_scale) {
        return decision;
    }

    const double prediction_dt = options_.prediction_time /
                                 static_cast<double>(options_.horizon);
    if (!std::isfinite(prediction_dt) || prediction_dt <= 0.0) {
        return decision;
    }

    const double velocity_scale = std::max(
        simulation_options_.max_velocity,
        simulation_options_.max_reverse_velocity) > 0.0
        ? std::max(simulation_options_.max_velocity,
                   simulation_options_.max_reverse_velocity) : 1.0;
    const double minimum_velocity = simulation_options_.allow_reverse
        ? -simulation_options_.max_reverse_velocity : 0.0;
    const double steering_scale = simulation_options_.max_steering > 0.0
        ? simulation_options_.max_steering : 1.0;
    const autompc::Control nominal{
        clampFinite(nominal_command.velocity, minimum_velocity,
                    simulation_options_.max_velocity),
        clampFinite(nominal_command.steering,
                    -simulation_options_.max_steering,
                    simulation_options_.max_steering)};

    struct RolloutResult {
        bool valid = false;
        double cost = 0.0;
        autompc::Control first_command;
        std::size_t dynamic_collision_rejections = 0;
        double minimum_dynamic_clearance =
            std::numeric_limits<double>::infinity();
    };

    const auto rollout_count = static_cast<std::size_t>(options_.rollouts);
    const auto horizon = static_cast<std::size_t>(options_.horizon);
    std::vector<autompc::Control> base_controls(horizon, nominal);
    double sampling_noise_scale = 1.0;
    {
        std::lock_guard<std::mutex> lock(warm_start_mutex_);
        sampling_noise_scale = std::clamp(
            sampling_noise_scale_, options_.minimum_noise_scale,
            options_.maximum_noise_scale);
        if (options_.warm_start &&
            previous_optimal_controls_.size() == horizon) {
            decision.warm_started = true;
            for (std::size_t step = 0; step < horizon; ++step) {
                const auto source = std::min(step + 1, horizon - 1);
                const auto& previous = previous_optimal_controls_[source];
                base_controls[step].velocity =
                    options_.warm_start_blend * previous.velocity +
                    (1.0 - options_.warm_start_blend) * nominal.velocity;
                base_controls[step].steering =
                    options_.warm_start_blend * previous.steering +
                    (1.0 - options_.warm_start_blend) * nominal.steering;
            }
        }
    }
    decision.sampling_noise_scale = sampling_noise_scale;
    std::vector<autompc::Control> sampled_controls(rollout_count * horizon);
    std::mt19937 generator(options_.random_seed);
    std::normal_distribution<double> velocity_distribution(
        0.0, options_.velocity_noise * sampling_noise_scale);
    std::normal_distribution<double> steering_distribution(
        0.0, options_.steering_noise * sampling_noise_scale);
    const double desired_clearance = std::max(
        options_.dynamic_clearance,
        std::max(options_.dynamic_obstacle_margin,
                 dynamic_context.collision_margin));

    // Generate noise serially in the historical order. This keeps the
    // sampled controls reproducible even when rollout evaluation is parallel.
    for (std::size_t rollout = 0; rollout < rollout_count; ++rollout) {
        for (std::size_t step = 0; step < horizon; ++step) {
            const bool baseline = rollout == 0;
            const auto& base = base_controls[step];
            sampled_controls[rollout * horizon + step] = {
                clampFinite(
                    base.velocity +
                        (baseline ? 0.0 : velocity_distribution(generator)),
                    minimum_velocity, simulation_options_.max_velocity),
                clampFinite(
                    base.steering +
                        (baseline ? 0.0 : steering_distribution(generator)),
                    -simulation_options_.max_steering,
                    simulation_options_.max_steering)};
        }
    }

    std::vector<RolloutResult> rollout_results(rollout_count);
#ifdef ROBOTNAV_HAS_OPENMP
#pragma omp parallel for schedule(static)
#endif
    for (int rollout = 0; rollout < options_.rollouts; ++rollout) {
        auto& result = rollout_results[static_cast<std::size_t>(rollout)];
        autompc::State predicted = state;
        double predicted_steering = current_steering;
        autompc::Control previous = nominal;
        bool valid = true;
        double elapsed_seconds = 0.0;
        for (int step = 0; step < options_.horizon; ++step) {
            const autompc::Control command = sampled_controls[
                static_cast<std::size_t>(rollout) * horizon +
                static_cast<std::size_t>(step)];
            if (step == 0) result.first_command = command;

            const auto next = rolloutStep(
                predicted, predicted_steering, command,
                simulation_options_, prediction_dt);
            if (!collision_checker_.isPoseSegmentValid(
                    {predicted.x, predicted.y, predicted.theta},
                    {next.x, next.y, next.theta})) {
                valid = false;
                break;
            }

            double candidate_clearance =
                std::numeric_limits<double>::infinity();
            if (!dynamicSegmentValid(
                    predicted, next, elapsed_seconds,
                    elapsed_seconds + prediction_dt, dynamic_context,
                    options_, candidate_clearance)) {
                result.minimum_dynamic_clearance = std::min(
                    result.minimum_dynamic_clearance, candidate_clearance);
                valid = false;
                ++result.dynamic_collision_rejections;
                break;
            }
            result.minimum_dynamic_clearance = std::min(
                result.minimum_dynamic_clearance, candidate_clearance);

            const auto reference = closestReference(trajectory, next);
            result.cost += options_.trajectory_weight *
                normalizedSquared(std::hypot(
                    next.x - reference.x, next.y - reference.y), 1.0);
            result.cost += options_.heading_weight * normalizedSquared(
                normalizeAngle(reference.theta - next.theta), 1.0);
            result.cost += options_.velocity_weight * normalizedSquared(
                next.v - reference.v, velocity_scale);
            result.cost += options_.control_weight * (
                normalizedSquared(command.velocity - nominal.velocity,
                                 velocity_scale) +
                normalizedSquared(command.steering - nominal.steering,
                                  steering_scale));
            result.cost += options_.control_rate_weight * (
                normalizedSquared(command.velocity - previous.velocity,
                                  velocity_scale) +
                normalizedSquared(command.steering - previous.steering,
                                  steering_scale));
            if (std::isfinite(candidate_clearance) &&
                desired_clearance > 0.0 &&
                candidate_clearance < desired_clearance) {
                result.cost += options_.dynamic_obstacle_weight *
                    normalizedSquared(desired_clearance - candidate_clearance,
                                      desired_clearance);
            }

            previous = command;
            predicted = next;
            elapsed_seconds += prediction_dt;
        }
        if (!valid) continue;

        const auto terminal_reference = closestReference(trajectory, predicted);
        result.cost += options_.trajectory_weight * 2.0 *
            normalizedSquared(std::hypot(
                predicted.x - terminal_reference.x,
                predicted.y - terminal_reference.y), 1.0);
        result.cost += options_.heading_weight * 2.0 * normalizedSquared(
            normalizeAngle(terminal_reference.theta - predicted.theta), 1.0);
        result.valid = true;
    }

    autompc::Control best_first_command = nominal;
    std::size_t best_rollout = rollout_count;
    for (std::size_t index = 0; index < rollout_results.size(); ++index) {
        const auto& result = rollout_results[index];
        decision.dynamic_collision_rejections +=
            result.dynamic_collision_rejections;
        decision.minimum_dynamic_clearance = std::min(
            decision.minimum_dynamic_clearance,
            result.minimum_dynamic_clearance);
        if (!result.valid) continue;
        ++decision.feasible_rollouts;
        if (result.cost < decision.score) {
            decision.score = result.cost;
            best_first_command = result.first_command;
            best_rollout = index;
        }
    }

    if (decision.feasible_rollouts == 0) return decision;

    const double minimum_cost = decision.score;
    double weight_sum = 0.0;
    double squared_weight_sum = 0.0;
    std::vector<autompc::Control> weighted_controls(horizon);
    for (std::size_t rollout_index = 0;
         rollout_index < rollout_results.size(); ++rollout_index) {
        const auto& rollout = rollout_results[rollout_index];
        if (!rollout.valid) continue;
        const double exponent = -(rollout.cost - minimum_cost) /
                                options_.temperature;
        const double weight = std::exp(std::max(-700.0, exponent));
        if (!std::isfinite(weight)) continue;
        weight_sum += weight;
        squared_weight_sum += weight * weight;
        for (std::size_t step = 0; step < horizon; ++step) {
            const auto& command = sampled_controls[
                rollout_index * horizon + step];
            weighted_controls[step].velocity += weight * command.velocity;
            weighted_controls[step].steering += weight * command.steering;
        }
    }
    if (weight_sum <= 0.0 || !std::isfinite(weight_sum) ||
        squared_weight_sum <= 0.0 || !std::isfinite(squared_weight_sum)) {
        return decision;
    }
    decision.effective_sample_size =
        weight_sum * weight_sum / squared_weight_sum;
    decision.effective_sample_ratio = std::clamp(
        decision.effective_sample_size /
            static_cast<double>(decision.feasible_rollouts),
        0.0, 1.0);

    std::vector<autompc::Control> optimized_controls(horizon);
    for (std::size_t step = 0; step < horizon; ++step) {
        optimized_controls[step].velocity = clampFinite(
            weighted_controls[step].velocity / weight_sum,
            minimum_velocity, simulation_options_.max_velocity);
        optimized_controls[step].steering = clampFinite(
            weighted_controls[step].steering / weight_sum,
            -simulation_options_.max_steering,
            simulation_options_.max_steering);
    }
    const auto& aggregated_command = optimized_controls.front();

    // The weighted mean is not guaranteed to remain inside the non-convex
    // collision-free set. Keep the best sampled first action as a safety
    // fallback if the aggregate itself crosses an obstacle.
    double aggregated_steering = current_steering;
    const auto aggregated_next = rolloutStep(
        state, aggregated_steering, aggregated_command,
        simulation_options_, prediction_dt);
    double aggregate_clearance = std::numeric_limits<double>::infinity();
    const bool aggregate_valid =
        collision_checker_.isPoseSegmentValid(
            {state.x, state.y, state.theta},
            {aggregated_next.x, aggregated_next.y, aggregated_next.theta}) &&
        dynamicSegmentValid(
            state, aggregated_next, 0.0, prediction_dt, dynamic_context,
            options_, aggregate_clearance);
    decision.command = aggregate_valid
        ? aggregated_command
        : best_first_command;
    decision.feasible = true;
    if (options_.warm_start && best_rollout < rollout_count) {
        std::lock_guard<std::mutex> lock(warm_start_mutex_);
        if (aggregate_valid) {
            previous_optimal_controls_ = std::move(optimized_controls);
        } else {
            const auto begin = sampled_controls.begin() +
                static_cast<std::ptrdiff_t>(best_rollout * horizon);
            previous_optimal_controls_.assign(
                begin, begin + static_cast<std::ptrdiff_t>(horizon));
        }
    }
    if (options_.adaptive_sampling) {
        const double adjustment = std::exp(
            options_.sampling_adaptation_gain *
            (decision.effective_sample_ratio -
             options_.target_effective_sample_ratio));
        const double next_scale = std::clamp(
            sampling_noise_scale * adjustment,
            options_.minimum_noise_scale,
            options_.maximum_noise_scale);
        std::lock_guard<std::mutex> lock(warm_start_mutex_);
        sampling_noise_scale_ = next_scale;
    }
    return decision;
}

void MppiLocalPlanner::resetWarmStart() {
    std::lock_guard<std::mutex> lock(warm_start_mutex_);
    previous_optimal_controls_.clear();
}

}  // namespace robotnav
