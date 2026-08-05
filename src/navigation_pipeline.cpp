#include "robotnav/navigation_pipeline.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>

#include "autompc/controllers/controllers.h"
#ifdef AUTOMPC_HAS_EIGEN
#include "autompc/controllers/mpc_controller.h"
#endif
#include "autompc/trajectory/trajectory_generator.h"
#include "autoplanner/collision/footprint_collision_checker.h"
#include "autoplanner/collision/grid_collision_checker.h"
#include "autoplanner/costmap/costmap_2d.h"
#include "autoplanner/core/path.h"
#include "autoplanner/core/planner_factory.h"
#include "autoplanner/smoothing/curvature_constrained_smoother.h"
#include "autoplanner/smoothing/shortcut_smoother.h"

namespace robotnav {
namespace {

autompc::TrajectoryPoint closestReferencePoint(
    const autompc::Trajectory& trajectory,
    const autompc::State& state) {
    autompc::TrajectoryPoint best = trajectory.front();
    double best_distance = std::numeric_limits<double>::max();
    for (const auto& point : trajectory) {
        const double distance = std::hypot(point.x - state.x,
                                           point.y - state.y);
        if (distance < best_distance) {
            best_distance = distance;
            best = point;
        }
    }
    return best;
}

double headingError(const autompc::State& state,
                    const autompc::TrajectoryPoint& reference) {
    const double error = reference.theta - state.theta;
    return std::abs(std::atan2(std::sin(error), std::cos(error)));
}

double crossTrackError(const autompc::State& state,
                       const autompc::TrajectoryPoint& reference) {
    return std::hypot(reference.x - state.x, reference.y - state.y);
}

std::vector<autoplanner::Pose2d> makePoses(
    const autoplanner::Path2d& path) {
    std::vector<autoplanner::Pose2d> poses;
    poses.reserve(path.size());
    for (std::size_t i = 0; i < path.size(); ++i) {
        const std::size_t next = std::min(i + 1, path.size() - 1);
        const std::size_t previous = i == 0 ? i : i - 1;
        const double dx = path[next].x - path[previous].x;
        const double dy = path[next].y - path[previous].y;
        poses.push_back({path[i].x, path[i].y, std::atan2(dy, dx)});
    }
    return poses;
}

std::string escapeJsonString(const std::string& value) {
    std::string escaped;
    for (const char character : value) {
        switch (character) {
            case '"': escaped += "\\\""; break;
            case '\\': escaped += "\\\\"; break;
            case '\n': escaped += "\\n"; break;
            case '\r': escaped += "\\r"; break;
            case '\t': escaped += "\\t"; break;
            default: escaped += character; break;
        }
    }
    return escaped;
}

void writeJsonNumber(std::ofstream& output, double value) {
    if (std::isfinite(value)) output << value;
    else output << "null";
}

double configuredMaxCurvature(const PipelineConfig& config) {
    const double configured = config.trajectory_options.max_curvature;
    if (!config.enforce_kinematic_constraints) return configured;
    if (!std::isfinite(config.simulation_options.wheelbase) ||
        config.simulation_options.wheelbase <= 0.0 ||
        !std::isfinite(config.simulation_options.max_steering) ||
        config.simulation_options.max_steering < 0.0) {
        return -1.0;
    }
    const double vehicle_limit = std::tan(
        config.simulation_options.max_steering) /
        config.simulation_options.wheelbase;
    if (!std::isfinite(vehicle_limit) || vehicle_limit <= 0.0) return -1.0;
    return configured > 0.0 ? std::min(configured, vehicle_limit)
                            : vehicle_limit;
}

bool containsReverseMotion(const std::vector<int>& directions) {
    return std::any_of(directions.begin(), directions.end(),
                       [](int direction) { return direction < 0; });
}

bool trajectoryIsCollisionFree(
    const autompc::Trajectory& trajectory,
    const autoplanner::CollisionChecker& collision_checker) {
    if (trajectory.empty()) return false;
    for (std::size_t index = 0; index < trajectory.size(); ++index) {
        const auto& point = trajectory[index];
        if (!collision_checker.isPoseValid({point.x, point.y, point.theta})) {
            return false;
        }
        if (index > 0) {
            const auto& previous = trajectory[index - 1];
            if (!collision_checker.isPoseSegmentValid(
                    {previous.x, previous.y, previous.theta},
                    {point.x, point.y, point.theta})) {
                return false;
            }
        }
    }
    return true;
}

}  // namespace

PipelineResult NavigationPipeline::run(
    const autoplanner::GridMap& map,
    const autoplanner::Point2i& start,
    const autoplanner::Point2i& goal,
    const PipelineConfig& config) const {
    PipelineResult result;
    result.metrics.status = StatusCode::InternalError;
    result.metrics.footprint = config.footprint;
    result.metrics.smoother = config.smoother;
    result.metrics.local_planner = config.local_planner;

    auto fail = [&result](StatusCode status, const std::string& message) {
        result.metrics.status = status;
        result.message = message;
        return result;
    };

    if (map.isEmpty()) {
        return fail(StatusCode::InvalidConfiguration, "map is empty");
    }
    if (!map.isFree(start.x, start.y)) {
        return fail(StatusCode::InvalidStart,
                    "start cell is occupied or outside the map");
    }
    if (!map.isFree(goal.x, goal.y)) {
        return fail(StatusCode::InvalidGoal,
                    "goal cell is occupied or outside the map");
    }
    if (config.max_steps == 0 || config.simulation_options.dt <= 0.0) {
        return fail(StatusCode::InvalidConfiguration,
                    "pipeline step configuration is invalid");
    }
    if (!std::isfinite(config.trajectory_options.max_curvature) ||
        config.trajectory_options.max_curvature < 0.0) {
        return fail(StatusCode::InvalidConfiguration,
                    "trajectory curvature limit is invalid");
    }
    if (!std::isfinite(config.trajectory_options.max_reverse_velocity) ||
        config.trajectory_options.max_reverse_velocity < 0.0 ||
        !std::isfinite(config.simulation_options.max_reverse_velocity) ||
        config.simulation_options.max_reverse_velocity < 0.0) {
        return fail(StatusCode::InvalidConfiguration,
                    "reverse velocity limit is invalid");
    }
    const double max_trajectory_curvature = configuredMaxCurvature(config);
    if (max_trajectory_curvature < 0.0) {
        return fail(StatusCode::InvalidConfiguration,
                    "kinematic vehicle constraints are invalid");
    }
    if (config.local_planner != "none" && config.local_planner != "dwa" &&
        config.local_planner != "mppi") {
        return fail(StatusCode::InvalidConfiguration,
                    "unsupported local planner: " + config.local_planner);
    }
    if (config.local_planner == "dwa" &&
        (config.dwa_options.prediction_time <= 0.0 ||
         config.dwa_options.velocity_samples <= 0 ||
         config.dwa_options.steering_samples <= 0 ||
         config.dwa_options.dynamic_collision_samples <= 0 ||
         !std::isfinite(config.dwa_options.dynamic_obstacle_margin) ||
         config.dwa_options.dynamic_obstacle_margin < 0.0)) {
        return fail(StatusCode::InvalidConfiguration,
                    "invalid DWA local planner configuration");
    }
    if (config.local_planner == "mppi" &&
        (config.mppi_options.prediction_time <= 0.0 ||
         config.mppi_options.horizon <= 0 ||
         config.mppi_options.rollouts <= 0 ||
         !std::isfinite(config.mppi_options.temperature) ||
         config.mppi_options.temperature <= 0.0 ||
         !std::isfinite(config.mppi_options.velocity_noise) ||
         config.mppi_options.velocity_noise < 0.0 ||
         !std::isfinite(config.mppi_options.steering_noise) ||
         config.mppi_options.steering_noise < 0.0 ||
         config.mppi_options.dynamic_collision_samples <= 0 ||
         !std::isfinite(config.mppi_options.dynamic_clearance) ||
         config.mppi_options.dynamic_clearance < 0.0 ||
         !std::isfinite(config.mppi_options.dynamic_obstacle_margin) ||
         config.mppi_options.dynamic_obstacle_margin < 0.0)) {
        return fail(StatusCode::InvalidConfiguration,
                    "invalid MPPI local planner configuration");
    }

    autoplanner::RobotFootprint footprint;
    double circumscribed_radius = 0.0;
    if (config.footprint == "point") {
        footprint = autoplanner::RobotFootprint::circle(0.0);
    } else if (config.footprint == "circle" &&
               std::isfinite(config.robot_radius) &&
               config.robot_radius > 0.0) {
        footprint = autoplanner::RobotFootprint::circle(config.robot_radius);
        circumscribed_radius = config.robot_radius;
    } else if (config.footprint == "rectangle" &&
               std::isfinite(config.robot_length) &&
               std::isfinite(config.robot_width) &&
               config.robot_length > 0.0 && config.robot_width > 0.0) {
        footprint = autoplanner::RobotFootprint::rectangle(
            config.robot_length, config.robot_width);
        circumscribed_radius = std::hypot(
            0.5 * config.robot_length, 0.5 * config.robot_width);
    } else {
        return fail(StatusCode::InvalidConfiguration,
                    "invalid robot footprint configuration");
    }

    autoplanner::GridMap planning_map = map;
    if ((config.inflate_map || config.footprint != "point") &&
        circumscribed_radius > 0.0) {
        planning_map.inflateObstacles(circumscribed_radius);
    }
    if (config.footprint != "point" && !planning_map.isFree(start.x, start.y)) {
        return fail(StatusCode::InvalidStart,
                    "robot footprint makes the start cell invalid");
    }
    if (config.footprint != "point" && !planning_map.isFree(goal.x, goal.y)) {
        return fail(StatusCode::InvalidGoal,
                    "robot footprint makes the goal cell invalid");
    }

    autoplanner::Costmap2D costmap;
    const autoplanner::Costmap2D* costmap_ptr = nullptr;
    if (config.planner == "improved_astar") {
        costmap.buildFromGridMap(planning_map);
        costmap.inflateObstacles(
            config.robot_radius > 0.0 ? config.robot_radius
                                      : circumscribed_radius);
        costmap_ptr = &costmap;
    }

    auto planner = autoplanner::createPlanner(
        config.planner, config.planner_options, costmap_ptr);
    if (!planner) {
        return fail(StatusCode::InvalidConfiguration,
                    "unknown planner: " + config.planner);
    }

    result.planning = planner->plan(planning_map, start, goal);
    result.metrics.planning_time_ms = result.planning.planning_time_ms;
    result.metrics.path_length = result.planning.path_length;
    if (!result.planning.success || result.planning.path.empty()) {
        return fail(result.planning.statusCode(),
                    result.planning.message.empty()
                        ? "planner failed"
                        : result.planning.message);
    }

    std::unique_ptr<autoplanner::CollisionChecker> collision_checker;
    if (config.footprint == "point") {
        collision_checker = std::make_unique<autoplanner::GridCollisionChecker>(
            map);
    } else {
        collision_checker = std::make_unique<autoplanner::FootprintCollisionChecker>(
            map, footprint);
    }

    const auto pathIsValid = [&]() {
        if (config.footprint == "point") {
            return collision_checker->isPathValid(result.planning.path);
        }
        return collision_checker->isPosePathValid(
            makePoses(result.planning.path));
    };
    if (!pathIsValid()) {
        return fail(StatusCode::Collision, "planner returned a colliding path");
    }

    result.planning.collision_free = true;
    result.metrics.collision_free = true;
    const bool preserve_motion_directions =
        result.planning.motion_directions.size() == result.planning.path.size();
    const bool path_requires_reverse = preserve_motion_directions &&
        containsReverseMotion(result.planning.motion_directions);
    const bool can_smooth_path = !path_requires_reverse;
    if (config.smoother != "none" && config.smoother != "shortcut" &&
        config.smoother != "curvature") {
        return fail(StatusCode::InvalidConfiguration,
                    "unsupported path smoother configuration");
    }
    if (config.smoother == "curvature" &&
        (config.smoothing_iterations < 0 ||
         config.smoothing_max_curvature <= 0.0)) {
        return fail(StatusCode::InvalidConfiguration,
                    "unsupported path smoother configuration");
    }
    if (config.smoother != "none" && can_smooth_path) {
        if (config.smoothing_iterations < 0) {
            return fail(StatusCode::InvalidConfiguration,
                    "unsupported path smoother configuration");
        }
        std::unique_ptr<autoplanner::CollisionChecker> smoothing_checker;
        autoplanner::CollisionChecker* checker_for_smoothing =
            collision_checker.get();
        if (config.footprint == "rectangle") {
            // ShortcutSmoother only sees Point2d segments. Use the
            // circumscribed circle during shortcutting so the result is safe
            // for every possible rectangle heading; the exact pose-aware
            // rectangle check below remains the final acceptance gate.
            smoothing_checker =
                std::make_unique<autoplanner::FootprintCollisionChecker>(
                    map,
                    autoplanner::RobotFootprint::circle(
                        circumscribed_radius));
            checker_for_smoothing = smoothing_checker.get();
        }
        if (config.smoother == "shortcut") {
            autoplanner::ShortcutSmoother smoother(
                *checker_for_smoothing, config.smoothing_iterations);
            result.planning.path = smoother.smooth(result.planning.path);
        } else {
            const double smoothing_curvature = max_trajectory_curvature > 0.0
                ? std::min(config.smoothing_max_curvature,
                           max_trajectory_curvature)
                : config.smoothing_max_curvature;
            autoplanner::CurvatureConstrainedSmoother smoother(
                *checker_for_smoothing,
                smoothing_curvature,
                config.smoothing_iterations);
            result.planning.path = smoother.smooth(result.planning.path);
        }
        if (!pathIsValid()) {
            return fail(StatusCode::Collision,
                        "smoothed path failed collision validation");
        }
        result.planning.path_length =
            autoplanner::computePathLength(result.planning.path);
        result.planning.message = "Path found and smoothed.";
    }

    autompc::Waypoints waypoints;
    waypoints.reserve(result.planning.path.size());
    for (const auto& point : result.planning.path) {
        waypoints.push_back({point.x, point.y});
    }
    auto trajectory_options = config.trajectory_options;
    trajectory_options.max_curvature = max_trajectory_curvature;
    if (path_requires_reverse) {
        // A Hybrid A* reverse primitive is meaningful only when the tracker,
        // simulator, and supervisor all retain its signed velocity.
        trajectory_options.allow_reverse = true;
        trajectory_options.max_reverse_velocity = std::min(
            trajectory_options.max_reverse_velocity,
            config.simulation_options.max_reverse_velocity);
        if (trajectory_options.max_reverse_velocity <= 0.0) {
            return fail(StatusCode::InvalidConfiguration,
                        "reverse path requires a positive reverse velocity limit");
        }
        result.trajectory = autompc::generateTrajectory(
            waypoints, result.planning.motion_directions, trajectory_options);
    } else {
        result.trajectory = autompc::generateTrajectory(
            waypoints, trajectory_options);
    }
    result.metrics.trajectory_length = autompc::arcLength(result.trajectory);
    const auto trajectory_quality = autompc::assessTrajectory(
        result.trajectory, max_trajectory_curvature);
    result.metrics.max_trajectory_curvature =
        trajectory_quality.max_abs_curvature;
    result.metrics.minimum_turning_radius =
        trajectory_quality.minimum_turning_radius;
    result.metrics.kinematic_feasible = trajectory_quality.finite &&
                                        trajectory_quality.curvature_feasible;
    if (!result.metrics.kinematic_feasible) {
        return fail(StatusCode::InvalidTrajectory,
                    "trajectory violates the configured turning-radius constraint");
    }
    // Corner rounding is only enabled with a hard curvature limit and can
    // leave the original polyline corridor. Validate that new continuous
    // geometry before allowing the tracker to consume it.
    if (max_trajectory_curvature > 0.0 && !path_requires_reverse &&
        !trajectoryIsCollisionFree(result.trajectory, *collision_checker)) {
        return fail(StatusCode::Collision,
                    "continuous trajectory failed collision validation");
    }

    SafetyOptions safety_options = config.safety_options;
    if (safety_options.max_velocity <= 0.0) {
        safety_options.max_velocity = config.simulation_options.max_velocity;
    } else {
        safety_options.max_velocity = std::min(
            safety_options.max_velocity,
            config.simulation_options.max_velocity);
    }
    if (safety_options.max_steering <= 0.0) {
        safety_options.max_steering = config.simulation_options.max_steering;
    } else {
        safety_options.max_steering = std::min(
            safety_options.max_steering,
            config.simulation_options.max_steering);
    }
    if (path_requires_reverse) {
        safety_options.allow_reverse = true;
        safety_options.max_reverse_velocity = std::min(
            safety_options.max_reverse_velocity,
            config.simulation_options.max_reverse_velocity);
    }
    SafetySupervisor supervisor(map, safety_options, collision_checker.get());
    const auto trajectory_decision = supervisor.validateTrajectory(result.trajectory);
    if (!trajectory_decision.safe) {
        return fail(trajectory_decision.status, trajectory_decision.message);
    }

    autompc::State state{result.trajectory.front().x,
                         result.trajectory.front().y,
                         result.trajectory.front().theta,
                         0.0};
    const auto initial_state_decision = supervisor.validateState(state);
    if (!initial_state_decision.safe) {
        return fail(initial_state_decision.status, initial_state_decision.message);
    }

    auto simulation_options = config.simulation_options;
    if (path_requires_reverse) simulation_options.allow_reverse = true;
    autompc::KinematicBicycleSimulator simulator(state, simulation_options);
    std::unique_ptr<DwaLocalPlanner> dwa;
    std::unique_ptr<MppiLocalPlanner> mppi;
    if (config.local_planner == "dwa") {
        dwa = std::make_unique<DwaLocalPlanner>(
                *collision_checker, simulation_options,
                config.dwa_options);
    } else if (config.local_planner == "mppi") {
        mppi = std::make_unique<MppiLocalPlanner>(
                *collision_checker, simulation_options,
                config.mppi_options);
    }
    std::unique_ptr<autompc::PIDController> pid;
    std::unique_ptr<autompc::PurePursuitController> pure_pursuit;
    std::unique_ptr<autompc::StanleyController> stanley;
#ifdef AUTOMPC_HAS_EIGEN
    std::unique_ptr<autompc::MPCController> mpc;
#endif

    if (config.controller == "pid") {
        pid = std::make_unique<autompc::PIDController>(
            1.0, 0.0, 0.0, 2.0, 0.0, 0.5,
            config.simulation_options.wheelbase);
    } else if (config.controller == "pure_pursuit") {
        pure_pursuit = std::make_unique<autompc::PurePursuitController>(
            2.0, config.simulation_options.wheelbase);
    } else if (config.controller == "stanley") {
        stanley = std::make_unique<autompc::StanleyController>(
            0.5, config.simulation_options.wheelbase);
    } else if (config.controller == "mpc") {
#ifdef AUTOMPC_HAS_EIGEN
        mpc = std::make_unique<autompc::MPCController>(
            15, config.simulation_options.dt,
            config.simulation_options.wheelbase,
            config.simulation_options.max_velocity,
            config.simulation_options.max_steering,
            config.simulation_options.max_acceleration,
            config.simulation_options.max_deceleration,
            config.simulation_options.max_steering_rate);
#else
        return fail(StatusCode::InvalidConfiguration,
                    "MPC controller requires Eigen3");
#endif
    } else {
        return fail(StatusCode::InvalidConfiguration,
                    "unknown controller: " + config.controller);
    }

    for (std::size_t step = 0; step < config.max_steps; ++step) {
        const auto reference = closestReferencePoint(result.trajectory, state);
        autompc::Control command;
        if (pid) {
            command = pid->compute(state, reference,
                                   config.simulation_options.dt);
        } else if (pure_pursuit) {
            command = pure_pursuit->compute(
                state, result.trajectory, reference.v);
        } else if (stanley) {
            command = stanley->compute(state, reference, reference.v);
        } else {
#ifdef AUTOMPC_HAS_EIGEN
            command = mpc->compute(state, result.trajectory, reference.v);
#endif
        }

        if (dwa) {
            const auto local_planner_begin = std::chrono::steady_clock::now();
            const auto decision = dwa->computeCommand(
                state, simulator.steering(), result.trajectory, command);
            result.metrics.local_planner_time_ms +=
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - local_planner_begin)
                    .count();
            if (!decision.feasible) {
                result.metrics.safe_stop = true;
                return fail(StatusCode::ControllerInfeasible,
                            "DWA found no collision-free local command");
            }
            if (std::abs(decision.command.velocity - command.velocity) >
                    1e-9 ||
                std::abs(decision.command.steering - command.steering) >
                    1e-9) {
                ++result.metrics.local_planner_adjustments;
            }
            command = decision.command;
            result.metrics.local_planner_collision_rejections +=
                decision.dynamic_collision_rejections;
            if (std::isfinite(decision.minimum_dynamic_clearance)) {
                if (result.metrics.minimum_dynamic_obstacle_clearance == 0.0) {
                    result.metrics.minimum_dynamic_obstacle_clearance =
                        decision.minimum_dynamic_clearance;
                } else {
                    result.metrics.minimum_dynamic_obstacle_clearance =
                        std::min(result.metrics.minimum_dynamic_obstacle_clearance,
                                 decision.minimum_dynamic_clearance);
                }
            }
        } else if (mppi) {
            const auto local_planner_begin = std::chrono::steady_clock::now();
            const auto decision = mppi->computeCommand(
                state, simulator.steering(), result.trajectory, command);
            result.metrics.local_planner_time_ms +=
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - local_planner_begin)
                    .count();
            result.metrics.local_planner_rollouts +=
                decision.feasible_rollouts;
            result.metrics.local_planner_collision_rejections +=
                decision.dynamic_collision_rejections;
            if (!decision.feasible) {
                result.metrics.safe_stop = true;
                return fail(StatusCode::ControllerInfeasible,
                            "MPPI found no collision-free local command");
            }
            if (std::abs(decision.command.velocity - command.velocity) >
                    1e-9 ||
                std::abs(decision.command.steering - command.steering) >
                    1e-9) {
                ++result.metrics.local_planner_adjustments;
            }
            command = decision.command;
        }

        const auto command_decision = supervisor.validateCommand(command);
        if (!command_decision.safe) {
            result.metrics.safe_stop = true;
            return fail(command_decision.status, command_decision.message);
        }

        state = simulator.step(command);
        const auto state_decision = supervisor.validateState(state);
        const double cross_track = crossTrackError(state, reference);
        const double heading = headingError(state, reference);
        result.trace.append({
            static_cast<double>(step + 1) * config.simulation_options.dt,
            state, command, cross_track, heading});
        result.metrics.steps = result.trace.size();

        if (!state_decision.safe) {
            result.metrics.safe_stop = true;
            return fail(state_decision.status, state_decision.message);
        }
        if (cross_track > safety_options.max_cross_track_error) {
            result.metrics.safe_stop = true;
            return fail(StatusCode::SafeStop,
                        "cross-track error exceeded the safety threshold");
        }
        if (supervisor.goalReached(state, result.trajectory)) {
            result.metrics.goal_reached = true;
            break;
        }
    }

    const auto& samples = result.trace.samples();
    if (!samples.empty()) {
        double sum_cross_track = 0.0;
        double sum_heading = 0.0;
        for (const auto& sample : samples) {
            result.metrics.max_cross_track_error = std::max(
                result.metrics.max_cross_track_error,
                sample.cross_track_error);
            result.metrics.max_heading_error = std::max(
                result.metrics.max_heading_error, sample.heading_error);
            sum_cross_track += sample.cross_track_error;
            sum_heading += sample.heading_error;
        }
        result.metrics.mean_cross_track_error =
            sum_cross_track / static_cast<double>(samples.size());
        result.metrics.mean_heading_error =
            sum_heading / static_cast<double>(samples.size());
        result.metrics.goal_distance = std::hypot(
            samples.back().state.x - result.trajectory.back().x,
            samples.back().state.y - result.trajectory.back().y);
    }
    result.metrics.steps = result.trace.size();

    if (!result.metrics.goal_reached) {
        return fail(StatusCode::Timeout,
                    "pipeline reached the step limit before the goal");
    }

    result.metrics.status = StatusCode::Success;
    result.message = "navigation pipeline completed";
    return result;
}

bool savePipelineMetricsJson(const PipelineResult& result,
                             const std::string& file_path) {
    std::ofstream output(file_path);
    if (!output.is_open()) return false;

    output << std::fixed << std::setprecision(8);
    output << "{\n"
           << "  \"status_code\": \""
           << toString(result.metrics.status) << "\",\n"
           << "  \"message\": \""
           << escapeJsonString(result.message) << "\",\n"
           << "  \"planner\": \""
           << escapeJsonString(result.planning.planner_name) << "\",\n"
           << "  \"footprint\": \""
           << escapeJsonString(result.metrics.footprint) << "\",\n"
           << "  \"smoother\": \""
           << escapeJsonString(result.metrics.smoother) << "\",\n"
           << "  \"local_planner\": \""
           << escapeJsonString(result.metrics.local_planner) << "\",\n"
           << "  \"success\": "
           << (result.metrics.status == StatusCode::Success ? "true" : "false")
           << ",\n"
           << "  \"goal_reached\": "
           << (result.metrics.goal_reached ? "true" : "false") << ",\n"
           << "  \"collision_free\": "
           << (result.metrics.collision_free ? "true" : "false") << ",\n"
           << "  \"safe_stop\": "
           << (result.metrics.safe_stop ? "true" : "false") << ",\n"
           << "  \"controller_trace_steps\": " << result.metrics.steps << ",\n"
           << "  \"local_planner_adjustments\": "
           << result.metrics.local_planner_adjustments << ",\n"
           << "  \"local_planner_rollouts\": "
           << result.metrics.local_planner_rollouts << ",\n"
           << "  \"local_planner_collision_rejections\": "
           << result.metrics.local_planner_collision_rejections << ",\n"
           << "  \"local_planner_time_ms\": ";
    writeJsonNumber(output, result.metrics.local_planner_time_ms);
    output << ",\n"
           << "  \"minimum_dynamic_obstacle_clearance\": ";
    writeJsonNumber(output, result.metrics.minimum_dynamic_obstacle_clearance);
    output << ",\n"
           << "  \"planning_time_ms\": ";
    writeJsonNumber(output, result.metrics.planning_time_ms);
    output << ",\n  \"path_length\": ";
    writeJsonNumber(output, result.metrics.path_length);
    output << ",\n  \"trajectory_length\": ";
    writeJsonNumber(output, result.metrics.trajectory_length);
    output << ",\n  \"max_trajectory_curvature\": ";
    writeJsonNumber(output, result.metrics.max_trajectory_curvature);
    output << ",\n  \"minimum_turning_radius\": ";
    writeJsonNumber(output, result.metrics.minimum_turning_radius);
    output << ",\n  \"kinematic_feasible\": "
           << (result.metrics.kinematic_feasible ? "true" : "false");
    output << ",\n  \"max_cross_track_error\": ";
    writeJsonNumber(output, result.metrics.max_cross_track_error);
    output << ",\n  \"mean_cross_track_error\": ";
    writeJsonNumber(output, result.metrics.mean_cross_track_error);
    output << ",\n  \"max_heading_error\": ";
    writeJsonNumber(output, result.metrics.max_heading_error);
    output << ",\n  \"mean_heading_error\": ";
    writeJsonNumber(output, result.metrics.mean_heading_error);
    output << ",\n  \"goal_distance\": ";
    writeJsonNumber(output, result.metrics.goal_distance);
    output << "\n}\n";
    return true;
}

}  // namespace robotnav
