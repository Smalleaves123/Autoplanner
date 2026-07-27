#include "robotnav/dynamic_navigation_pipeline.h"

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
#include "autompc/simulation/kinematic_bicycle.h"
#include "autompc/trajectory/trajectory_generator.h"
#include "autoplanner/collision/footprint_collision_checker.h"
#include "autoplanner/collision/grid_collision_checker.h"
#include "autoplanner/core/grid_map.h"
#include "autoplanner/core/path.h"
#include "autoplanner/costmap/costmap_2d.h"
#include "autoplanner/planners/graph_search/astar.h"
#include "autoplanner/planners/graph_search/dstar_lite.h"
#include "autoplanner/smoothing/shortcut_smoother.h"
#include "robotnav/safety_supervisor.h"

namespace robotnav {
namespace {

struct PreparedGeometry {
    autoplanner::GridMap planning_map;
    autoplanner::RobotFootprint footprint;
    double circumscribed_radius = 0.0;
    std::unique_ptr<autoplanner::CollisionChecker> checker;
};

bool configureGeometry(const autoplanner::GridMap& map,
                       const PipelineConfig& config,
                       PreparedGeometry& geometry,
                       std::string& error) {
    if (config.footprint == "point") {
        geometry.footprint = autoplanner::RobotFootprint::circle(0.0);
    } else if (config.footprint == "circle" &&
               std::isfinite(config.robot_radius) &&
               config.robot_radius > 0.0) {
        geometry.footprint = autoplanner::RobotFootprint::circle(
            config.robot_radius);
        geometry.circumscribed_radius = config.robot_radius;
    } else if (config.footprint == "rectangle" &&
               std::isfinite(config.robot_length) &&
               std::isfinite(config.robot_width) &&
               config.robot_length > 0.0 && config.robot_width > 0.0) {
        geometry.footprint = autoplanner::RobotFootprint::rectangle(
            config.robot_length, config.robot_width);
        geometry.circumscribed_radius = std::hypot(
            0.5 * config.robot_length, 0.5 * config.robot_width);
    } else {
        error = "invalid robot footprint configuration";
        return false;
    }

    geometry.planning_map = map;
    if ((config.inflate_map || config.footprint != "point") &&
        geometry.circumscribed_radius > 0.0) {
        geometry.planning_map.inflateObstacles(
            geometry.circumscribed_radius);
    }

    if (config.footprint == "point") {
        geometry.checker =
            std::make_unique<autoplanner::GridCollisionChecker>(map);
    } else {
        geometry.checker =
            std::make_unique<autoplanner::FootprintCollisionChecker>(
                map, geometry.footprint);
    }
    return true;
}

std::vector<autoplanner::Pose2d> makePoses(
    const autoplanner::Path2d& path) {
    std::vector<autoplanner::Pose2d> poses;
    poses.reserve(path.size());
    for (std::size_t i = 0; i < path.size(); ++i) {
        const std::size_t next = std::min(i + 1, path.size() - 1);
        const std::size_t previous = i == 0 ? i : i - 1;
        poses.push_back({
            path[i].x, path[i].y,
            std::atan2(path[next].y - path[previous].y,
                       path[next].x - path[previous].x)});
    }
    return poses;
}

bool pathIsValid(const autoplanner::CollisionChecker& checker,
                 const std::string& footprint,
                 const autoplanner::Path2d& path) {
    if (footprint == "point") return checker.isPathValid(path);
    return checker.isPosePathValid(makePoses(path));
}

bool preparePath(const autoplanner::GridMap& map,
                 const PipelineConfig& config,
                 PreparedGeometry& geometry,
                 autoplanner::Path2d& path,
                 std::string& error) {
    if (!pathIsValid(*geometry.checker, config.footprint, path)) {
        error = "planned path failed footprint collision validation";
        return false;
    }
    if (config.smoother == "none") return true;
    if (config.smoother != "shortcut" || config.smoothing_iterations < 0) {
        error = "unsupported path smoother configuration";
        return false;
    }

    std::unique_ptr<autoplanner::CollisionChecker> smoothing_checker;
    autoplanner::CollisionChecker* checker_for_smoothing =
        geometry.checker.get();
    if (config.footprint == "rectangle") {
        smoothing_checker =
            std::make_unique<autoplanner::FootprintCollisionChecker>(
                map,
                autoplanner::RobotFootprint::circle(
                    geometry.circumscribed_radius));
        checker_for_smoothing = smoothing_checker.get();
    }
    autoplanner::ShortcutSmoother smoother(
        *checker_for_smoothing, config.smoothing_iterations);
    path = smoother.smooth(path);
    if (!pathIsValid(*geometry.checker, config.footprint, path)) {
        error = "smoothed path failed footprint collision validation";
        return false;
    }
    return true;
}

autompc::Trajectory makeTrajectory(const autoplanner::Path2d& path,
                                   const PipelineConfig& config) {
    autompc::Waypoints waypoints;
    waypoints.reserve(path.size());
    for (const auto& point : path) {
        waypoints.push_back({point.x, point.y});
    }
    return autompc::generateTrajectory(waypoints, config.trajectory_options);
}

autompc::TrajectoryPoint closestReference(
    const autompc::Trajectory& trajectory,
    const autompc::State& state) {
    autompc::TrajectoryPoint best = trajectory.front();
    double best_distance = std::numeric_limits<double>::max();
    for (const auto& point : trajectory) {
        const double distance = std::hypot(
            point.x - state.x, point.y - state.y);
        if (distance < best_distance) {
            best_distance = distance;
            best = point;
        }
    }
    return best;
}

double crossTrackError(const autompc::State& state,
                       const autompc::TrajectoryPoint& reference) {
    return std::hypot(reference.x - state.x, reference.y - state.y);
}

autoplanner::Point2i stateCell(const autompc::State& state,
                               const autoplanner::GridMap& map) {
    return {
        std::clamp(static_cast<int>(std::lround(state.x)), 0,
                   std::max(0, map.width() - 1)),
        std::clamp(static_cast<int>(std::lround(state.y)), 0,
                   std::max(0, map.height() - 1))};
}

bool alreadyPlaced(const std::vector<autoplanner::Point2i>& placed,
                   int x, int y) {
    return std::any_of(placed.begin(), placed.end(),
                       [x, y](const autoplanner::Point2i& point) {
                           return point.x == x && point.y == y;
                       });
}

bool sameCell(const autoplanner::Point2i& left,
              const autoplanner::Point2i& right) {
    return left.x == right.x && left.y == right.y;
}

bool containsCell(const std::vector<autoplanner::Point2i>& cells,
                  const autoplanner::Point2i& cell) {
    return std::any_of(cells.begin(), cells.end(),
                       [&cell](const autoplanner::Point2i& candidate) {
                           return sameCell(candidate, cell);
                       });
}

autoplanner::Point2i movingObstacleCell(
    const MovingObstacle& obstacle,
    std::size_t frame) {
    const auto delta = static_cast<int>(frame - obstacle.start_frame);
    return {
        obstacle.start_cell.x + obstacle.dx_per_frame * delta,
        obstacle.start_cell.y + obstacle.dy_per_frame * delta};
}

bool insertObstacleAhead(autoplanner::GridMap& map,
                         const autoplanner::Path2d& path,
                         const autompc::State& state,
                         const autoplanner::Point2i& goal,
                         bool allow_diagonal,
                         std::size_t ahead,
                         std::size_t margin_cells,
                         std::vector<autoplanner::Point2i>& placed,
                         autoplanner::Point2i& obstacle) {
    if (path.size() < 2) return false;
    std::vector<autoplanner::Point2d> samples;
    for (std::size_t i = 1; i < path.size(); ++i) {
        const auto& previous = path[i - 1];
        const auto& current = path[i];
        const double distance = std::hypot(
            current.x - previous.x, current.y - previous.y);
        const int count = std::max(1, static_cast<int>(std::ceil(distance)));
        for (int step = 0; step < count; ++step) {
            const double t = static_cast<double>(step) /
                static_cast<double>(count);
            samples.push_back({
                previous.x + t * (current.x - previous.x),
                previous.y + t * (current.y - previous.y)});
        }
    }
    samples.push_back(path.back());
    if (samples.empty()) return false;

    std::size_t closest = 0;
    double best_distance = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < samples.size(); ++i) {
        const double distance = std::hypot(
            samples[i].x - state.x, samples[i].y - state.y);
        if (distance < best_distance) {
            best_distance = distance;
            closest = i;
        }
    }
    const std::size_t first = std::min(closest + ahead, samples.size() - 1);
    for (std::size_t index = first; index < samples.size(); ++index) {
        const int x = static_cast<int>(std::lround(samples[index].x));
        const int y = static_cast<int>(std::lround(samples[index].y));
        if ((x != goal.x || y != goal.y) && map.isInside(x, y) &&
            map.isFree(x, y) &&
            !alreadyPlaced(placed, x, y)) {
            std::vector<autoplanner::Point2i> added;
            const int margin = static_cast<int>(margin_cells);
            for (int dy = -margin; dy <= margin; ++dy) {
                for (int dx = -margin; dx <= margin; ++dx) {
                    const int candidate_x = x + dx;
                    const int candidate_y = y + dy;
                    if (!map.isInside(candidate_x, candidate_y) ||
                        (candidate_x == goal.x && candidate_y == goal.y) ||
                        !map.isFree(candidate_x, candidate_y)) {
                        continue;
                    }
                    map.setOccupied(candidate_x, candidate_y, true);
                    added.push_back({candidate_x, candidate_y});
                }
            }
            const auto current = stateCell(state, map);
            autoplanner::AStarPlanner reachability_check(allow_diagonal);
            const auto check = reachability_check.plan(map, current, goal);
            if (!check.success) {
                for (const auto& cell : added) {
                    map.setOccupied(cell.x, cell.y, false);
                }
                continue;
            }
            obstacle = {x, y};
            placed.push_back(obstacle);
            return true;
        }
    }
    return false;
}

std::string escapeJsonString(const std::string& value) {
    std::string escaped;
    for (const char character : value) {
        if (character == '"') escaped += "\\\"";
        else if (character == '\\') escaped += "\\\\";
        else escaped += character;
    }
    return escaped;
}

void writeJsonNumber(std::ofstream& output, double value) {
    if (std::isfinite(value)) output << value;
    else output << "null";
}

}  // namespace

DynamicPipelineResult DynamicNavigationPipeline::run(
    const autoplanner::GridMap& map,
    const autoplanner::Point2i& start,
    const autoplanner::Point2i& goal,
    const DynamicPipelineConfig& config) const {
    DynamicPipelineResult result;
    result.metrics.status = StatusCode::InternalError;
    result.metrics.footprint = config.pipeline.footprint;
    result.metrics.smoother = config.pipeline.smoother;
    result.metrics.frames_requested = config.frames;

    auto fail = [&result](StatusCode status, const std::string& message) {
        result.metrics.status = status;
        result.message = message;
        return result;
    };

    if (map.isEmpty() || config.frames == 0 ||
        config.steps_per_frame == 0 || config.pipeline.max_steps == 0) {
        return fail(StatusCode::InvalidConfiguration,
                    "dynamic pipeline configuration is invalid");
    }
    if (!map.isFree(start.x, start.y)) {
        return fail(StatusCode::InvalidStart, "start cell is invalid");
    }
    if (!map.isFree(goal.x, goal.y)) {
        return fail(StatusCode::InvalidGoal, "goal cell is invalid");
    }

    autoplanner::GridMap dynamic_map = map;
    PreparedGeometry geometry;
    std::string error;
    if (!configureGeometry(dynamic_map, config.pipeline, geometry, error)) {
        return fail(StatusCode::InvalidConfiguration, error);
    }
    if (!geometry.planning_map.isFree(start.x, start.y) ||
        !geometry.planning_map.isFree(goal.x, goal.y)) {
        return fail(StatusCode::InvalidConfiguration,
                    "footprint makes start or goal invalid");
    }

    autoplanner::DStarLitePlanner dstar(
        config.pipeline.planner_options.allow_diagonal);
    autoplanner::AStarPlanner astar(
        config.pipeline.planner_options.allow_diagonal);
    auto initial = dstar.plan(geometry.planning_map, start, goal);
    result.initial_planning = initial;
    if (!initial.success || initial.path.empty()) {
        return fail(initial.statusCode(), "initial D* Lite planning failed");
    }
    autoplanner::Path2d current_path = initial.path;
    if (!preparePath(dynamic_map, config.pipeline, geometry,
                     current_path, error)) {
        return fail(StatusCode::Collision, error);
    }
    autompc::Trajectory trajectory = makeTrajectory(
        current_path, config.pipeline);
    if (trajectory.empty()) {
        return fail(StatusCode::InvalidTrajectory,
                    "initial trajectory generation failed");
    }

    SafetyOptions safety_options = config.pipeline.safety_options;
    safety_options.max_velocity = safety_options.max_velocity <= 0.0
        ? config.pipeline.simulation_options.max_velocity
        : std::min(safety_options.max_velocity,
                   config.pipeline.simulation_options.max_velocity);
    safety_options.max_steering = safety_options.max_steering <= 0.0
        ? config.pipeline.simulation_options.max_steering
        : std::min(safety_options.max_steering,
                   config.pipeline.simulation_options.max_steering);
    SafetySupervisor supervisor(dynamic_map, safety_options,
                                geometry.checker.get());
    const auto trajectory_check = supervisor.validateTrajectory(trajectory);
    if (!trajectory_check.safe) {
        return fail(trajectory_check.status, trajectory_check.message);
    }

    autompc::State state{trajectory.front().x, trajectory.front().y,
                         trajectory.front().theta, 0.0};
    if (!supervisor.validateState(state).safe) {
        return fail(StatusCode::Collision,
                    "initial state is not safe for the configured footprint");
    }
    autompc::KinematicBicycleSimulator simulator(
        state, config.pipeline.simulation_options);
    std::unique_ptr<autompc::PIDController> pid;
    std::unique_ptr<autompc::PurePursuitController> pure_pursuit;
    std::unique_ptr<autompc::StanleyController> stanley;
#ifdef AUTOMPC_HAS_EIGEN
    std::unique_ptr<autompc::MPCController> mpc;
#endif
    if (config.pipeline.controller == "pid") {
        pid = std::make_unique<autompc::PIDController>(
            1.0, 0.0, 0.0, 2.0, 0.0, 0.5,
            config.pipeline.simulation_options.wheelbase);
    } else if (config.pipeline.controller == "pure_pursuit") {
        pure_pursuit = std::make_unique<autompc::PurePursuitController>(
            2.0, config.pipeline.simulation_options.wheelbase);
    } else if (config.pipeline.controller == "stanley") {
        stanley = std::make_unique<autompc::StanleyController>(
            0.5, config.pipeline.simulation_options.wheelbase);
    } else if (config.pipeline.controller == "mpc") {
#ifdef AUTOMPC_HAS_EIGEN
        mpc = std::make_unique<autompc::MPCController>(
            15, config.pipeline.simulation_options.dt,
            config.pipeline.simulation_options.wheelbase,
            config.pipeline.simulation_options.max_velocity,
            config.pipeline.simulation_options.max_steering,
            config.pipeline.simulation_options.max_acceleration,
            config.pipeline.simulation_options.max_deceleration,
            config.pipeline.simulation_options.max_steering_rate);
#else
        return fail(StatusCode::InvalidConfiguration,
                    "MPC controller requires Eigen3");
#endif
    } else {
        return fail(StatusCode::InvalidConfiguration,
                    "unknown controller: " + config.pipeline.controller);
    }

    std::vector<autoplanner::Point2i> placed_obstacles;
    std::vector<autoplanner::Point2i> active_moving_cells(
        config.moving_obstacles.size(), {-1, -1});
    std::vector<autoplanner::Point2i> externally_occupied_cells;
    autompc::Control previous_control{};
    bool has_previous_control = false;
    std::size_t total_control_samples = 0;
    double control_jump_sum = 0.0;
    double time = 0.0;

    auto appendSample = [&](std::size_t frame, std::size_t step,
                            const autompc::Control& command,
                            bool replanned,
                            const autoplanner::Point2i& obstacle,
                            double dstar_ms, double astar_ms,
                            bool safe_stop) {
        const auto reference = closestReference(trajectory, state);
        const double steering_delta = has_previous_control
            ? std::abs(command.steering - previous_control.steering) : 0.0;
        const double velocity_delta = has_previous_control
            ? std::abs(command.velocity - previous_control.velocity) : 0.0;
        if (has_previous_control) {
            control_jump_sum += std::hypot(steering_delta, velocity_delta);
            result.metrics.max_control_jump = std::max(
                result.metrics.max_control_jump,
                std::hypot(steering_delta, velocity_delta));
            ++total_control_samples;
        }
        time += config.pipeline.simulation_options.dt;
        result.trace.push_back({
            frame, step, time, state, command, replanned, obstacle,
            dstar_ms, astar_ms, crossTrackError(state, reference),
            steering_delta, velocity_delta, safe_stop});
        ++result.metrics.steps;
        previous_control = command;
        has_previous_control = true;
    };

    for (std::size_t frame = 0;
         frame < config.frames && result.metrics.steps < config.pipeline.max_steps;
         ++frame) {
        autoplanner::Point2i obstacle{-1, -1};
        bool map_changed = false;
        for (std::size_t index = 0;
             index < config.moving_obstacles.size(); ++index) {
            const auto& moving = config.moving_obstacles[index];
            const bool active_this_frame =
                moving.end_frame >= moving.start_frame &&
                frame >= moving.start_frame && frame <= moving.end_frame;
            const auto current = active_this_frame
                ? movingObstacleCell(moving, frame)
                : autoplanner::Point2i{-1, -1};
            const auto previous = active_moving_cells[index];
            if (dynamic_map.isInside(previous.x, previous.y) &&
                !sameCell(previous, current)) {
                const bool externally_owned = containsCell(
                    externally_occupied_cells, previous);
                if (!externally_owned &&
                    dynamic_map.isOccupied(previous.x, previous.y) &&
                    dynamic_map.setOccupied(previous.x, previous.y, false)) {
                    map_changed = true;
                    obstacle = previous;
                    ++result.metrics.moving_obstacle_update_count;
                } else if (externally_owned) {
                    ++result.metrics.moving_obstacle_conflict_count;
                }
                active_moving_cells[index] = {-1, -1};
            }
            if (!active_this_frame) continue;
            if (sameCell(previous, current) &&
                dynamic_map.isInside(current.x, current.y) &&
                dynamic_map.isOccupied(current.x, current.y)) {
                active_moving_cells[index] = current;
                continue;
            }
            if (!dynamic_map.isInside(current.x, current.y) ||
                !dynamic_map.isFree(current.x, current.y) ||
                (current.x == goal.x && current.y == goal.y)) {
                continue;
            }
            if (dynamic_map.setOccupied(current.x, current.y, true)) {
                map_changed = true;
                obstacle = current;
                active_moving_cells[index] = current;
                ++result.metrics.moving_obstacle_update_count;
            }
        }
        for (const auto& update : config.obstacle_updates) {
            if (update.frame != frame ||
                !dynamic_map.isInside(update.cell.x, update.cell.y)) {
                continue;
            }
            const bool was_occupied = dynamic_map.isOccupied(
                update.cell.x, update.cell.y);
            if (update.occupied) {
                if (!containsCell(externally_occupied_cells, update.cell)) {
                    externally_occupied_cells.push_back(update.cell);
                }
            } else {
                externally_occupied_cells.erase(
                    std::remove_if(
                        externally_occupied_cells.begin(),
                        externally_occupied_cells.end(),
                        [&update](const autoplanner::Point2i& cell) {
                            return sameCell(cell, update.cell);
                        }),
                    externally_occupied_cells.end());
            }
            if (dynamic_map.setOccupied(update.cell.x, update.cell.y,
                                        update.occupied) &&
                was_occupied != update.occupied) {
                map_changed = true;
                obstacle = update.cell;
                ++result.metrics.external_update_count;
            }
        }
        if (config.auto_insert_obstacles && frame > 0 &&
            placed_obstacles.size() < config.max_auto_obstacles) {
            map_changed = insertObstacleAhead(
                dynamic_map, current_path, state, goal,
                config.pipeline.planner_options.allow_diagonal,
                config.obstacle_insertion_ahead,
                config.auto_obstacle_margin_cells,
                placed_obstacles, obstacle) || map_changed;
        }

        geometry.planning_map = dynamic_map;
        if ((config.pipeline.inflate_map ||
             config.pipeline.footprint != "point") &&
            geometry.circumscribed_radius > 0.0) {
            geometry.planning_map.inflateObstacles(
                geometry.circumscribed_radius);
        }

        bool replanned = false;
        double dstar_ms = 0.0;
        double astar_ms = 0.0;
        if (map_changed || !pathIsValid(*geometry.checker,
                                         config.pipeline.footprint,
                                         current_path)) {
            const auto current = stateCell(state, geometry.planning_map);
            const auto dstar_begin = std::chrono::steady_clock::now();
            const auto dstar_result = dstar.replan(
                geometry.planning_map, current);
            const auto dstar_end = std::chrono::steady_clock::now();
            dstar_ms = std::chrono::duration<double, std::milli>(
                dstar_end - dstar_begin).count();
            result.metrics.total_dstar_replanning_time_ms += dstar_ms;

            bool used_astar_fallback = false;
            if (config.compare_astar) {
                const auto astar_begin = std::chrono::steady_clock::now();
                const auto astar_result = astar.plan(
                    geometry.planning_map, current, goal);
                const auto astar_end = std::chrono::steady_clock::now();
                astar_ms = std::chrono::duration<double, std::milli>(
                    astar_end - astar_begin).count();
                result.metrics.total_astar_replanning_time_ms += astar_ms;

                if (!dstar_result.success && astar_result.success) {
                    current_path = astar_result.path;
                    used_astar_fallback = true;
                    ++result.metrics.astar_fallback_count;
                }
            }

            ++result.metrics.replanning_count;
            if (!dstar_result.success) ++result.metrics.dstar_failure_count;
            if ((!dstar_result.success && !used_astar_fallback) ||
                (dstar_result.success && dstar_result.path.empty())) {
                result.metrics.safe_stop = true;
                for (std::size_t stop = 0;
                     stop < config.steps_per_frame &&
                     result.metrics.steps < config.pipeline.max_steps;
                     ++stop) {
                    const autompc::Control command{0.0, 0.0};
                    state = simulator.step(command);
                    appendSample(frame, stop, command, false, obstacle,
                                 dstar_ms, astar_ms, true);
                    if (state.v <= 1e-6) break;
                }
                result.metrics.collision_steps += 1;
                return fail(StatusCode::ReplanningFailed,
                            "D* Lite failed; vehicle entered safe stop");
            }

            if (dstar_result.success) current_path = dstar_result.path;
            if (!preparePath(dynamic_map, config.pipeline, geometry,
                             current_path, error)) {
                result.metrics.safe_stop = true;
                return fail(StatusCode::Collision, error);
            }
            trajectory = makeTrajectory(current_path, config.pipeline);
            if (trajectory.empty()) {
                result.metrics.safe_stop = true;
                return fail(StatusCode::InvalidTrajectory,
                            "replanned trajectory generation failed");
            }
            if (pid) pid->reset();
#ifdef AUTOMPC_HAS_EIGEN
            if (mpc) mpc->resetReferenceProgress();
#endif
            replanned = true;
        }

        ++result.metrics.frames_run;
        for (std::size_t step = 0;
             step < config.steps_per_frame &&
             result.metrics.steps < config.pipeline.max_steps;
             ++step) {
            const auto reference = closestReference(trajectory, state);
            autompc::Control command;
            if (pid) {
                command = pid->compute(
                    state, reference, config.pipeline.simulation_options.dt);
            } else if (pure_pursuit) {
                command = pure_pursuit->compute(
                    state, trajectory, reference.v);
            } else if (stanley) {
                command = stanley->compute(state, reference, reference.v);
            } else {
#ifdef AUTOMPC_HAS_EIGEN
                command = mpc->compute(state, trajectory, reference.v);
#endif
            }
            const auto command_check = supervisor.validateCommand(command);
            if (!command_check.safe) {
                result.metrics.safe_stop = true;
                return fail(command_check.status, command_check.message);
            }
            state = simulator.step(command);
            const auto state_check = supervisor.validateState(state);
            appendSample(frame, step, command,
                         replanned && step == 0, obstacle,
                         step == 0 ? dstar_ms : 0.0,
                         step == 0 ? astar_ms : 0.0, false);
            if (!state_check.safe) {
                ++result.metrics.collision_steps;
                result.metrics.safe_stop = true;
                return fail(state_check.status, state_check.message);
            }
            if (supervisor.goalReached(state, trajectory)) {
                result.metrics.goal_reached = true;
                break;
            }
        }
        if (result.metrics.goal_reached) break;
    }

    result.final_path = current_path;
    if (!result.trace.empty()) {
        const auto& last = result.trace.back();
        result.metrics.goal_distance = std::hypot(
            last.state.x - static_cast<double>(goal.x),
            last.state.y - static_cast<double>(goal.y));
    }
    if (total_control_samples > 0) {
        result.metrics.mean_control_jump = control_jump_sum /
            static_cast<double>(total_control_samples);
    }
    if (!result.metrics.goal_reached) {
        const bool step_limit_reached =
            result.metrics.steps >= config.pipeline.max_steps;
        return fail(StatusCode::Timeout,
                    step_limit_reached
                        ? "dynamic pipeline reached the step limit before the goal"
                        : "dynamic pipeline reached the frame limit before the goal");
    }
    result.metrics.status = StatusCode::Success;
    result.message = "dynamic navigation pipeline completed";
    return result;
}

bool saveDynamicTraceCsv(const DynamicPipelineResult& result,
                         const std::string& file_path) {
    std::ofstream output(file_path);
    if (!output.is_open()) return false;
    output << "frame,step,time,x,y,theta,velocity,command_velocity,"
              "command_steering,replanned,obstacle_x,obstacle_y,"
              "dstar_replan_ms,astar_replan_ms,cross_track_error,"
              "steering_delta,velocity_delta,safe_stop\n";
    output << std::fixed << std::setprecision(8);
    for (const auto& sample : result.trace) {
        output << sample.frame << ',' << sample.step << ',' << sample.time << ','
               << sample.state.x << ',' << sample.state.y << ','
               << sample.state.theta << ',' << sample.state.v << ','
               << sample.command.velocity << ',' << sample.command.steering << ','
               << (sample.replanned ? 1 : 0) << ','
               << sample.obstacle.x << ',' << sample.obstacle.y << ','
               << sample.dstar_replan_ms << ',' << sample.astar_replan_ms << ','
               << sample.cross_track_error << ',' << sample.steering_delta << ','
               << sample.velocity_delta << ','
               << (sample.safe_stop ? 1 : 0) << '\n';
    }
    return true;
}

bool saveDynamicMetricsJson(const DynamicPipelineResult& result,
                            const std::string& file_path) {
    std::ofstream output(file_path);
    if (!output.is_open()) return false;
    output << std::fixed << std::setprecision(8)
           << "{\n"
           << "  \"status_code\": \""
           << toString(result.metrics.status) << "\",\n"
           << "  \"message\": \""
           << escapeJsonString(result.message) << "\",\n"
           << "  \"footprint\": \""
           << escapeJsonString(result.metrics.footprint) << "\",\n"
           << "  \"smoother\": \""
           << escapeJsonString(result.metrics.smoother) << "\",\n"
           << "  \"success\": "
           << (result.metrics.status == StatusCode::Success ? "true" : "false")
           << ",\n"
           << "  \"frames_requested\": "
           << result.metrics.frames_requested << ",\n"
           << "  \"frames_run\": " << result.metrics.frames_run << ",\n"
           << "  \"steps\": " << result.metrics.steps << ",\n"
           << "  \"replanning_count\": "
           << result.metrics.replanning_count << ",\n"
           << "  \"external_update_count\": "
           << result.metrics.external_update_count << ",\n"
           << "  \"moving_obstacle_update_count\": "
           << result.metrics.moving_obstacle_update_count << ",\n"
           << "  \"moving_obstacle_conflict_count\": "
           << result.metrics.moving_obstacle_conflict_count << ",\n"
           << "  \"dstar_failure_count\": "
           << result.metrics.dstar_failure_count << ",\n"
           << "  \"astar_fallback_count\": "
           << result.metrics.astar_fallback_count << ",\n"
           << "  \"collision_steps\": "
           << result.metrics.collision_steps << ",\n"
           << "  \"goal_reached\": "
           << (result.metrics.goal_reached ? "true" : "false") << ",\n"
           << "  \"safe_stop\": "
           << (result.metrics.safe_stop ? "true" : "false") << ",\n"
           << "  \"total_dstar_replanning_time_ms\": ";
    writeJsonNumber(output, result.metrics.total_dstar_replanning_time_ms);
    output << ",\n  \"total_astar_replanning_time_ms\": ";
    writeJsonNumber(output, result.metrics.total_astar_replanning_time_ms);
    output << ",\n  \"max_control_jump\": ";
    writeJsonNumber(output, result.metrics.max_control_jump);
    output << ",\n  \"mean_control_jump\": ";
    writeJsonNumber(output, result.metrics.mean_control_jump);
    output << ",\n  \"goal_distance\": ";
    writeJsonNumber(output, result.metrics.goal_distance);
    output << "\n}\n";
    return true;
}

}  // namespace robotnav
