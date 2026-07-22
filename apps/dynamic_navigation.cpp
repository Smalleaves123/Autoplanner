#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include "autoplanner/collision/grid_collision_checker.h"
#include "autoplanner/core/grid_map.h"
#include "autoplanner/planners/graph_search/astar.h"
#include "autoplanner/planners/graph_search/dstar_lite.h"
#include "autompc/autompc.h"

using namespace autoplanner;
using namespace autompc;

namespace {

std::size_t closestPathIndex(const Path2d& path, const State& state) {
    std::size_t best = 0;
    double best_distance = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < path.size(); ++i) {
        const double dx = path[i].x - state.x;
        const double dy = path[i].y - state.y;
        const double distance = dx * dx + dy * dy;
        if (distance < best_distance) {
            best_distance = distance;
            best = i;
        }
    }
    return best;
}

std::size_t closestTrajectoryIndex(const Trajectory& trajectory,
                                   const State& state) {
    std::size_t best = 0;
    double best_distance = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < trajectory.size(); ++i) {
        const double dx = trajectory[i].x - state.x;
        const double dy = trajectory[i].y - state.y;
        const double distance = dx * dx + dy * dy;
        if (distance < best_distance) {
            best_distance = distance;
            best = i;
        }
    }
    return best;
}

Trajectory makeTrajectory(const Path2d& path) {
    Waypoints waypoints;
    waypoints.reserve(path.size());
    for (const auto& point : path) waypoints.push_back({point.x, point.y});

    TrajectoryOptions options;
    options.sample_spacing = 0.5;
    options.target_velocity = 1.0;
    options.max_velocity = 2.0;
    options.max_acceleration = 1.5;
    options.max_deceleration = 2.0;
    options.max_lateral_acceleration = 1.5;
    return generateTrajectory(waypoints, options);
}

Point2i stateCell(const State& state, const GridMap& map) {
    const int x = std::clamp(static_cast<int>(std::lround(state.x)),
                             0, std::max(0, map.width() - 1));
    const int y = std::clamp(static_cast<int>(std::lround(state.y)),
                             0, std::max(0, map.height() - 1));
    return {x, y};
}

bool isAlreadyPlaced(const std::vector<Point2i>& placed, int x, int y) {
    return std::any_of(placed.begin(), placed.end(),
                       [x, y](const Point2i& point) {
                           return point.x == x && point.y == y;
                       });
}

bool insertObstacleAhead(GridMap& map, const Path2d& path,
                         const State& state, std::vector<Point2i>& placed,
                         Point2i& obstacle) {
    if (path.size() < 2) return false;
    const std::size_t current = closestPathIndex(path, state);
    const std::size_t first = std::min(current + 5, path.size() - 1);
    for (std::size_t offset = 0; offset < path.size(); ++offset) {
        const std::size_t index = std::min(first + offset, path.size() - 1);
        const int x = static_cast<int>(std::round(path[index].x));
        const int y = static_cast<int>(std::round(path[index].y));
        if (map.isInside(x, y) && map.isFree(x, y) &&
            !isAlreadyPlaced(placed, x, y)) {
            map.setOccupied(x, y, true);
            obstacle = {x, y};
            placed.push_back(obstacle);
            return true;
        }
    }
    return false;
}

}  // namespace

int main(int argc, char** argv) {
    std::string map_path = "autoplanner/data/maps/simple_50x50.txt";
    std::string output = "autoplanner/results/dynamic_navigation.csv";
    std::string summary_output;
    std::string controller_name = "mpc";
    int frames = 5;
    int steps_per_frame = 40;
    double dt = 0.05;
    double wheelbase = 1.0;
    double max_velocity = 2.0;
    double max_acceleration = 1.5;
    double max_deceleration = 2.0;
    double max_steering = 0.7;
    double max_steering_rate = 1.5;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--map" && i + 1 < argc) map_path = argv[++i];
        else if (arg == "--output" && i + 1 < argc) output = argv[++i];
        else if (arg == "--summary" && i + 1 < argc) summary_output = argv[++i];
        else if (arg == "--controller" && i + 1 < argc) controller_name = argv[++i];
        else if (arg == "--frames" && i + 1 < argc) frames = std::stoi(argv[++i]);
        else if (arg == "--steps-per-frame" && i + 1 < argc)
            steps_per_frame = std::stoi(argv[++i]);
        else if (arg == "--dt" && i + 1 < argc) dt = std::stod(argv[++i]);
        else if (arg == "--wheelbase" && i + 1 < argc)
            wheelbase = std::stod(argv[++i]);
        else if (arg == "--max-velocity" && i + 1 < argc)
            max_velocity = std::stod(argv[++i]);
        else if (arg == "--max-acceleration" && i + 1 < argc)
            max_acceleration = std::stod(argv[++i]);
        else if (arg == "--max-deceleration" && i + 1 < argc)
            max_deceleration = std::stod(argv[++i]);
        else if (arg == "--max-steering" && i + 1 < argc)
            max_steering = std::stod(argv[++i]);
        else if (arg == "--max-steering-rate" && i + 1 < argc)
            max_steering_rate = std::stod(argv[++i]);
        else if (arg == "--help") {
            std::cout << "Dynamic navigation benchmark\n"
                      << "  --map PATH              occupancy grid map\n"
                      << "  --frames N              dynamic obstacle updates\n"
                      << "  --steps-per-frame N     control steps per update\n"
                      << "  --controller stanley|mpc\n"
                      << "  --dt N                  simulation timestep\n"
                      << "  --wheelbase N           vehicle wheelbase\n"
                      << "  --max-velocity N        velocity execution limit\n"
                      << "  --max-acceleration N    acceleration limit\n"
                      << "  --max-deceleration N    braking limit\n"
                      << "  --max-steering N        steering angle limit\n"
                      << "  --max-steering-rate N   steering rate limit\n"
                      << "  --output PATH           per-step CSV\n"
                      << "  --summary PATH          JSON summary\n";
            return 0;
        }
    }

    if (controller_name != "stanley" && controller_name != "mpc") {
        std::cerr << "Unknown controller: " << controller_name << "\n";
        return 1;
    }
#ifndef AUTOMPC_HAS_EIGEN
    if (controller_name == "mpc") {
        std::cerr << "MPC requires Eigen3; rebuild with Eigen3 available.\n";
        return 1;
    }
#endif

    GridMap map;
    if (!map.loadFromTxt(map_path)) {
        std::cerr << "Failed to load map: " << map_path << "\n";
        return 1;
    }

    const Point2i start{1, 1};
    const Point2i goal{map.width() - 2, map.height() - 2};
    DStarLitePlanner dstar(true);
    AStarPlanner astar(true);
    auto initial = dstar.plan(map, start, goal);
    if (!initial.success) {
        std::cerr << "Initial D* Lite planning failed: " << initial.message << "\n";
        return 2;
    }

    Path2d current_path = initial.path;
    Trajectory trajectory = makeTrajectory(current_path);
    if (trajectory.empty()) return 2;
    State state{trajectory.front().x, trajectory.front().y,
                trajectory.front().theta, 0.0};
    SimulationOptions simulation_options;
    simulation_options.dt = dt;
    simulation_options.wheelbase = wheelbase;
    simulation_options.max_velocity = max_velocity;
    simulation_options.max_acceleration = max_acceleration;
    simulation_options.max_deceleration = max_deceleration;
    simulation_options.max_steering = max_steering;
    simulation_options.max_steering_rate = max_steering_rate;
    KinematicBicycleSimulator simulator(state, simulation_options);

    StanleyController stanley(0.5, wheelbase);
#ifdef AUTOMPC_HAS_EIGEN
    MPCController mpc(15, dt, wheelbase, max_velocity, max_steering,
                      max_acceleration, max_deceleration,
                      max_steering_rate);
#endif

    const std::filesystem::path output_path(output);
    if (!output_path.parent_path().empty()) {
        std::filesystem::create_directories(output_path.parent_path());
    }
    if (summary_output.empty()) {
        auto default_summary_path = output_path;
        default_summary_path.replace_extension(".json");
        summary_output = default_summary_path.string();
    }
    std::ofstream csv(output);
    if (!csv.is_open()) {
        std::cerr << "Failed to open output: " << output << "\n";
        return 1;
    }
    csv << "frame,step,x,y,theta,velocity,steering,replanned,obstacle_x,"
           "obstacle_y,dstar_replan_ms,astar_replan_ms,cross_track,"
           "steering_delta,velocity_delta,safe_stop\n";
    csv << std::fixed << std::setprecision(6);

    std::vector<Point2i> placed_obstacles;
    Control previous_control{};
    bool has_previous_control = false;
    int replanning_count = 0;
    bool safe_stop = false;
    int safe_stop_steps = 0;
    double safe_stop_distance = 0.0;
    int frames_run = 0;
    double dstar_total_ms = 0.0;
    double astar_total_ms = 0.0;
    double steering_delta_sum = 0.0;
    double velocity_delta_sum = 0.0;
    double max_steering_delta = 0.0;
    double max_velocity_delta = 0.0;
    int control_samples = 0;
    int replan_control_samples = 0;
    double replan_steering_delta_sum = 0.0;
    double replan_velocity_delta_sum = 0.0;
    double max_replan_steering_delta = 0.0;
    double max_replan_velocity_delta = 0.0;

    for (int frame = 0; frame < frames; ++frame) {
        Point2i obstacle{-1, -1};
        if (frame > 0) {
            insertObstacleAhead(map, current_path, state, placed_obstacles,
                                obstacle);
        }

        bool replanned = false;
        double dstar_time_ms = 0.0;
        double astar_time_ms = 0.0;
        GridCollisionChecker checker(map);
        if (!checker.isPathValid(current_path)) {
            const Point2i current = stateCell(state, map);
            const auto dstar_start = std::chrono::steady_clock::now();
            const auto dstar_result = dstar.replan(map, current);
            const auto dstar_end = std::chrono::steady_clock::now();
            dstar_time_ms = std::chrono::duration<double, std::milli>(
                dstar_end - dstar_start).count();

            const auto astar_start = std::chrono::steady_clock::now();
            const auto astar_result = astar.plan(map, current, goal);
            const auto astar_end = std::chrono::steady_clock::now();
            astar_time_ms = std::chrono::duration<double, std::milli>(
                astar_end - astar_start).count();

            dstar_total_ms += dstar_time_ms;
            astar_total_ms += astar_time_ms;
            ++replanning_count;
            replanned = dstar_result.success;
            if (!dstar_result.success) {
                std::cerr << "D* Lite replan failed at frame " << frame << "\n";
                safe_stop = true;
                for (int stop_step = 0; stop_step < steps_per_frame;
                     ++stop_step) {
                    const Control stop_control{0.0, 0.0};
                    const double steering_delta = has_previous_control
                        ? std::abs(stop_control.steering -
                                   previous_control.steering) : 0.0;
                    const double velocity_delta = has_previous_control
                        ? std::abs(stop_control.velocity -
                                   previous_control.velocity) : 0.0;
                    const State before_stop = state;
                    state = simulator.step(stop_control);
                    safe_stop_distance += std::hypot(
                        state.x - before_stop.x, state.y - before_stop.y);
                    const double cross_track = trajectory.empty()
                        ? 0.0 : closestPointDistance(trajectory, state);
                    csv << frame << "," << stop_step << ","
                        << state.x << "," << state.y << "," << state.theta << ","
                        << stop_control.velocity << ","
                        << stop_control.steering << ",0,"
                        << obstacle.x << "," << obstacle.y << ","
                        << dstar_time_ms << "," << astar_time_ms << ","
                        << cross_track << "," << steering_delta << ","
                        << velocity_delta << ",1\n";
                    previous_control = stop_control;
                    has_previous_control = true;
                    ++safe_stop_steps;
                    if (state.v <= 1e-6) break;
                }
                break;
            }
            current_path = dstar_result.path;
            trajectory = makeTrajectory(current_path);
            if (trajectory.empty()) break;
#ifdef AUTOMPC_HAS_EIGEN
            if (controller_name == "mpc") mpc.resetReferenceProgress();
#endif
            // Keep the controller state and previous command across replans;
            // the resulting command jump is part of the continuity metric.
        }

        ++frames_run;
        for (int control_step = 0; control_step < steps_per_frame; ++control_step) {
            if (trajectory.empty()) break;
            const std::size_t nearest = closestTrajectoryIndex(trajectory, state);
            const auto& reference = trajectory[nearest];
            Control control;
            if (controller_name == "stanley") {
                control = stanley.compute(state, reference, reference.v);
            } else {
#ifdef AUTOMPC_HAS_EIGEN
                control = mpc.compute(state, trajectory, reference.v);
#endif
            }

            const double steering_delta = has_previous_control
                ? std::abs(control.steering - previous_control.steering) : 0.0;
            const double velocity_delta = has_previous_control
                ? std::abs(control.velocity - previous_control.velocity) : 0.0;
            if (has_previous_control) {
                steering_delta_sum += steering_delta;
                velocity_delta_sum += velocity_delta;
                max_steering_delta = std::max(max_steering_delta, steering_delta);
                max_velocity_delta = std::max(max_velocity_delta, velocity_delta);
                ++control_samples;
                if (replanned && control_step == 0) {
                    replan_steering_delta_sum += steering_delta;
                    replan_velocity_delta_sum += velocity_delta;
                    max_replan_steering_delta =
                        std::max(max_replan_steering_delta, steering_delta);
                    max_replan_velocity_delta =
                        std::max(max_replan_velocity_delta, velocity_delta);
                    ++replan_control_samples;
                }
            }

            state = simulator.step(control);
            const double cross_track = closestPointDistance(trajectory, state);
            csv << frame << "," << control_step << ","
                << state.x << "," << state.y << "," << state.theta << ","
                << control.velocity << "," << control.steering << ","
                << (replanned && control_step == 0 ? 1 : 0) << ","
                << obstacle.x << "," << obstacle.y << ","
                << (control_step == 0 ? dstar_time_ms : 0.0) << ","
                << (control_step == 0 ? astar_time_ms : 0.0) << ","
                << cross_track << "," << steering_delta << ","
                << velocity_delta << ",0\n";
            previous_control = control;
            has_previous_control = true;
        }

        if (std::hypot(state.x - goal.x, state.y - goal.y) < 0.75) break;
    }

    const std::filesystem::path summary_path(summary_output);
    if (!summary_path.parent_path().empty()) {
        std::filesystem::create_directories(summary_path.parent_path());
    }
    std::ofstream summary(summary_output);
    if (!summary.is_open()) {
        std::cerr << "Failed to open summary: " << summary_output << "\n";
        return 1;
    }
    const double mean_steering_delta = control_samples > 0
        ? steering_delta_sum / control_samples : 0.0;
    const double mean_velocity_delta = control_samples > 0
        ? velocity_delta_sum / control_samples : 0.0;
    const double mean_replan_steering_delta = replan_control_samples > 0
        ? replan_steering_delta_sum / replan_control_samples : 0.0;
    const double mean_replan_velocity_delta = replan_control_samples > 0
        ? replan_velocity_delta_sum / replan_control_samples : 0.0;
    const bool final_success = std::hypot(state.x - goal.x, state.y - goal.y) < 0.75;
    summary << std::fixed << std::setprecision(6)
            << "{\n"
            << "  \"map\": \"" << map_path << "\",\n"
            << "  \"controller\": \"" << controller_name << "\",\n"
            << "  \"frames_requested\": " << frames << ",\n"
            << "  \"frames_run\": " << frames_run << ",\n"
            << "  \"replanning_count\": " << replanning_count << ",\n"
            << "  \"dstar_total_time_ms\": " << dstar_total_ms << ",\n"
            << "  \"astar_total_time_ms\": " << astar_total_ms << ",\n"
            << "  \"dstar_over_astar_speedup\": "
            << (dstar_total_ms > 0.0 ? astar_total_ms / dstar_total_ms : 0.0)
            << ",\n"
            << "  \"mean_steering_delta\": " << mean_steering_delta << ",\n"
            << "  \"max_steering_delta\": " << max_steering_delta << ",\n"
            << "  \"mean_velocity_delta\": " << mean_velocity_delta << ",\n"
            << "  \"max_velocity_delta\": " << max_velocity_delta << ",\n"
            << "  \"mean_replan_steering_delta\": "
            << mean_replan_steering_delta << ",\n"
            << "  \"max_replan_steering_delta\": "
            << max_replan_steering_delta << ",\n"
            << "  \"mean_replan_velocity_delta\": "
            << mean_replan_velocity_delta << ",\n"
            << "  \"max_replan_velocity_delta\": "
            << max_replan_velocity_delta << ",\n"
            << "  \"safe_stop\": " << (safe_stop ? "true" : "false") << ",\n"
            << "  \"safe_stop_steps\": " << safe_stop_steps << ",\n"
            << "  \"safe_stop_distance\": " << safe_stop_distance << ",\n"
            << "  \"final_success\": " << (final_success ? "true" : "false") << "\n"
            << "}\n";

    std::cout << "Dynamic navigation results: " << output << "\n"
              << "Summary: " << summary_output << "\n";
    return final_success ? 0 : 3;
}
