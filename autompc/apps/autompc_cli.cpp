// autompc_cli.cpp — AutoMPC command-line trajectory tracker.
//
// Usage:
//   ./build/apps/autompc_cli --controller pid --trajectory circle --radius 5.0 --steps 500
//
// Build:
//   cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <algorithm>
#include <cmath>
#include <limits>

#include "autompc/autompc.h"

using namespace autompc;

namespace {

TrajectoryPoint closestReferencePoint(const Trajectory& trajectory,
                                      const State& state) {
    TrajectoryPoint best = trajectory.front();
    double best_distance = std::numeric_limits<double>::max();
    for (const auto& point : trajectory) {
        const double dx = point.x - state.x;
        const double dy = point.y - state.y;
        const double distance = std::sqrt(dx * dx + dy * dy);
        if (distance < best_distance) {
            best_distance = distance;
            best = point;
        }
    }
    return best;
}

bool reachedPathGoal(const State& state, const Trajectory& trajectory) {
    if (trajectory.empty()) return false;
    const auto& goal = trajectory.back();
    return std::hypot(state.x - goal.x, state.y - goal.y) <= 0.75;
}

}  // namespace

int main(int argc, char** argv) {
    std::string controller_name = "pid";
    std::string trajectory_type = "circle";
    double radius = 5.0;
    double velocity = 1.0;
    int steps = 500;
    double dt = 0.05;
    double wheelbase = 1.0;
    int mpc_horizon = 15;
    double max_velocity = 2.0;
    double max_steering = 0.7;
    double max_acceleration = 1.5;
    double max_deceleration = 2.0;
    double max_steering_rate = 1.5;
    double sample_spacing = 0.5;
    double max_lateral_acceleration = 1.5;
    std::string output = "results/tracking.csv";
    std::string path_file;
    std::string metrics_output;
    std::string trajectory_output;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--controller" && i+1 < argc) controller_name = argv[++i];
        else if (a == "--trajectory" && i+1 < argc) trajectory_type = argv[++i];
        else if (a == "--path" && i+1 < argc) path_file = argv[++i];
        else if (a == "--radius" && i+1 < argc) radius = std::stod(argv[++i]);
        else if (a == "--velocity" && i+1 < argc) velocity = std::stod(argv[++i]);
        else if (a == "--steps" && i+1 < argc) steps = std::stoi(argv[++i]);
        else if (a == "--dt" && i+1 < argc) dt = std::stod(argv[++i]);
        else if (a == "--wheelbase" && i+1 < argc)
            wheelbase = std::stod(argv[++i]);
        else if (a == "--mpc-horizon" && i+1 < argc)
            mpc_horizon = std::stoi(argv[++i]);
        else if (a == "--max-velocity" && i+1 < argc)
            max_velocity = std::stod(argv[++i]);
        else if (a == "--max-steering" && i+1 < argc)
            max_steering = std::stod(argv[++i]);
        else if (a == "--max-acceleration" && i+1 < argc)
            max_acceleration = std::stod(argv[++i]);
        else if (a == "--max-deceleration" && i+1 < argc)
            max_deceleration = std::stod(argv[++i]);
        else if (a == "--max-steering-rate" && i+1 < argc)
            max_steering_rate = std::stod(argv[++i]);
        else if (a == "--sample-spacing" && i+1 < argc)
            sample_spacing = std::stod(argv[++i]);
        else if (a == "--max-lateral-acceleration" && i+1 < argc)
            max_lateral_acceleration = std::stod(argv[++i]);
        else if (a == "--output" && i+1 < argc) output = argv[++i];
        else if (a == "--metrics" && i+1 < argc) metrics_output = argv[++i];
        else if (a == "--trajectory-output" && i+1 < argc)
            trajectory_output = argv[++i];
        else if (a == "--help") {
            std::cout << "AutoMPC CLI\n"
                << "  --controller  pid | pure_pursuit | stanley | mpc\n"
                << "  --trajectory  circle | line | path\n"
                << "  --path PATH   AutoPlanner x,y CSV (required for path)\n"
                << "  --radius N    circle radius (default 5.0)\n"
                << "  --velocity N  target velocity (default 1.0)\n"
                << "  --steps N     simulation steps (default 500)\n"
                << "  --dt N        timestep (default 0.05)\n"
                << "  --wheelbase N vehicle wheelbase (default 1.0)\n"
                << "  --mpc-horizon N  MPC prediction horizon (default 15)\n"
                << "  --max-velocity N  MPC velocity constraint\n"
                << "  --max-steering N  MPC steering constraint\n"
                << "  --max-acceleration N  MPC acceleration constraint\n"
                << "  --max-deceleration N  MPC deceleration constraint\n"
                << "  --max-steering-rate N  MPC steering rate constraint\n"
                << "  --sample-spacing N  path trajectory sample spacing\n"
                << "  --max-lateral-acceleration N  curvature speed limit\n"
                << "  --output PATH CSV output path\n"
                << "  --metrics PATH JSON metrics output (optional)\n"
                << "  --trajectory-output PATH generated reference CSV\n";
            return 0;
        }
    }

    SimulationOptions simulation_options;
    simulation_options.dt = dt;
    simulation_options.wheelbase = wheelbase;
    simulation_options.max_velocity = max_velocity;
    simulation_options.max_acceleration = max_acceleration;
    simulation_options.max_deceleration = max_deceleration;
    simulation_options.max_steering = max_steering;
    simulation_options.max_steering_rate = max_steering_rate;

    // Generate reference trajectory
    Trajectory ref;
    if (trajectory_type == "circle")
        ref = makeCircle(radius, velocity, steps);
    else if (trajectory_type == "line")
        ref = makeStraightLine(0, 0, 10, 0, velocity, steps);
    else if (trajectory_type == "path") {
        TrajectoryOptions trajectory_options;
        trajectory_options.sample_spacing = sample_spacing;
        trajectory_options.target_velocity = velocity;
        trajectory_options.max_velocity = max_velocity;
        trajectory_options.max_acceleration = max_acceleration;
        trajectory_options.max_deceleration = max_deceleration;
        trajectory_options.max_lateral_acceleration =
            max_lateral_acceleration;
        if (path_file.empty() || !loadPathCsv(
                path_file, velocity, ref, trajectory_options)) {
            std::cerr << "Failed to load path CSV: " << path_file << "\n";
            return 1;
        }
        const double distance_per_step = std::max(velocity * dt, 1e-6);
        const int path_tracking_steps = static_cast<int>(
            std::ceil(1.5 * arcLength(ref) / distance_per_step)) + 100;
        steps = std::max({steps, static_cast<int>(ref.size()),
                          path_tracking_steps});
    } else {
        std::cerr << "Unknown trajectory: " << trajectory_type << "\n";
        return 1;
    }

    if (trajectory_type == "path" && trajectory_output.empty()) {
        trajectory_output = (std::filesystem::path(output).parent_path() /
                             "trajectory.csv").string();
    }
    if (!trajectory_output.empty()) {
        const auto trajectory_parent =
            std::filesystem::path(trajectory_output).parent_path();
        if (!trajectory_parent.empty()) {
            std::filesystem::create_directories(trajectory_parent);
        }
        if (!saveTrajectoryCsv(ref, trajectory_output)) {
            std::cerr << "Failed to save generated trajectory: "
                      << trajectory_output << "\n";
            return 1;
        }
    }

    // Start slightly off the reference to make tracking quality measurable.
    State initial;
    if (trajectory_type == "circle") {
        initial = {radius, 0.0, M_PI_2, 0.0};
    } else {
        initial = {ref.front().x, ref.front().y + 0.5, ref.front().theta, 0.0};
    }

    // Run simulation
    std::vector<State> actual;
    if (controller_name == "pid") {
        PIDController pid(1.0, 0.0, 0.0, 2.0, 0.0, 0.5, wheelbase);
        actual = simulate(initial, ref, pid, simulation_options, steps * dt);
    } else if (controller_name == "pure_pursuit") {
        KinematicBicycleSimulator simulator(initial, simulation_options);
        PurePursuitController pp(2.0, wheelbase);
        for (int i = 0; i < steps; ++i) {
            const auto nearest = closestReferencePoint(ref, simulator.state());
            auto u = pp.compute(simulator.state(), ref, nearest.v);
            const State s = simulator.step(u);
            actual.push_back(s);
            if (trajectory_type == "path" && reachedPathGoal(s, ref)) break;
        }
    } else if (controller_name == "stanley") {
        KinematicBicycleSimulator simulator(initial, simulation_options);
        StanleyController stanley(0.5, wheelbase);
        for (int i = 0; i < steps; ++i) {
            const auto nearest = closestReferencePoint(ref, simulator.state());
            auto u = stanley.compute(simulator.state(), nearest, nearest.v);
            const State s = simulator.step(u);
            actual.push_back(s);
            if (trajectory_type == "path" && reachedPathGoal(s, ref)) break;
        }
    } else if (controller_name == "mpc") {
#ifdef AUTOMPC_HAS_EIGEN
        KinematicBicycleSimulator simulator(initial, simulation_options);
        MPCController mpc(mpc_horizon, dt, wheelbase,
                          max_velocity, max_steering,
                          max_acceleration, max_deceleration,
                          max_steering_rate);
        for (int i = 0; i < steps; ++i) {
            const auto nearest = closestReferencePoint(ref, simulator.state());
            const auto u = mpc.compute(simulator.state(), ref, nearest.v);
            const State s = simulator.step(u);
            actual.push_back(s);
            if (trajectory_type == "path" && reachedPathGoal(s, ref)) break;
        }
#else
        std::cerr << "MPC requires Eigen3; rebuild with Eigen3 available.\n";
        return 1;
#endif
    } else {
        std::cerr << "Unknown controller: " << controller_name << "\n";
        return 1;
    }

    // Write CSV: x_actual, y_actual, x_ref, y_ref. The reference column is
    // sampled by arc length so it matches the reference used by PID.
    std::filesystem::create_directories(
        std::filesystem::path(output).parent_path());
    std::ofstream fout(output);
    fout << "x_actual,y_actual,x_ref,y_ref\n";
    fout << std::fixed << std::setprecision(6);
    double reference_s = 0.0;
    for (size_t i = 0; i < actual.size(); ++i) {
        const auto reference_point = interpolate(ref, reference_s);
        fout << actual[i].x << "," << actual[i].y << ","
             << reference_point.x << "," << reference_point.y << "\n";
        reference_s += std::max(0.0, reference_point.v) * dt;
    }

    // Compute errors
    auto errors = computeErrors(actual, ref);
    const double goal_distance = actual.empty()
        ? 0.0
        : std::hypot(actual.back().x - ref.back().x,
                     actual.back().y - ref.back().y);
    const bool goal_reached = trajectory_type == "path" &&
                              !actual.empty() && reachedPathGoal(actual.back(), ref);
    std::cout << "Controller: " << controller_name << "\n"
              << "Trajectory: " << trajectory_type
              << " (r=" << radius << ", v=" << velocity << ")\n"
              << "Steps: " << actual.size() << "\n"
              << "Max cross-track error: " << errors.max_cross_track << " m\n"
              << "Mean cross-track error: " << errors.mean_cross_track << " m\n"
              << "Max heading error: " << errors.max_heading_err << " rad\n"
              << "Mean heading error: " << errors.mean_heading_err << " rad\n"
              << "Goal reached: " << (goal_reached ? "yes" : "no") << "\n"
              << "Output: " << output << "\n";

    if (metrics_output.empty()) {
        metrics_output = (std::filesystem::path(output).parent_path()
                          / "tracking_metrics.json").string();
    }
    std::ofstream metrics(metrics_output);
    if (metrics.is_open()) {
        metrics << "{\n"
                << "  \"controller\": \"" << controller_name << "\",\n"
                << "  \"trajectory\": \"" << trajectory_type << "\",\n"
                << "  \"steps\": " << actual.size() << ",\n"
                << "  \"max_cross_track\": " << errors.max_cross_track << ",\n"
                << "  \"mean_cross_track\": " << errors.mean_cross_track << ",\n"
                << "  \"max_heading_error\": " << errors.max_heading_err << ",\n"
                << "  \"mean_heading_error\": " << errors.mean_heading_err << ",\n"
                << "  \"goal_reached\": " << (goal_reached ? "true" : "false") << ",\n"
                << "  \"goal_distance\": " << goal_distance << "\n"
                << "}\n";
    }

    return 0;
}
