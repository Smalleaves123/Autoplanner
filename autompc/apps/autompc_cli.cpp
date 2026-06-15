// autompc_cli.cpp — AutoMPC command-line trajectory tracker.
//
// Usage:
//   ./build/apps/autompc_cli --controller pid --trajectory circle --radius 5.0 --steps 500
//
// Build:
//   cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j

#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>

#include "autompc/autompc.h"

using namespace autompc;

int main(int argc, char** argv) {
    std::string controller_name = "pid";
    std::string trajectory_type = "circle";
    double radius = 5.0;
    double velocity = 1.0;
    int steps = 500;
    double dt = 0.05;
    double wheelbase = 1.0;
    std::string output = "results/tracking.csv";

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--controller" && i+1 < argc) controller_name = argv[++i];
        else if (a == "--trajectory" && i+1 < argc) trajectory_type = argv[++i];
        else if (a == "--radius" && i+1 < argc) radius = std::stod(argv[++i]);
        else if (a == "--velocity" && i+1 < argc) velocity = std::stod(argv[++i]);
        else if (a == "--steps" && i+1 < argc) steps = std::stoi(argv[++i]);
        else if (a == "--dt" && i+1 < argc) dt = std::stod(argv[++i]);
        else if (a == "--output" && i+1 < argc) output = argv[++i];
        else if (a == "--help") {
            std::cout << "AutoMPC CLI\n"
                << "  --controller  pid | pure_pursuit | stanley\n"
                << "  --trajectory  circle | line\n"
                << "  --radius N    circle radius (default 5.0)\n"
                << "  --velocity N  target velocity (default 1.0)\n"
                << "  --steps N     simulation steps (default 500)\n"
                << "  --dt N        timestep (default 0.05)\n"
                << "  --output PATH CSV output path\n";
            return 0;
        }
    }

    // Generate reference trajectory
    Trajectory ref;
    if (trajectory_type == "circle")
        ref = makeCircle(radius, velocity, steps);
    else
        ref = makeStraightLine(0, 0, 10, 0, velocity, steps);

    // Initial state: slightly offset to create tracking error
    State initial{radius, 0.0, M_PI_2, 0.0};

    // Run simulation
    std::vector<State> actual;
    if (controller_name == "pid") {
        PIDController pid(1.0, 0.0, 0.0, 2.0, 0.0, 0.5, wheelbase);
        actual = simulate(initial, ref, pid, dt, steps * dt);
    } else if (controller_name == "pure_pursuit") {
        // Pure pursuit uses a different API (needs full trajectory not just ref point)
        State s = initial;
        PurePursuitController pp(2.0, wheelbase);
        for (int i = 0; i < steps; ++i) {
            auto u = pp.compute(s, ref, velocity);
            s = step(s, u, dt);
            actual.push_back(s);
        }
    } else if (controller_name == "stanley") {
        State s = initial;
        StanleyController stanley(0.5, wheelbase);
        for (int i = 0; i < steps; ++i) {
            size_t idx = std::min(static_cast<size_t>(i), ref.size() - 1);
            auto u = stanley.compute(s, ref[idx], velocity);
            s = step(s, u, dt);
            actual.push_back(s);
        }
    } else {
        std::cerr << "Unknown controller: " << controller_name << "\n";
        return 1;
    }

    // Write CSV: x_actual, y_actual, x_ref, y_ref
    std::filesystem::create_directories(
        std::filesystem::path(output).parent_path());
    std::ofstream fout(output);
    fout << "x_actual,y_actual,x_ref,y_ref\n";
    fout << std::fixed << std::setprecision(6);
    for (size_t i = 0; i < actual.size() && i < ref.size(); ++i) {
        fout << actual[i].x << "," << actual[i].y << ","
             << ref[i].x << "," << ref[i].y << "\n";
    }

    // Compute errors
    auto errors = computeErrors(actual, ref);
    std::cout << "Controller: " << controller_name << "\n"
              << "Trajectory: " << trajectory_type
              << " (r=" << radius << ", v=" << velocity << ")\n"
              << "Steps: " << actual.size() << "\n"
              << "Max cross-track error: " << errors.max_cross_track << " m\n"
              << "Mean cross-track error: " << errors.mean_cross_track << " m\n"
              << "Max heading error: " << errors.max_heading_err << " rad\n"
              << "Mean heading error: " << errors.mean_heading_err << " rad\n"
              << "Output: " << output << "\n";

    return 0;
}
