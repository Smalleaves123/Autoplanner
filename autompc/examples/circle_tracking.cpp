// circle_tracking.cpp — Circle trajectory tracking demo with all controllers.
//
// Generates a circle trajectory and runs all three controllers against it,
// writing CSV output files for visualization.

#include <fstream>
#include <iostream>
#include <string>

#include "autompc/autompc.h"

using namespace autompc;

int main() {
    double radius = 5.0;
    double velocity = 1.0;
    double dt = 0.05;
    double wheelbase = 1.0;
    int steps = 400;
    double max_time = steps * dt;  // 20 seconds

    // Reference trajectory
    auto ref = makeCircle(radius, velocity, steps);

    // Initial state with offset
    State s0{radius + 1.0, 0.0, M_PI_2, 0.0};

    std::cout << "=== Circle Tracking Demo ===\n";
    std::cout << "Radius: " << radius << " m, Velocity: " << velocity
              << " m/s, Steps: " << steps << "\n\n";

    auto write_csv = [&](const std::string& path,
                          const std::vector<State>& actual) {
        std::ofstream f(path);
        f << "x,y,theta,v,x_ref,y_ref,theta_ref\n";
        f << std::fixed << std::setprecision(6);
        for (size_t i = 0; i < actual.size() && i < ref.size(); ++i) {
            f << actual[i].x << "," << actual[i].y << ","
              << actual[i].theta << "," << actual[i].v << ","
              << ref[i].x << "," << ref[i].y << "," << ref[i].theta << "\n";
        }
    };

    // --- PID ---
    {
        PIDController pid(1.0, 0.0, 0.0, 2.0, 0.0, 0.5, wheelbase);
        auto actual = simulate(s0, ref, pid, dt, max_time);
        auto err = computeErrors(actual, ref);
        std::cout << "PID:       max_cte=" << err.max_cross_track
                  << " mean_cte=" << err.mean_cross_track << "\n";
        write_csv("results/circle_pid.csv", actual);
    }

    // --- Pure Pursuit ---
    {
        State s = s0;
        PurePursuitController pp(2.0, wheelbase);
        std::vector<State> actual;
        for (int i = 0; i < steps; ++i) {
            auto u = pp.compute(s, ref, velocity);
            s = step(s, u, dt);
            actual.push_back(s);
        }
        auto err = computeErrors(actual, ref);
        std::cout << "PurePurs:  max_cte=" << err.max_cross_track
                  << " mean_cte=" << err.mean_cross_track << "\n";
        write_csv("results/circle_pure_pursuit.csv", actual);
    }

    // --- Stanley ---
    {
        State s = s0;
        StanleyController stanley(0.5, wheelbase);
        std::vector<State> actual;
        for (int i = 0; i < steps; ++i) {
            size_t idx = std::min(static_cast<size_t>(i), ref.size() - 1);
            auto u = stanley.compute(s, ref[idx], velocity);
            s = step(s, u, dt);
            actual.push_back(s);
        }
        auto err = computeErrors(actual, ref);
        std::cout << "Stanley:   max_cte=" << err.max_cross_track
                  << " mean_cte=" << err.mean_cross_track << "\n";
        write_csv("results/circle_stanley.csv", actual);
    }

    std::cout << "\nResults written to results/circle_*.csv\n";
    std::cout << "Visualize: python ../AutoPlanner/scripts/plot_tracking.py results/circle_pid.csv\n";
    return 0;
}
