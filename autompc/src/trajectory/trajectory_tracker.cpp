#include "autompc/trajectory/trajectory_tracker.h"
#include "autompc/core/trajectory.h"

#include <algorithm>
#include <cmath>

namespace autompc {

std::vector<State> simulate(const State& initial,
                             const Trajectory& reference,
                             PIDController& controller,
                             double dt, double max_time) {
    SimulationOptions options;
    options.dt = dt;
    return simulate(initial, reference, controller, options, max_time);
}

std::vector<State> simulate(const State& initial,
                             const Trajectory& reference,
                             PIDController& controller,
                             const SimulationOptions& options,
                             double max_time) {
    std::vector<State> states;
    if (reference.empty() || options.dt <= 0.0 || max_time <= 0.0) {
        return states;
    }
    KinematicBicycleSimulator simulator(initial, options);
    double t = 0.0;
    double path_s = 0.0;
    const double total_path_s = arcLength(reference);
    while (t < max_time) {
        // Advance along the reference by its desired speed.  The trajectory
        // interpolation parameter is arc length, not wall-clock time.
        const auto ref = interpolate(reference, path_s);
        auto u = controller.compute(simulator.state(), ref, options.dt);
        const State s = simulator.step(u);
        states.push_back(s);
        path_s += std::max(0.0, ref.v) * options.dt;
        t += options.dt;

        if (path_s >= total_path_s &&
            std::hypot(s.x - reference.back().x,
                       s.y - reference.back().y) <= 0.75) {
            break;
        }
    }
    return states;
}

}  // namespace autompc
