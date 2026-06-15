#include "autompc/trajectory/trajectory_tracker.h"
#include "autompc/core/trajectory.h"

namespace autompc {

std::vector<State> simulate(const State& initial,
                             const Trajectory& reference,
                             PIDController& controller,
                             double dt, double max_time) {
    std::vector<State> states;
    if (reference.empty() || dt <= 0.0 || max_time <= 0.0) return states;
    State s = initial;
    double t = 0.0;
    while (t < max_time) {
        // Find closest reference point
        const auto& ref = interpolate(reference, t);
        auto u = controller.compute(s, ref, dt);
        s = step(s, u, dt);
        states.push_back(s);
        t += dt;
    }
    return states;
}

}  // namespace autompc
