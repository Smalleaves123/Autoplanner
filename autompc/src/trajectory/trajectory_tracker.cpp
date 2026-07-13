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
    double path_s = 0.0;
    const double total_path_s = arcLength(reference);
    while (t < max_time) {
        // Advance along the reference by its desired speed.  The trajectory
        // interpolation parameter is arc length, not wall-clock time.
        const auto ref = interpolate(reference, path_s);
        auto u = controller.compute(s, ref, dt);
        s = step(s, u, dt);
        states.push_back(s);
        path_s += std::max(0.0, ref.v) * dt;
        t += dt;

        if (path_s >= total_path_s &&
            std::hypot(s.x - reference.back().x,
                       s.y - reference.back().y) <= 0.75) {
            break;
        }
    }
    return states;
}

}  // namespace autompc
