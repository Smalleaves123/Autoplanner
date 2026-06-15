#include "autompc/trajectory/error_metrics.h"

#include <algorithm>
#include <cmath>

namespace autompc {

TrackingErrors computeErrors(const std::vector<State>& actual,
                              const Trajectory& reference) {
    TrackingErrors err;
    if (actual.empty() || reference.empty()) return err;

    double sum_ct = 0.0, sum_he = 0.0;
    for (auto& s : actual) {
        // Find nearest reference point
        double best_d = 1e9, best_he = 0.0;
        for (auto& r : reference) {
            double dx = r.x - s.x, dy = r.y - s.y;
            double d = std::sqrt(dx*dx + dy*dy);
            if (d < best_d) {
                best_d = d;
                double he = r.theta - s.theta;
                best_he = std::abs(std::atan2(std::sin(he), std::cos(he)));
            }
        }
        if (best_d > err.max_cross_track) err.max_cross_track = best_d;
        if (best_he > err.max_heading_err) err.max_heading_err = best_he;
        sum_ct += best_d;
        sum_he += best_he;
    }
    err.mean_cross_track = sum_ct / actual.size();
    err.mean_heading_err = sum_he / actual.size();
    return err;
}

}  // namespace autompc
