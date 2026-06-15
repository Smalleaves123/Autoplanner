#pragma once

#include <vector>
#include "autompc/core/types.h"
#include "autompc/core/trajectory.h"

namespace autompc {

// Error metrics for evaluating trajectory tracking performance.
struct TrackingErrors {
    double max_cross_track = 0.0;
    double mean_cross_track = 0.0;
    double max_heading_err = 0.0;
    double mean_heading_err = 0.0;
};

// Compute tracking errors between actual and reference trajectories.
TrackingErrors computeErrors(const std::vector<State>& actual,
                              const Trajectory& reference);

}  // namespace autompc
