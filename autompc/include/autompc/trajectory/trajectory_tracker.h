#pragma once

#include <vector>
#include "autompc/core/types.h"
#include "autompc/controllers/controllers.h"

namespace autompc {

// Trajectory tracker: simulates a controller following a reference trajectory.
std::vector<State> simulate(const State& initial,
                             const Trajectory& reference,
                             PIDController& controller,
                             double dt, double max_time);

}  // namespace autompc
