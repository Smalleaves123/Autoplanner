#pragma once

#include <string>

#include "autompc/core/trajectory.h"
#include "autompc/core/types.h"
#include "autoplanner/core/grid_map.h"
#include "robotnav/status.h"

namespace robotnav {

struct SafetyOptions {
    double goal_tolerance = 0.75;
    double max_cross_track_error = 5.0;
    double max_velocity = 2.0;
    double max_steering = 0.7;
    bool enforce_collision = true;
};

struct SafetyDecision {
    bool safe = false;
    StatusCode status = StatusCode::InternalError;
    std::string message;
};

// Lightweight, ROS-free execution guard. It validates the reference,
// controller commands, and simulator states before the next cycle is allowed
// to continue.
class SafetySupervisor {
public:
    SafetySupervisor(const autoplanner::GridMap& map,
                     SafetyOptions options = {});

    SafetyDecision validateTrajectory(const autompc::Trajectory& trajectory) const;
    SafetyDecision validateCommand(const autompc::Control& command) const;
    SafetyDecision validateState(const autompc::State& state) const;

    bool goalReached(const autompc::State& state,
                     const autompc::Trajectory& trajectory) const;

    const SafetyOptions& options() const { return options_; }

private:
    const autoplanner::GridMap& map_;
    SafetyOptions options_;
};

}  // namespace robotnav
