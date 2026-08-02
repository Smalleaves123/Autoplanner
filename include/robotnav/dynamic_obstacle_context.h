#pragma once

#include <cstddef>
#include <vector>

#include "robotnav/dynamic_obstacle_prediction.h"

namespace robotnav {

// Shared time reference for local planners that evaluate predicted obstacles.
// current_frame is the dynamic-map frame at the current vehicle state and
// frame_period_seconds converts local rollout time into prediction frames.
struct DynamicObstacleContext {
    const std::vector<MovingObstacle>* obstacles = nullptr;
    std::size_t current_frame = 0;
    double frame_period_seconds = 1.0;
    double collision_margin = 0.0;
};

}  // namespace robotnav
