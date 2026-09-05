#pragma once

#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "robotnav/dynamic_obstacle_prediction.h"

namespace robotnav {

inline double predictionFrameAtTime(double elapsed_seconds,
                                    double frame_period_seconds) noexcept {
    if (!std::isfinite(elapsed_seconds) || elapsed_seconds < 0.0 ||
        !std::isfinite(frame_period_seconds) ||
        frame_period_seconds <= 0.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return elapsed_seconds / frame_period_seconds;
}

// Shared time reference for local planners that evaluate predicted obstacles.
// current_time_seconds is authoritative when supplied. current_frame remains
// as a compatibility fallback for callers that construct the original
// four-field aggregate.
struct DynamicObstacleContext {
    const std::vector<MovingObstacle>* obstacles = nullptr;
    std::size_t current_frame = 0;
    double frame_period_seconds = 1.0;
    double collision_margin = 0.0;
    double current_time_seconds =
        std::numeric_limits<double>::quiet_NaN();

    double predictionFrameAfter(double rollout_seconds) const noexcept {
        if (!std::isfinite(rollout_seconds)) {
            return std::numeric_limits<double>::quiet_NaN();
        }
        if (std::isfinite(current_time_seconds)) {
            return predictionFrameAtTime(
                current_time_seconds + rollout_seconds,
                frame_period_seconds);
        }
        if (!std::isfinite(frame_period_seconds) ||
            frame_period_seconds <= 0.0) {
            return std::numeric_limits<double>::quiet_NaN();
        }
        return static_cast<double>(current_frame) +
               rollout_seconds / frame_period_seconds;
    }
};

}  // namespace robotnav
