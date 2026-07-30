#pragma once

#include <cstddef>
#include <vector>

#include "autoplanner/core/point.h"

namespace robotnav {

struct MovingObstacle {
    std::size_t start_frame = 0;
    std::size_t end_frame = 0;
    autoplanner::Point2i start_cell{-1, -1};
    int dx_per_frame = 0;
    int dy_per_frame = 0;
};

inline bool predictMovingObstacleCell(const MovingObstacle& obstacle,
                                      std::size_t frame,
                                      autoplanner::Point2i& cell) {
    if (obstacle.end_frame < obstacle.start_frame ||
        frame < obstacle.start_frame || frame > obstacle.end_frame) {
        return false;
    }
    const auto delta = static_cast<int>(frame - obstacle.start_frame);
    cell = {
        obstacle.start_cell.x + obstacle.dx_per_frame * delta,
        obstacle.start_cell.y + obstacle.dy_per_frame * delta};
    return true;
}

inline bool isPredictedOccupied(
    const std::vector<MovingObstacle>& obstacles,
    const autoplanner::Point2i& cell,
    std::size_t frame) {
    for (const auto& obstacle : obstacles) {
        autoplanner::Point2i predicted;
        if (predictMovingObstacleCell(obstacle, frame, predicted) &&
            predicted == cell) {
            return true;
        }
    }
    return false;
}

}  // namespace robotnav
