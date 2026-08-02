#pragma once

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <limits>
#include <vector>

#include "autoplanner/core/point.h"

namespace robotnav {

struct MovingObstacle {
    std::size_t start_frame = 0;
    std::size_t end_frame = 0;
    autoplanner::Point2i start_cell{-1, -1};
    int dx_per_frame = 0;
    int dy_per_frame = 0;
    // Optional obstacle footprint and prediction uncertainty, expressed in
    // map-cell coordinates. Zero preserves the original point-cell model.
    double radius = 0.0;
    double uncertainty_growth_per_frame = 0.0;
};

inline bool isValidMovingObstacle(const MovingObstacle& obstacle) {
    return obstacle.end_frame >= obstacle.start_frame &&
           std::isfinite(obstacle.radius) && obstacle.radius >= 0.0 &&
           std::isfinite(obstacle.uncertainty_growth_per_frame) &&
           obstacle.uncertainty_growth_per_frame >= 0.0;
}

inline bool predictMovingObstaclePosition(const MovingObstacle& obstacle,
                                          double frame,
                                          autoplanner::Point2d& position) {
    if (obstacle.end_frame < obstacle.start_frame ||
        !std::isfinite(frame) ||
        frame < static_cast<double>(obstacle.start_frame) ||
        frame > static_cast<double>(obstacle.end_frame)) {
        return false;
    }
    const double delta = frame -
                         static_cast<double>(obstacle.start_frame);
    position = {
        static_cast<double>(obstacle.start_cell.x) +
            static_cast<double>(obstacle.dx_per_frame) * delta,
        static_cast<double>(obstacle.start_cell.y) +
            static_cast<double>(obstacle.dy_per_frame) * delta};
    return true;
}

inline double distanceToOccupiedBox(
    const autoplanner::Point2d& position,
    const autoplanner::Point2d& lower_left) {
    // Grid coordinates describe cell interiors [x, x + 1] and
    // [y, y + 1]. This is consistent with GridCollisionChecker's floor-based
    // occupancy query and is conservative on cell boundaries.
    const double dx = std::max(
        std::abs(position.x - (lower_left.x + 0.5)) - 0.5, 0.0);
    const double dy = std::max(
        std::abs(position.y - (lower_left.y + 0.5)) - 0.5, 0.0);
    return std::hypot(dx, dy);
}

inline double distanceToOccupiedCell(const autoplanner::Point2d& position,
                                     const autoplanner::Point2i& cell) {
    return distanceToOccupiedBox(
        position,
        {static_cast<double>(cell.x), static_cast<double>(cell.y)});
}

inline double predictedObstacleClearance(
    const std::vector<MovingObstacle>& obstacles,
    const autoplanner::Point2d& position,
    double frame) {
    double minimum = std::numeric_limits<double>::infinity();
    for (const auto& obstacle : obstacles) {
        autoplanner::Point2d predicted;
        if (!predictMovingObstaclePosition(obstacle, frame, predicted)) {
            continue;
        }
        const double delta = std::max(
            0.0, frame - static_cast<double>(obstacle.start_frame));
        const double occupied_radius = std::max(
            0.0, obstacle.radius +
                     obstacle.uncertainty_growth_per_frame * delta);
        minimum = std::min(
            minimum,
            distanceToOccupiedBox(position, predicted) - occupied_radius);
    }
    return minimum;
}

inline bool isPredictedCollision(
    const std::vector<MovingObstacle>& obstacles,
    const autoplanner::Point2d& position,
    double frame,
    double collision_margin = 0.0) {
    for (const auto& obstacle : obstacles) {
        autoplanner::Point2d predicted;
        if (!predictMovingObstaclePosition(obstacle, frame, predicted)) {
            continue;
        }
        const double delta = std::max(
            0.0, frame - static_cast<double>(obstacle.start_frame));
        const double expansion = std::max(
            0.0, obstacle.radius +
                     obstacle.uncertainty_growth_per_frame * delta +
                     collision_margin);
        if (expansion <= 0.0 &&
            static_cast<int>(std::floor(position.x)) ==
                static_cast<int>(std::floor(predicted.x)) &&
            static_cast<int>(std::floor(position.y)) ==
                static_cast<int>(std::floor(predicted.y))) {
            return true;
        }
        if (distanceToOccupiedBox(position, predicted) <= expansion) {
            return true;
        }
    }
    return false;
}

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
