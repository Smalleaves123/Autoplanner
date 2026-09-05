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

    // Optional constant acceleration, expressed in cells per frame squared.
    // The integer velocity fields above remain the compatibility-facing base
    // model used by existing scenarios and aggregate initializers.
    double acceleration_x_per_frame2 = 0.0;
    double acceleration_y_per_frame2 = 0.0;

    // Position covariance in cell^2 at start_frame. The covariance growth
    // terms are added once per frame and must also be positive semidefinite.
    double covariance_xx = 0.0;
    double covariance_xy = 0.0;
    double covariance_yy = 0.0;
    double covariance_growth_xx_per_frame = 0.0;
    double covariance_growth_xy_per_frame = 0.0;
    double covariance_growth_yy_per_frame = 0.0;
    double covariance_confidence_scale = 2.0;
};

inline bool isValidCovariance(double xx, double xy, double yy) {
    if (!std::isfinite(xx) || !std::isfinite(xy) || !std::isfinite(yy) ||
        xx < 0.0 || yy < 0.0) {
        return false;
    }
    const double determinant = xx * yy - xy * xy;
    return std::isfinite(determinant) && determinant >= 0.0;
}

inline bool isValidMovingObstacle(const MovingObstacle& obstacle) {
    return obstacle.end_frame >= obstacle.start_frame &&
           std::isfinite(obstacle.radius) && obstacle.radius >= 0.0 &&
           std::isfinite(obstacle.uncertainty_growth_per_frame) &&
           obstacle.uncertainty_growth_per_frame >= 0.0 &&
           std::isfinite(obstacle.acceleration_x_per_frame2) &&
           std::isfinite(obstacle.acceleration_y_per_frame2) &&
           isValidCovariance(obstacle.covariance_xx,
                             obstacle.covariance_xy,
                             obstacle.covariance_yy) &&
           isValidCovariance(obstacle.covariance_growth_xx_per_frame,
                             obstacle.covariance_growth_xy_per_frame,
                             obstacle.covariance_growth_yy_per_frame) &&
           std::isfinite(obstacle.covariance_confidence_scale) &&
           obstacle.covariance_confidence_scale >= 0.0;
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
            static_cast<double>(obstacle.dx_per_frame) * delta +
            0.5 * obstacle.acceleration_x_per_frame2 * delta * delta,
        static_cast<double>(obstacle.start_cell.y) +
            static_cast<double>(obstacle.dy_per_frame) * delta +
            0.5 * obstacle.acceleration_y_per_frame2 * delta * delta};
    return std::isfinite(position.x) && std::isfinite(position.y);
}

inline double largestCovarianceStandardDeviation(
    const MovingObstacle& obstacle,
    double frame) {
    if (!std::isfinite(frame) ||
        frame < static_cast<double>(obstacle.start_frame)) {
        return std::numeric_limits<double>::infinity();
    }
    const double delta = frame -
                         static_cast<double>(obstacle.start_frame);
    const double xx = obstacle.covariance_xx +
                      obstacle.covariance_growth_xx_per_frame * delta;
    const double xy = obstacle.covariance_xy +
                      obstacle.covariance_growth_xy_per_frame * delta;
    const double yy = obstacle.covariance_yy +
                      obstacle.covariance_growth_yy_per_frame * delta;
    if (!isValidCovariance(xx, xy, yy)) {
        return std::numeric_limits<double>::infinity();
    }
    const double half_trace = 0.5 * (xx + yy);
    const double half_difference = 0.5 * (xx - yy);
    const double largest_eigenvalue =
        half_trace + std::hypot(half_difference, xy);
    return std::sqrt(std::max(0.0, largest_eigenvalue));
}

inline double predictedObstacleSafetyRadius(const MovingObstacle& obstacle,
                                            double frame,
                                            double collision_margin = 0.0) {
    if (!std::isfinite(collision_margin) || collision_margin < 0.0) {
        return std::numeric_limits<double>::infinity();
    }
    const double delta = std::max(
        0.0, frame - static_cast<double>(obstacle.start_frame));
    const double standard_deviation =
        largestCovarianceStandardDeviation(obstacle, frame);
    const double safety_radius =
        obstacle.radius + obstacle.uncertainty_growth_per_frame * delta +
        obstacle.covariance_confidence_scale * standard_deviation +
        collision_margin;
    return std::isfinite(safety_radius)
               ? std::max(0.0, safety_radius)
               : std::numeric_limits<double>::infinity();
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

inline double predictedObstacleCollisionProbability(
    const MovingObstacle& obstacle,
    const autoplanner::Point2d& position,
    double frame,
    double collision_margin = 0.0) {
    autoplanner::Point2d predicted;
    if (!predictMovingObstaclePosition(obstacle, frame, predicted)) return 0.0;
    if (!std::isfinite(collision_margin) || collision_margin < 0.0) return 1.0;

    const double delta = std::max(
        0.0, frame - static_cast<double>(obstacle.start_frame));
    const double nominal_radius =
        obstacle.radius + obstacle.uncertainty_growth_per_frame * delta +
        collision_margin;
    const double clearance =
        distanceToOccupiedBox(position, predicted) - nominal_radius;
    if (clearance <= 0.0) return 1.0;

    const double standard_deviation =
        largestCovarianceStandardDeviation(obstacle, frame);
    if (!std::isfinite(standard_deviation)) return 1.0;
    if (standard_deviation <= 1e-12) return 0.0;
    const double normalized = clearance / standard_deviation;
    return std::clamp(std::exp(-0.5 * normalized * normalized), 0.0, 1.0);
}

inline double predictedObstacleCollisionProbability(
    const std::vector<MovingObstacle>& obstacles,
    const autoplanner::Point2d& position,
    double frame,
    double collision_margin = 0.0) {
    double maximum_probability = 0.0;
    for (const auto& obstacle : obstacles) {
        maximum_probability = std::max(
            maximum_probability,
            predictedObstacleCollisionProbability(
                obstacle, position, frame, collision_margin));
    }
    return maximum_probability;
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
        const double occupied_radius = predictedObstacleSafetyRadius(
            obstacle, frame);
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
        const double expansion = predictedObstacleSafetyRadius(
            obstacle, frame, collision_margin);
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
    autoplanner::Point2d predicted;
    if (!predictMovingObstaclePosition(obstacle,
                                       static_cast<double>(frame),
                                       predicted)) {
        return false;
    }
    cell = {static_cast<int>(std::floor(predicted.x)),
            static_cast<int>(std::floor(predicted.y))};
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
