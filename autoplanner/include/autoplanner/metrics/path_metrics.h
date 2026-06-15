#pragma once

#include <vector>

#include "autoplanner/core/point.h"

namespace autoplanner {

// Compute various quality metrics for a path.
// All metrics are in continuous (world) coordinates.

// Number of direction changes along the path.
int computeTurningCount(const std::vector<Point2d>& path);

// Sum of absolute turning angles (radians) along the path.
double computeTotalTurning(const std::vector<Point2d>& path);

// Average curvature computed from three consecutive points.
// Lower values indicate a smoother path.
double computeAverageCurvature(const std::vector<Point2d>& path);

// Smoothness score: ratio of total straight-line distance to actual path length.
// 1.0 = perfectly straight; lower values = more winding.
double computeSmoothnessScore(const std::vector<Point2d>& path);

// Minimum distance to the nearest obstacle for every point on the path.
// obstacles is a vector of (x, y) obstacle positions (grid cells).
double computeMinObstacleDistance(
    const std::vector<Point2d>& path,
    const std::vector<Point2i>& obstacles
);

}  // namespace autoplanner
