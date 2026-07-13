#pragma once

#include <vector>

#include "autoplanner/core/point.h"
#include "autoplanner/core/pose.h"

namespace autoplanner {

// Abstract interface for collision checking.
// Different backends (grid, line-based) can be used by planners and smoothers.
class CollisionChecker {
public:
    virtual ~CollisionChecker() = default;

    // True if the point lies in free space (not inside or too close to obstacle).
    virtual bool isStateValid(const Point2d& p) const = 0;

    // True if the straight-line segment between two points is collision-free.
    virtual bool isSegmentValid(const Point2d& p1, const Point2d& p2) const = 0;

    // True if every segment along a path is collision-free.
    virtual bool isPathValid(const std::vector<Point2d>& path) const = 0;

    // Pose-aware variants. Point-only checkers inherit these defaults, while
    // footprint checkers can account for robot orientation and dimensions.
    virtual bool isPoseValid(const Pose2d& pose) const;
    virtual bool isPoseSegmentValid(const Pose2d& p1,
                                    const Pose2d& p2) const;
    virtual bool isPosePathValid(const std::vector<Pose2d>& path) const;
};

}  // namespace autoplanner
