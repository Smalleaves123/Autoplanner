#pragma once

#include <vector>

#include "autoplanner/core/point.h"

namespace autoplanner {

// Abstract interface for path smoothing algorithms.
// Each smoother takes a raw path (typically from a planner) and returns
// a shorter and/or smoother version that remains collision-free.
class PathSmoother {
public:
    virtual ~PathSmoother() = default;

    // Smooth the input path. The smoothed result must still be collision-free.
    virtual std::vector<Point2d> smooth(const std::vector<Point2d>& path) = 0;

    virtual std::string name() const = 0;
};

}  // namespace autoplanner
