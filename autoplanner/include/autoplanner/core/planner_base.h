#pragma once

#include <string>

#include "autoplanner/core/grid_map.h"
#include "autoplanner/core/planner_result.h"
#include "autoplanner/core/point.h"

namespace autoplanner {

// Unified interface that every path planner must implement.
// Makes it easy to benchmark different algorithms and swap planners at
// runtime via a factory function.
class PlannerBase {
public:
    virtual ~PlannerBase() = default;

    // Run the planner on the given map and return a result.
    virtual PlannerResult plan(
        const GridMap& map,
        const Point2i& start,
        const Point2i& goal
    ) = 0;

    // Short human-readable name (e.g. "astar", "dijkstra").
    virtual std::string name() const = 0;
};

}  // namespace autoplanner
