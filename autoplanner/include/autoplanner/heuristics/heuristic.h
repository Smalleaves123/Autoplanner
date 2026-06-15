#pragma once

#include <string>

#include "autoplanner/core/point.h"

namespace autoplanner {

// Abstract base for heuristic functions used in graph search planners.
// A heuristic estimates the remaining cost from a given node to the goal,
// guiding the search towards the target efficiently.
class Heuristic {
public:
    virtual ~Heuristic() = default;

    // Returns the estimated cost from current to goal.
    // Must be admissible (never overestimate) for A* to guarantee optimality.
    virtual double compute(const Point2i& current, const Point2i& goal) const = 0;

    // Human-readable name, e.g. "manhattan", "euclidean".
    virtual std::string name() const = 0;
};

}  // namespace autoplanner
