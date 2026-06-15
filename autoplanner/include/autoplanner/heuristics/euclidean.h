#pragma once

#include <cmath>
#include <string>

#include "autoplanner/heuristics/heuristic.h"

namespace autoplanner {

// Euclidean distance: sqrt(dx^2 + dy^2).
// Admissible for unrestricted movement in continuous space.
class EuclideanHeuristic final : public Heuristic {
public:
    double compute(const Point2i& current, const Point2i& goal) const override {
        const double dx = static_cast<double>(current.x - goal.x);
        const double dy = static_cast<double>(current.y - goal.y);
        return std::sqrt(dx * dx + dy * dy);
    }

    std::string name() const override { return "euclidean"; }
};

}  // namespace autoplanner
