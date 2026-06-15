#pragma once

#include <cstdlib>
#include <string>

#include "autoplanner/heuristics/heuristic.h"

namespace autoplanner {

// Manhattan distance: |dx| + |dy|.
// Admissible for 4-connected grids (cardinal moves only).
class ManhattanHeuristic final : public Heuristic {
public:
    double compute(const Point2i& current, const Point2i& goal) const override {
        return std::abs(current.x - goal.x) + std::abs(current.y - goal.y);
    }

    std::string name() const override { return "manhattan"; }
};

}  // namespace autoplanner
