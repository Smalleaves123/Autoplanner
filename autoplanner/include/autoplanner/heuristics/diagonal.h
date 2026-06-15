#pragma once

#include <algorithm>
#include <cmath>
#include <string>

#include "autoplanner/heuristics/heuristic.h"

namespace autoplanner {

// Octile (diagonal) distance for 8-connected grids.
// Formula: |dx - dy| + sqrt(2) * min(dx, dy)
// Equivalent to moving diagonally as far as possible, then cardinally.
// Admissible when diagonal moves cost sqrt(2) and cardinal moves cost 1.
class DiagonalHeuristic final : public Heuristic {
public:
    double compute(const Point2i& current, const Point2i& goal) const override {
        int dx = std::abs(current.x - goal.x);
        int dy = std::abs(current.y - goal.y);
        if (dx < dy) {
            std::swap(dx, dy);
        }
        return static_cast<double>(dx - dy) + std::sqrt(2.0) * static_cast<double>(dy);
    }

    std::string name() const override { return "diagonal"; }
};

}  // namespace autoplanner
