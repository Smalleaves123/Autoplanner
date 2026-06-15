#pragma once

#include "autoplanner/collision/collision_checker.h"
#include "autoplanner/core/grid_map.h"

namespace autoplanner {

// Grid-based collision checker that queries a GridMap directly.
// A state is valid iff the cell it falls in is free.
class GridCollisionChecker final : public CollisionChecker {
public:
    explicit GridCollisionChecker(const GridMap& map);

    bool isStateValid(const Point2d& p) const override;
    bool isSegmentValid(const Point2d& p1, const Point2d& p2) const override;
    bool isPathValid(const std::vector<Point2d>& path) const override;

private:
    const GridMap& map_;
};

}  // namespace autoplanner
