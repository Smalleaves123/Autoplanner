#pragma once

#include "autoplanner/collision/collision_checker.h"
#include "autoplanner/core/grid_map.h"

namespace autoplanner {

// Line-based collision checker using Bresenham ray casting.
// A segment is valid iff every cell along the rasterized line is free.
class LineCollisionChecker final : public CollisionChecker {
public:
    explicit LineCollisionChecker(const GridMap& map);

    bool isStateValid(const Point2d& p) const override;
    bool isSegmentValid(const Point2d& p1, const Point2d& p2) const override;
    bool isPathValid(const std::vector<Point2d>& path) const override;

    // Set the resolution for sub-pixel checks along a segment.
    // Smaller values give finer checking but are slower.
    void setCheckResolution(double resolution);

private:
    const GridMap& map_;
    double check_resolution_ = 0.5;
};

}  // namespace autoplanner
