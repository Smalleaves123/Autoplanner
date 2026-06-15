#pragma once

#include <vector>

#include "autoplanner/core/grid_map.h"

namespace autoplanner {

// A 2-D costmap that assigns a scalar cost [0, 1] to every grid cell.
//
//   0.0  →  definitely free
//   1.0  →  definitely obstructed (lethal)
//
// Intermediate values represent proximity to obstacles and can be used by
// planners to prefer routes that stay clear of walls.
class Costmap2D {
public:
    Costmap2D() = default;

    // Initialize cost data from a binary occupancy grid.
    // Obstacle cells get cost 1.0; free cells get 0.0.
    void buildFromGridMap(const GridMap& map);

    // Inflate obstacles by the given robot radius (in metres).
    // Cells within the inflation radius get an exponentially decaying cost.
    void inflateObstacles(double robot_radius);

    // Return the cost at cell (x, y).  Out-of-bounds returns 1.0.
    double getCost(int x, int y) const;

    // True if the cell is traversable (cost < 1.0).
    bool isFree(int x, int y) const;

    int width() const;
    int height() const;

    // Underlying grid resolution (inherited from the source map).
    double resolution() const;

private:
    int width_ = 0;
    int height_ = 0;
    double resolution_ = 1.0;
    std::vector<double> cost_;
};

}  // namespace autoplanner
