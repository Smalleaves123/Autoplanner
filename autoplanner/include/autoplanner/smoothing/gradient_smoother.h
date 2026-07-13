#pragma once

#include <vector>

#include "autoplanner/core/grid_map.h"
#include "autoplanner/core/point.h"
#include "autoplanner/collision/collision_checker.h"
#include "autoplanner/smoothing/path_smoother.h"

namespace autoplanner {

// Gradient-based path smoother.
//
// Iteratively adjusts waypoints to minimise a weighted sum of:
//   - obstacle cost   (pushes points away from obstacles)
//   - smoothness cost (penalises sharp turns / curvature)
//   - length cost     (penalises unnecessary deviation)
//
// Uses the distance transform of the map to compute obstacle gradients
// efficiently.  The result retains the start and goal endpoints unchanged.
class GradientSmoother final : public PathSmoother {
public:
    // Construct a gradient smoother.
    //   map             — occupancy grid used for distance transform
    //   checker         — collision checker (validates intermediate states)
    //   alpha           — obstacle cost weight      (default 1.0)
    //   beta            — smoothness cost weight     (default 2.0)
    //   gamma           — path-length cost weight    (default 0.1)
    //   max_iterations  — gradient descent iterations
    //   learning_rate   — step size per iteration
    GradientSmoother(const GridMap& map,
                     const CollisionChecker& checker,
                     double alpha = 1.0,
                     double beta = 2.0,
                     double gamma = 0.1,
                     int max_iterations = 200,
                     double learning_rate = 0.5);

    std::vector<Point2d> smooth(const std::vector<Point2d>& path) override;

    std::string name() const override;

private:
    const GridMap& map_;
    const CollisionChecker& checker_;
    double alpha_;
    double beta_;
    double gamma_;
    int max_iterations_;
    double learning_rate_;

    // Cached distance transform values for the map.
    std::vector<double> dt_;
    int dt_width_ = 0;
    int dt_height_ = 0;

    // Look up distance-to-nearest-obstacle at a continuous point.
    double distanceToObstacle(const Point2d& p) const;

    // Finite-difference gradient of the distance field at a point.
    Point2d distanceGradient(const Point2d& p, double eps = 1.0) const;
};

}  // namespace autoplanner
