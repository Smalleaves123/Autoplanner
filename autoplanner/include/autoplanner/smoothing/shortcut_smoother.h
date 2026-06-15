#pragma once

#include <memory>

#include "autoplanner/collision/collision_checker.h"
#include "autoplanner/smoothing/path_smoother.h"

namespace autoplanner {

// Shortcut smoothing: repeatedly picks two points on the path and replaces
// the intermediate sub-path with a straight line if it is collision-free.
// Simple, fast, and effective at removing redundant waypoints.
class ShortcutSmoother final : public PathSmoother {
public:
    explicit ShortcutSmoother(const CollisionChecker& checker,
                              int max_iterations = 100);

    std::vector<Point2d> smooth(const std::vector<Point2d>& path) override;

    std::string name() const override;

private:
    const CollisionChecker& checker_;
    int max_iterations_;
};

}  // namespace autoplanner
