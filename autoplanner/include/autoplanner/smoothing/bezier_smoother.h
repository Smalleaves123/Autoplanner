#pragma once

#include <vector>

#include "autoplanner/core/point.h"
#include "autoplanner/smoothing/path_smoother.h"

namespace autoplanner {

// Bezier curve smoothing: fits a cubic Bezier curve through the path
// waypoints and samples it at a given resolution.
// Produces visually smooth paths, but does not guarantee collision-free output
// unless combined with a collision checker.
class BezierSmoother final : public PathSmoother {
public:
    // samples_per_segment controls the output density.
    explicit BezierSmoother(int samples_per_segment = 10);

    std::vector<Point2d> smooth(const std::vector<Point2d>& path) override;

    std::string name() const override;

private:
    int samples_per_segment_;
};

}  // namespace autoplanner
