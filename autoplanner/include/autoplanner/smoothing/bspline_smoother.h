#pragma once

#include <vector>

#include "autoplanner/core/point.h"
#include "autoplanner/smoothing/path_smoother.h"

namespace autoplanner {

// B-spline smoothing: fits a cubic B-spline to the path waypoints.
// Provides C2 continuity and local control — modifying one control point
// only affects a local neighbourhood of the curve.
class BSplineSmoother final : public PathSmoother {
public:
    // samples_per_segment controls the output density.
    explicit BSplineSmoother(int samples_per_segment = 10);

    std::vector<Point2d> smooth(const std::vector<Point2d>& path) override;

    std::string name() const override;

private:
    int samples_per_segment_;
};

}  // namespace autoplanner
