#pragma once

#include <vector>

#include "autoplanner/collision/collision_checker.h"
#include "autoplanner/core/point.h"
#include "autoplanner/smoothing/path_smoother.h"

namespace autoplanner {

class CurvatureConstrainedSmoother final : public PathSmoother {
public:
    CurvatureConstrainedSmoother(const CollisionChecker& checker,
                                 double max_curvature = 0.5,
                                 int max_iterations = 100,
                                 double correction_gain = 0.35);

    std::vector<Point2d> smooth(const std::vector<Point2d>& path) override;

    std::string name() const override;

private:
    const CollisionChecker& checker_;
    double max_curvature_;
    int max_iterations_;
    double correction_gain_;
};

double discreteCurvature(const Point2d& previous,
                         const Point2d& current,
                         const Point2d& next);

}  // namespace autoplanner
