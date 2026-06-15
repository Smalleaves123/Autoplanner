#pragma once

#include <random>
#include <vector>

#include "autoplanner/core/planner_base.h"

namespace autoplanner {

// Informed RRT* — restricts sampling to an elliptical region once an
// initial path is found, focusing search on improving the path.
class InformedRRTStarPlanner final : public PlannerBase {
public:
    InformedRRTStarPlanner(double step_size = 2.0,
                           int max_iter = 8000,
                           double goal_sample_rate = 0.1,
                           double goal_tolerance = 2.0,
                           double rewire_radius = 5.0);

    PlannerResult plan(const GridMap& map, const Point2i& start,
                       const Point2i& goal) override;
    std::string name() const override;
    const std::vector<std::pair<Point2d, Point2d>>& treeEdges() const;

private:
    double step_size_;
    int max_iter_;
    double goal_sample_rate_;
    double goal_tolerance_;
    double rewire_radius_;
    std::mt19937 rng_;
    std::vector<std::pair<Point2d, Point2d>> tree_edges_;
};

}  // namespace autoplanner
