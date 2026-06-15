#pragma once

#include <memory>
#include <random>
#include <vector>

#include "autoplanner/collision/collision_checker.h"
#include "autoplanner/core/planner_base.h"

namespace autoplanner {

// RRT* — asymptotically optimal variant of RRT.
//
// On top of RRT's expansion step, RRT*:
//   1. Searches a neighbourhood of the new node for a better parent
//   2. Rewires nearby nodes if a cheaper path is found through the new node
//
// Path cost decreases as more iterations are run.
class RRTStarPlanner final : public PlannerBase {
public:
    RRTStarPlanner(double step_size = 2.0,
                   int max_iter = 8000,
                   double goal_sample_rate = 0.1,
                   double goal_tolerance = 2.0,
                   double rewire_radius = 5.0);

    PlannerResult plan(
        const GridMap& map,
        const Point2i& start,
        const Point2i& goal
    ) override;

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
