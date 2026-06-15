#pragma once

#include <random>
#include <vector>

#include "autoplanner/core/planner_base.h"

namespace autoplanner {

// Bi-RRT — grows two trees simultaneously from start and goal.
class BiRRTPlanner final : public PlannerBase {
public:
    BiRRTPlanner(double step_size = 2.0, int max_iter = 5000,
                 double goal_tolerance = 2.0);

    PlannerResult plan(const GridMap& map, const Point2i& start,
                       const Point2i& goal) override;
    std::string name() const override;

private:
    double step_size_;
    int max_iter_;
    double goal_tolerance_;
    std::mt19937 rng_;
};

}  // namespace autoplanner
