#pragma once

#include <memory>

#include "autoplanner/core/planner_base.h"
#include "autoplanner/costmap/costmap_2d.h"
#include "autoplanner/heuristics/heuristic.h"

namespace autoplanner {

// Improved A* — adds obstacle cost and turning penalty to the standard A* cost.
//
//   f(n) = g(n) + w_h * h(n) + w_obs * obstacle_cost(n) + w_turn * turning_cost(n)
//
// The obstacle cost encourages the planner to stay away from walls.
// The turning cost penalizes direction changes, producing smoother paths.
class ImprovedAStarPlanner final : public PlannerBase {
public:
    ImprovedAStarPlanner(double heuristic_weight = 1.0,
                         double obstacle_weight = 2.0,
                         double turning_weight = 0.5,
                         bool allow_diagonal = true);

    void setHeuristic(std::unique_ptr<Heuristic> heuristic);
    void setCostmap(const Costmap2D* costmap);

    PlannerResult plan(
        const GridMap& map,
        const Point2i& start,
        const Point2i& goal
    ) override;

    std::string name() const override;

private:
    double heuristic_weight_;
    double obstacle_weight_;
    double turning_weight_;
    bool allow_diagonal_;
    std::unique_ptr<Heuristic> heuristic_;
    const Costmap2D* costmap_ = nullptr;
};

}  // namespace autoplanner
