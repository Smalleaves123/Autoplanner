#pragma once

#include <memory>

#include "autoplanner/core/planner_base.h"
#include "autoplanner/heuristics/heuristic.h"

namespace autoplanner {

// Weighted A* — trades optimality for speed by multiplying the heuristic.
//
// f(n) = g(n) + w * h(n)
//
//   w = 1.0  →  standard A* (optimal)
//   w > 1.0  →  greedier search, fewer expanded nodes, path may be longer
//
// Useful as a speed baseline and for ablation studies against A*.
class WeightedAStarPlanner final : public PlannerBase {
public:
    // weight: heuristic multiplier (default 1.5).
    // allow_diagonal: enable 8-connected movement.
    explicit WeightedAStarPlanner(double weight = 1.5, bool allow_diagonal = true);

    // Replace the heuristic (default is Euclidean).
    void setHeuristic(std::unique_ptr<Heuristic> heuristic);

    // Override the heuristic weight after construction.
    void setWeight(double weight);

    PlannerResult plan(
        const GridMap& map,
        const Point2i& start,
        const Point2i& goal
    ) override;

    std::string name() const override;

private:
    double weight_ = 1.5;
    bool allow_diagonal_ = true;
    std::unique_ptr<Heuristic> heuristic_;
};

}  // namespace autoplanner
