#pragma once

#include <memory>

#include "autoplanner/core/planner_base.h"
#include "autoplanner/heuristics/heuristic.h"

namespace autoplanner {

// Jump Point Search — accelerates A* on uniform-cost grids by jumping
// over intermediate nodes that don't affect the optimal path.
//
// JPS identifies "jump points" — nodes where the optimal path may change
// direction — and only expands those, dramatically reducing the number
// of nodes explored compared to standard A*.
class JPSPlanner final : public PlannerBase {
public:
    explicit JPSPlanner(bool allow_diagonal = true);

    void setHeuristic(std::unique_ptr<Heuristic> heuristic);

    PlannerResult plan(
        const GridMap& map,
        const Point2i& start,
        const Point2i& goal
    ) override;

    std::string name() const override;

private:
    bool allow_diagonal_;
    std::unique_ptr<Heuristic> heuristic_;
};

}  // namespace autoplanner
