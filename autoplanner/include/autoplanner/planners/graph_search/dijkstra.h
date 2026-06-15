#pragma once

#include "autoplanner/core/planner_base.h"

namespace autoplanner {

// Dijkstra's algorithm — shortest path on a uniform-cost or weighted grid.
// Unlike A*, Dijkstra does not use a goal-directed heuristic, so it expands
// nodes in all directions equally.  This makes it a useful correctness
// baseline: on a 4-connected uniform grid it produces the true shortest path.
class DijkstraPlanner final : public PlannerBase {
public:
    // If allow_diagonal is true, 8-neighbour movement is used with diagonal
    // cost sqrt(2); otherwise 4-neighbour movement with cost 1.
    explicit DijkstraPlanner(bool allow_diagonal = true);

    PlannerResult plan(
        const GridMap& map,
        const Point2i& start,
        const Point2i& goal
    ) override;

    std::string name() const override;

private:
    bool allow_diagonal_ = true;
};

}  // namespace autoplanner
