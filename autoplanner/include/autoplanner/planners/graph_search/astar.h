#pragma once

#include <memory>

#include "autoplanner/core/planner_base.h"
#include "autoplanner/heuristics/heuristic.h"

namespace autoplanner {

// A* grid planner — combines Dijkstra's actual-cost (g) with a heuristic
// estimate (h) to focus search towards the goal.
//
// Setting a different Heuristic changes the search behaviour:
//   - Euclidean  — shortest geometric path, works for any grid
//   - Manhattan  — admissible only for 4-connected movement
//   - Diagonal   — tightest admissible bound for 8-connected movement
class AStarPlanner final : public PlannerBase {
public:
    // Constructs an A* planner with optional diagonal movement.
    // Defaults to the Euclidean heuristic — call setHeuristic() to change.
    explicit AStarPlanner(bool allow_diagonal = true);

    // Replace the heuristic (default is Euclidean).
    void setHeuristic(std::unique_ptr<Heuristic> heuristic);

    PlannerResult plan(
        const GridMap& map,
        const Point2i& start,
        const Point2i& goal
    ) override;

    std::string name() const override;

private:
    bool allow_diagonal_ = true;
    std::unique_ptr<Heuristic> heuristic_;
};

}  // namespace autoplanner
