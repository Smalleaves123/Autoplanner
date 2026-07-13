#pragma once

#include <memory>

#include "autoplanner/core/planner_base.h"
#include "autoplanner/heuristics/heuristic.h"

namespace autoplanner {

// D* Lite — incremental replanning for dynamic environments.
class DStarLitePlanner final : public PlannerBase {
public:
    explicit DStarLitePlanner(bool allow_diagonal = true);
    ~DStarLitePlanner() override;

    void setHeuristic(std::unique_ptr<Heuristic> heuristic);

    PlannerResult plan(const GridMap& map, const Point2i& start,
                       const Point2i& goal) override;

    // Reuse the previous search state after the occupancy map changes.
    // The new map is compared against the previous one and only affected
    // vertices are updated before the shortest-path computation continues.
    PlannerResult replan(const GridMap& map, const Point2i& start);

    std::string name() const override;

private:
    struct SearchState;

    bool allow_diagonal_;
    std::unique_ptr<Heuristic> heuristic_;
    std::unique_ptr<SearchState> state_;
};

}  // namespace autoplanner
