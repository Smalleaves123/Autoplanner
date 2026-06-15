#pragma once

#include <memory>

#include "autoplanner/core/planner_base.h"
#include "autoplanner/heuristics/heuristic.h"

namespace autoplanner {

// D* Lite — incremental replanning for dynamic environments.
class DStarLitePlanner final : public PlannerBase {
public:
    explicit DStarLitePlanner(bool allow_diagonal = true);
    void setHeuristic(std::unique_ptr<Heuristic> heuristic);

    PlannerResult plan(const GridMap& map, const Point2i& start,
                       const Point2i& goal) override;
    std::string name() const override;

private:
    bool allow_diagonal_;
    std::unique_ptr<Heuristic> heuristic_;
};

}  // namespace autoplanner
