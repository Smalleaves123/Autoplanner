#pragma once

#include <memory>
#include <vector>

#include "autoplanner/core/planner_base.h"
#include "autoplanner/heuristics/heuristic.h"

namespace autoplanner {

// Hybrid A* — kinematic-aware planning with continuous (x, y, theta) state.
// Uses a kinematic bicycle model for state expansion, producing feasible
// paths for car-like robots.
class HybridAStarPlanner final : public PlannerBase {
public:
    HybridAStarPlanner(double turning_radius = 5.0,
                       double step_size = 1.0,
                       int angle_bins = 72,
                       bool allow_reverse = true,
                       double reverse_penalty = 1.2,
                       double collision_check_resolution = 0.25);

    void setHeuristic(std::unique_ptr<Heuristic> heuristic);

    PlannerResult plan(const GridMap& map, const Point2i& start,
                       const Point2i& goal) override;
    std::string name() const override;

private:
    double turning_radius_;
    double step_size_;
    int angle_bins_;
    bool allow_reverse_;
    double reverse_penalty_;
    double collision_check_resolution_;
    std::unique_ptr<Heuristic> heuristic_;
};

}  // namespace autoplanner
