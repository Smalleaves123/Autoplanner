#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "autoplanner/core/grid_map.h"
#include "autoplanner/core/planner_result.h"
#include "autoplanner/core/point.h"

namespace robotnav {

struct CooperativeAStarOptions {
    bool allow_diagonal = true;
    bool allow_wait = true;
    std::size_t max_time_steps = 200;
    // When enabled, use Conflict-Based Search instead of prioritized planning.
    bool use_cbs = false;
    // Treat agents as completed once they reach an identical shared goal.
    bool allow_shared_goals = false;
    // Optional input-agent order for prioritized planning. Empty means
    // natural order [0, 1, ...]. CBS intentionally ignores this setting.
    std::vector<std::size_t> priority_order;
    // Bound high-level CBS expansions to keep stress benchmarks predictable.
    std::size_t max_cbs_nodes = 10000;
};

struct MultiAgentPlanResult {
    bool success = false;
    std::vector<autoplanner::PlannerResult> plans;
    std::size_t planned_agents = 0;
    std::size_t vertex_conflicts = 0;
    std::size_t edge_conflicts = 0;
    std::string message;
};

// Prioritized Cooperative A*: agents are planned in input order. Completed
// paths become hard vertex/edge reservations for subsequent agents.
class CooperativeAStarPlanner {
public:
    explicit CooperativeAStarPlanner(CooperativeAStarOptions options = {});

    MultiAgentPlanResult plan(
        const autoplanner::GridMap& map,
        const std::vector<autoplanner::Point2i>& starts,
        const std::vector<autoplanner::Point2i>& goals) const;

private:
    CooperativeAStarOptions options_;
};

}  // namespace robotnav
