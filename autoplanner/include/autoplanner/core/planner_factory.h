#pragma once

#include <memory>
#include <string>

#include "autoplanner/core/planner_base.h"

namespace autoplanner {

class Costmap2D;

// Common construction parameters for all planners exposed by the CLI and
// Python experiment tools. Unused fields are ignored by a given planner.
struct PlannerFactoryOptions {
    bool allow_diagonal = true;
    double robot_radius = 0.0;

    double heuristic_weight = 1.0;
    double weighted_astar_weight = 1.5;
    double obstacle_weight = 2.0;
    double turning_weight = 0.5;

    double step_size = 2.0;
    int max_iterations = 5000;
    double goal_sample_rate = 0.1;
    double goal_tolerance = 2.0;
    double rewire_radius = 5.0;

    double turning_radius = 5.0;
    int angle_bins = 72;
};

// Create any supported planner using one stable name. Returns nullptr for an
// unknown planner name.
std::unique_ptr<PlannerBase> createPlanner(
    const std::string& planner_name,
    const PlannerFactoryOptions& options = {},
    const Costmap2D* costmap = nullptr);

}  // namespace autoplanner
