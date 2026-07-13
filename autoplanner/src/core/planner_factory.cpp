#include "autoplanner/core/planner_factory.h"

#include "autoplanner/costmap/costmap_2d.h"
#include "autoplanner/planners/graph_search/astar.h"
#include "autoplanner/planners/graph_search/dijkstra.h"
#include "autoplanner/planners/graph_search/dstar_lite.h"
#include "autoplanner/planners/graph_search/improved_astar.h"
#include "autoplanner/planners/graph_search/jps.h"
#include "autoplanner/planners/graph_search/weighted_astar.h"
#include "autoplanner/planners/kinodynamic/hybrid_astar.h"
#include "autoplanner/planners/sampling/bi_rrt.h"
#include "autoplanner/planners/sampling/informed_rrt_star.h"
#include "autoplanner/planners/sampling/rrt.h"
#include "autoplanner/planners/sampling/rrt_star.h"

namespace autoplanner {

std::unique_ptr<PlannerBase> createPlanner(
    const std::string& planner_name,
    const PlannerFactoryOptions& options,
    const Costmap2D* costmap) {
    if (planner_name == "astar") {
        return std::make_unique<AStarPlanner>(options.allow_diagonal);
    }
    if (planner_name == "dijkstra") {
        return std::make_unique<DijkstraPlanner>(options.allow_diagonal);
    }
    if (planner_name == "weighted_astar") {
        return std::make_unique<WeightedAStarPlanner>(
            options.weighted_astar_weight, options.allow_diagonal);
    }
    if (planner_name == "improved_astar") {
        auto planner = std::make_unique<ImprovedAStarPlanner>(
            options.heuristic_weight,
            options.obstacle_weight,
            options.turning_weight,
            options.allow_diagonal);
        planner->setCostmap(costmap);
        return planner;
    }
    if (planner_name == "jps") {
        return std::make_unique<JPSPlanner>(options.allow_diagonal);
    }
    if (planner_name == "dstar_lite") {
        return std::make_unique<DStarLitePlanner>(options.allow_diagonal);
    }
    if (planner_name == "rrt") {
        return std::make_unique<RRTPlanner>(
            options.step_size,
            options.max_iterations,
            options.goal_sample_rate,
            options.goal_tolerance);
    }
    if (planner_name == "rrt_star") {
        return std::make_unique<RRTStarPlanner>(
            options.step_size,
            options.max_iterations,
            options.goal_sample_rate,
            options.goal_tolerance,
            options.rewire_radius);
    }
    if (planner_name == "informed_rrt_star") {
        return std::make_unique<InformedRRTStarPlanner>(
            options.step_size,
            options.max_iterations,
            options.goal_sample_rate,
            options.goal_tolerance,
            options.rewire_radius);
    }
    if (planner_name == "bi_rrt") {
        return std::make_unique<BiRRTPlanner>(
            options.step_size,
            options.max_iterations,
            options.goal_tolerance);
    }
    if (planner_name == "hybrid_astar") {
        return std::make_unique<HybridAStarPlanner>(
            options.turning_radius,
            options.step_size,
            options.angle_bins);
    }

    return nullptr;
}

}  // namespace autoplanner
