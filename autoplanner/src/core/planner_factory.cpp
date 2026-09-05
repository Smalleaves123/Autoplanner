#include "autoplanner/core/planner_factory.h"

#include <utility>

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

PlannerRegistry::PlannerRegistry() {
    factories_.emplace("astar", [](const PlannerFactoryOptions& options,
                                    const Costmap2D*) {
        return std::make_unique<AStarPlanner>(options.allow_diagonal);
    });
    factories_.emplace("dijkstra", [](const PlannerFactoryOptions& options,
                                       const Costmap2D*) {
        return std::make_unique<DijkstraPlanner>(options.allow_diagonal);
    });
    factories_.emplace("weighted_astar", [](const PlannerFactoryOptions& options,
                                             const Costmap2D*) {
        return std::make_unique<WeightedAStarPlanner>(
            options.weighted_astar_weight, options.allow_diagonal);
    });
    factories_.emplace("improved_astar", [](const PlannerFactoryOptions& options,
                                             const Costmap2D* costmap) {
        auto planner = std::make_unique<ImprovedAStarPlanner>(
            options.heuristic_weight,
            options.obstacle_weight,
            options.turning_weight,
            options.allow_diagonal);
        planner->setCostmap(costmap);
        return planner;
    });
    factories_.emplace("jps", [](const PlannerFactoryOptions& options,
                                  const Costmap2D*) {
        return std::make_unique<JPSPlanner>(options.allow_diagonal);
    });
    factories_.emplace("dstar_lite", [](const PlannerFactoryOptions& options,
                                         const Costmap2D*) {
        return std::make_unique<DStarLitePlanner>(options.allow_diagonal);
    });
    factories_.emplace("rrt", [](const PlannerFactoryOptions& options,
                                  const Costmap2D*) {
        return std::make_unique<RRTPlanner>(
            options.step_size,
            options.max_iterations,
            options.goal_sample_rate,
            options.goal_tolerance);
    });
    factories_.emplace("rrt_star", [](const PlannerFactoryOptions& options,
                                       const Costmap2D*) {
        return std::make_unique<RRTStarPlanner>(
            options.step_size,
            options.max_iterations,
            options.goal_sample_rate,
            options.goal_tolerance,
            options.rewire_radius);
    });
    factories_.emplace("informed_rrt_star",
                       [](const PlannerFactoryOptions& options,
                          const Costmap2D*) {
        return std::make_unique<InformedRRTStarPlanner>(
            options.step_size,
            options.max_iterations,
            options.goal_sample_rate,
            options.goal_tolerance,
            options.rewire_radius);
    });
    factories_.emplace("bi_rrt", [](const PlannerFactoryOptions& options,
                                     const Costmap2D*) {
        return std::make_unique<BiRRTPlanner>(
            options.step_size,
            options.max_iterations,
            options.goal_tolerance);
    });
    factories_.emplace("hybrid_astar", [](const PlannerFactoryOptions& options,
                                           const Costmap2D*) {
        return std::make_unique<HybridAStarPlanner>(
            options.turning_radius,
            options.step_size,
            options.angle_bins,
            options.allow_reverse,
            options.reverse_penalty,
            options.collision_check_resolution);
    });
}

PlannerRegistry& PlannerRegistry::instance() {
    static PlannerRegistry registry;
    return registry;
}

bool PlannerRegistry::registerPlanner(const std::string& name,
                                      Factory factory,
                                      bool replace) {
    if (name.empty() || !factory) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    const auto existing = factories_.find(name);
    if (existing != factories_.end() && !replace) {
        return false;
    }
    factories_[name] = std::move(factory);
    return true;
}

bool PlannerRegistry::unregisterPlanner(const std::string& name) {
    std::lock_guard<std::mutex> lock(mutex_);
    return factories_.erase(name) != 0;
}

bool PlannerRegistry::contains(const std::string& name) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return factories_.find(name) != factories_.end();
}

std::vector<std::string> PlannerRegistry::availablePlanners() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::string> names;
    names.reserve(factories_.size());
    for (const auto& entry : factories_) {
        names.push_back(entry.first);
    }
    return names;
}

std::unique_ptr<PlannerBase> PlannerRegistry::create(
    const std::string& name,
    const PlannerFactoryOptions& options,
    const Costmap2D* costmap) const {
    Factory factory;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto entry = factories_.find(name);
        if (entry == factories_.end()) {
            return nullptr;
        }
        factory = entry->second;
    }
    return factory(options, costmap);
}

std::unique_ptr<PlannerBase> createPlanner(
    const std::string& planner_name,
    const PlannerFactoryOptions& options,
    const Costmap2D* costmap) {
    return PlannerRegistry::instance().create(planner_name, options, costmap);
}

std::vector<std::string> availablePlanners() {
    return PlannerRegistry::instance().availablePlanners();
}

}  // namespace autoplanner
