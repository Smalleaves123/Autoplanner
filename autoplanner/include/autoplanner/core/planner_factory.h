#pragma once

#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

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
    bool allow_reverse = true;
    double reverse_penalty = 1.2;
    double collision_check_resolution = 0.25;
};

// Runtime registry used by the built-in factory and by applications that
// provide their own planners. Registration is process-local and thread-safe;
// factories are invoked without holding the registry lock.
class PlannerRegistry {
public:
    using Factory = std::function<std::unique_ptr<PlannerBase>(
        const PlannerFactoryOptions&, const Costmap2D*)>;

    static PlannerRegistry& instance();

    // Register a planner under a stable name. Returns false for an empty name,
    // an empty factory, or an existing name unless replace is true.
    bool registerPlanner(const std::string& name,
                         Factory factory,
                         bool replace = false);
    bool unregisterPlanner(const std::string& name);
    bool contains(const std::string& name) const;
    std::vector<std::string> availablePlanners() const;

    std::unique_ptr<PlannerBase> create(
        const std::string& name,
        const PlannerFactoryOptions& options = {},
        const Costmap2D* costmap = nullptr) const;

private:
    PlannerRegistry();

    mutable std::mutex mutex_;
    std::map<std::string, Factory> factories_;
};

// Create any supported planner using one stable name. Returns nullptr for an
// unknown planner name.
std::unique_ptr<PlannerBase> createPlanner(
    const std::string& planner_name,
    const PlannerFactoryOptions& options = {},
    const Costmap2D* costmap = nullptr);

// Return all planner names currently registered in deterministic order.
std::vector<std::string> availablePlanners();

}  // namespace autoplanner
