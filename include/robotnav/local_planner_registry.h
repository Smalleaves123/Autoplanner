#pragma once

#include <cstddef>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "autompc/core/trajectory.h"
#include "autompc/core/types.h"
#include "autompc/simulation/kinematic_bicycle.h"
#include "autoplanner/collision/collision_checker.h"
#include "robotnav/dwa_local_planner.h"
#include "robotnav/dynamic_obstacle_context.h"
#include "robotnav/mppi_local_planner.h"

namespace robotnav {

struct LocalPlannerDecision {
    bool feasible = false;
    autompc::Control command;
    double score = 0.0;
    std::size_t feasible_rollouts = 0;
    std::size_t dynamic_collision_rejections = 0;
    double minimum_dynamic_clearance = 0.0;
    bool warm_started = false;
    double effective_sample_size = 0.0;
    double effective_sample_ratio = 0.0;
    double sampling_noise_scale = 1.0;
    double maximum_collision_probability = 0.0;
    std::size_t workspace_allocation_count = 0;
    bool workspace_reused = false;
};

class LocalPlanner {
public:
    virtual ~LocalPlanner() = default;

    virtual void onTrajectoryChanged() {}

    virtual LocalPlannerDecision computeCommand(
        const autompc::State& state,
        double current_steering,
        const autompc::Trajectory& trajectory,
        const autompc::Control& nominal_command,
        const DynamicObstacleContext& dynamic_context = {}) const = 0;
};

class LocalPlannerRegistry {
public:
    using Factory = std::function<std::unique_ptr<LocalPlanner>(
        const autoplanner::CollisionChecker&,
        const autompc::SimulationOptions&,
        const DwaOptions&,
        const MppiOptions&)>;

    static LocalPlannerRegistry& instance();

    bool registerLocalPlanner(const std::string& name,
                              Factory factory,
                              bool replace = false);
    bool unregisterLocalPlanner(const std::string& name);
    bool contains(const std::string& name) const;
    std::vector<std::string> availableLocalPlanners() const;

    std::unique_ptr<LocalPlanner> create(
        const std::string& name,
        const autoplanner::CollisionChecker& collision_checker,
        const autompc::SimulationOptions& simulation_options,
        const DwaOptions& dwa_options = {},
        const MppiOptions& mppi_options = {}) const;

private:
    LocalPlannerRegistry();

    mutable std::mutex mutex_;
    std::map<std::string, Factory> factories_;
};

std::unique_ptr<LocalPlanner> createLocalPlanner(
    const std::string& name,
    const autoplanner::CollisionChecker& collision_checker,
    const autompc::SimulationOptions& simulation_options,
    const DwaOptions& dwa_options = {},
    const MppiOptions& mppi_options = {});
std::vector<std::string> availableLocalPlanners();

}  // namespace robotnav
