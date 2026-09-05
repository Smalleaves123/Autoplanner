#pragma once

#include <cstddef>
#include <mutex>
#include <vector>

#include "autompc/core/trajectory.h"
#include "autompc/core/types.h"
#include "autompc/simulation/kinematic_bicycle.h"
#include "autoplanner/collision/collision_checker.h"
#include "robotnav/dynamic_obstacle_context.h"

namespace robotnav {

struct MppiOptions {
    double prediction_time = 1.0;
    int horizon = 20;
    int rollouts = 64;
    double temperature = 0.5;
    double velocity_noise = 0.35;
    double steering_noise = 0.12;
    double trajectory_weight = 1.0;
    double heading_weight = 0.25;
    double velocity_weight = 0.15;
    double control_weight = 0.2;
    double control_rate_weight = 0.1;
    double dynamic_obstacle_weight = 1.0;
    double dynamic_clearance = 0.5;
    double dynamic_obstacle_margin = 0.0;
    int dynamic_collision_samples = 3;
    unsigned int random_seed = 42;
    bool warm_start = true;
    double warm_start_blend = 0.25;
};

struct MppiDecision {
    bool feasible = false;
    autompc::Control command;
    double score = 0.0;
    std::size_t feasible_rollouts = 0;
    std::size_t dynamic_collision_rejections = 0;
    double minimum_dynamic_clearance = 0.0;
    bool warm_started = false;
};

class MppiLocalPlanner {
public:
    MppiLocalPlanner(const autoplanner::CollisionChecker& collision_checker,
                     autompc::SimulationOptions simulation_options,
                     MppiOptions options = {});

    MppiDecision computeCommand(
        const autompc::State& state,
        double current_steering,
        const autompc::Trajectory& trajectory,
        const autompc::Control& nominal_command,
        const DynamicObstacleContext& dynamic_context = {}) const;

    void resetWarmStart();

private:
    const autoplanner::CollisionChecker& collision_checker_;
    autompc::SimulationOptions simulation_options_;
    MppiOptions options_;
    mutable std::mutex warm_start_mutex_;
    mutable std::vector<autompc::Control> previous_optimal_controls_;
};

}  // namespace robotnav
