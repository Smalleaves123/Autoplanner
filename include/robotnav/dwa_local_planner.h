#pragma once

#include <cstddef>
#include <vector>

#include "autompc/core/trajectory.h"
#include "autompc/core/types.h"
#include "autompc/simulation/kinematic_bicycle.h"
#include "autoplanner/collision/collision_checker.h"
#include "robotnav/dynamic_obstacle_context.h"

namespace robotnav {

struct DwaOptions {
    double prediction_time = 1.0;
    int velocity_samples = 5;
    int steering_samples = 7;
    double trajectory_weight = 1.0;
    double heading_weight = 0.25;
    double speed_weight = 0.15;
    double command_weight = 0.35;
    double dynamic_obstacle_margin = 0.0;
    int dynamic_collision_samples = 3;
};

using DwaDynamicContext = DynamicObstacleContext;

struct DwaDecision {
    bool feasible = false;
    autompc::Control command;
    double score = 0.0;
    std::size_t dynamic_collision_rejections = 0;
    double minimum_dynamic_clearance = 0.0;
};

class DwaLocalPlanner {
public:
    DwaLocalPlanner(const autoplanner::CollisionChecker& collision_checker,
                    autompc::SimulationOptions simulation_options,
                    DwaOptions options = {});

    DwaDecision computeCommand(const autompc::State& state,
                               double current_steering,
                               const autompc::Trajectory& trajectory,
                               const autompc::Control& nominal_command,
                               const DwaDynamicContext& dynamic_context = {}) const;

private:
    const autoplanner::CollisionChecker& collision_checker_;
    autompc::SimulationOptions simulation_options_;
    DwaOptions options_;
};

}  // namespace robotnav
