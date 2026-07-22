#pragma once

#include <cstddef>
#include <string>

#include "autompc/simulation/kinematic_bicycle.h"
#include "autompc/trajectory/trajectory_generator.h"
#include "autoplanner/core/planner_factory.h"
#include "robotnav/safety_supervisor.h"

namespace robotnav {

// Configuration shared by the standalone C++ navigation pipeline and its
// Python/CLI callers. The pipeline intentionally keeps all numerical work in
// the existing AutoPlanner and AutoMPC libraries.
struct PipelineConfig {
    std::string planner = "astar";
    std::string controller = "stanley";

    autoplanner::PlannerFactoryOptions planner_options;
    autompc::TrajectoryOptions trajectory_options;
    autompc::SimulationOptions simulation_options;
    SafetyOptions safety_options;

    std::size_t max_steps = 2000;
};

}  // namespace robotnav
