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
    std::string footprint = "point";
    std::string smoother = "none";

    double robot_radius = 0.0;
    double robot_length = 0.0;
    double robot_width = 0.0;
    bool inflate_map = false;
    int smoothing_iterations = 100;

    autoplanner::PlannerFactoryOptions planner_options;
    autompc::TrajectoryOptions trajectory_options;
    autompc::SimulationOptions simulation_options;
    SafetyOptions safety_options;

    std::size_t max_steps = 2000;
};

}  // namespace robotnav
