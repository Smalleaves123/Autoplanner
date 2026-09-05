#pragma once

#include <string>
#include <vector>

#include "robotnav/pipeline_config.h"

namespace robotnav {

struct ComponentCatalog {
    std::vector<std::string> planners;
    std::vector<std::string> controllers;
    std::vector<std::string> local_planners;
    std::vector<std::string> smoothers;
};

struct ComponentSelectionResult {
    bool valid = false;
    std::string message;
};

// Discover every component accepted by the standalone RobotNav pipelines.
// Optional stages include the reserved name "none".
ComponentCatalog availableComponents();

// Validate registry-backed component names before expensive map preparation.
// Dynamic mode additionally accepts the top-level Space-Time A* planner.
ComponentSelectionResult validateComponentSelection(
    const PipelineConfig& config,
    bool dynamic_mode = false);

}  // namespace robotnav
