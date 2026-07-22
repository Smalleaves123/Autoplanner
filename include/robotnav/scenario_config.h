#pragma once

#include <string>

#include "autoplanner/core/point.h"
#include "robotnav/pipeline_config.h"

namespace robotnav {

struct ScenarioConfig {
    std::string map_path = "autoplanner/data/maps/simple_50x50.txt";
    double map_resolution = 1.0;
    autoplanner::Point2i start{1, 1};
    autoplanner::Point2i goal{48, 48};
    PipelineConfig pipeline;
};

// Load scalar YAML/INI-style values through AutoPlanner's existing config
// loader. Lists and ROS-specific schemas are intentionally out of scope.
bool loadScenarioConfig(const std::string& file_path,
                        ScenarioConfig& scenario);

}  // namespace robotnav
