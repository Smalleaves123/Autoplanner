#pragma once

#include <string>

#include "autoplanner/core/path.h"

namespace autoplanner {

// Result returned by every PlannerBase::plan() call.
// Contains the path itself, timing, search statistics, and a human-readable
// message (success reason or failure description).
struct PlannerResult {
    bool success = false;
    std::string planner_name;

    // The computed path as a sequence of world coordinates.
    Path2d path;

    // Computed Euclidean length of the path.
    double path_length = 0.0;
    double planning_time_ms = 0.0;

    // Number of nodes popped from the open set.
    int expanded_nodes = 0;

    // Number of main-loop iterations (usually equals expanded_nodes).
    int iterations = 0;

    std::string message;
};

// Write result metadata to a JSON file.
bool saveMetricsJson(const PlannerResult& result, const std::string& file_path);

}  // namespace autoplanner
