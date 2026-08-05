#pragma once

#include <string>
#include <string_view>

#include "autoplanner/core/path.h"
#include "robotnav/status.h"

namespace autoplanner {

// Result returned by every PlannerBase::plan() call.
// Contains the path itself, timing, search statistics, and a human-readable
// message (success reason or failure description).
struct PlannerResult {
    bool success = false;
    std::string planner_name;

    // The computed path as a sequence of world coordinates.
    Path2d path;

    // Optional motion direction for each path point: -1 reverse, 1 forward,
    // and 0 for an unspecified endpoint. Kinodynamic planners can preserve
    // gear changes here; other planners leave this empty.
    std::vector<int> motion_directions;

    // Computed Euclidean length of the path.
    double path_length = 0.0;
    double planning_time_ms = 0.0;

    // Number of nodes popped from the open set.
    int expanded_nodes = 0;

    // Number of main-loop iterations (usually equals expanded_nodes).
    int iterations = 0;

    // Quality and safety metrics filled by the CLI after final path checks.
    bool collision_free = false;
    int turning_count = 0;
    double total_turning = 0.0;
    double average_curvature = 0.0;
    double smoothness_score = 0.0;
    double minimum_obstacle_distance = 0.0;

    std::string message;

    // Stable machine-readable status derived from the legacy success/message
    // fields. New callers should branch on this code instead of parsing text.
    robotnav::StatusCode statusCode() const noexcept {
        if (success) return robotnav::StatusCode::Success;
        if (message.find("iteration limit") != std::string::npos ||
            message.find("max iterations") != std::string::npos ||
            message.find("within max") != std::string::npos) {
            return robotnav::StatusCode::Timeout;
        }
        if (message.find("not initialized") != std::string::npos ||
            message.find("outside the map") != std::string::npos) {
            return robotnav::StatusCode::InvalidConfiguration;
        }
        if (message.find("Start") != std::string::npos) {
            return robotnav::StatusCode::InvalidStart;
        }
        if (message.find("Goal") != std::string::npos) {
            return robotnav::StatusCode::InvalidGoal;
        }
        if (message.find("collision") != std::string::npos ||
            message.find("Collision") != std::string::npos) {
            return robotnav::StatusCode::Collision;
        }
        if (!message.empty()) return robotnav::StatusCode::NoPath;
        return robotnav::StatusCode::InternalError;
    }

    std::string_view statusCodeString() const noexcept {
        return robotnav::toString(statusCode());
    }
};

// Write result metadata to a JSON file.
bool saveMetricsJson(const PlannerResult& result, const std::string& file_path);

}  // namespace autoplanner
