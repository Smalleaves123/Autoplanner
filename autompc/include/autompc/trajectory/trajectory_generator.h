#pragma once

#include <string>
#include <vector>

#include "autompc/core/trajectory.h"

namespace autompc {

struct Waypoint2d {
    double x = 0.0;
    double y = 0.0;
};

using Waypoints = std::vector<Waypoint2d>;

// Options for converting planner waypoints into a controller-ready
// trajectory. Distances are in metres, speeds in m/s, and accelerations in
// m/s^2.
struct TrajectoryOptions {
    double sample_spacing = 0.5;
    double target_velocity = 1.0;
    double max_velocity = 2.0;
    double max_acceleration = 1.5;
    double max_deceleration = 2.0;
    double max_lateral_acceleration = 1.5;
};

// Convert a polyline into an equally spaced, curvature-aware trajectory.
// The final point is stopped and the speed profile is limited by both
// curvature and longitudinal acceleration/deceleration constraints.
Trajectory generateTrajectory(const Waypoints& waypoints,
                               const TrajectoryOptions& options = {});

// Read an AutoPlanner x,y CSV and generate a controller-ready trajectory.
bool loadPathCsv(const std::string& file_path, double velocity,
                 Trajectory& trajectory,
                 const TrajectoryOptions& options = {});

// Save the generated reference trajectory for inspection and reproducibility.
bool saveTrajectoryCsv(const Trajectory& trajectory,
                       const std::string& file_path);

}  // namespace autompc
