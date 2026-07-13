#pragma once

#include <string>
#include <vector>

#include "autompc/core/types.h"

namespace autompc {

using Trajectory = std::vector<TrajectoryPoint>;

// Distance to the closest point on the trajectory.
double closestPointDistance(const Trajectory& traj, const State& state);

// Interpolate the trajectory at a given arc-length position.
TrajectoryPoint interpolate(const Trajectory& traj, double s);

// Total arc length of the trajectory.
double arcLength(const Trajectory& traj);

// Generate a circular trajectory.
Trajectory makeCircle(double radius, double velocity, int n);

// Generate a straight-line trajectory.
Trajectory makeStraightLine(double x0, double y0, double x1, double y1,
                             double velocity, int n);

// Load an x,y waypoint CSV (as produced by AutoPlanner) and derive heading
// angles for each waypoint.
bool loadPathCsv(const std::string& file_path, double velocity,
                 Trajectory& trajectory);

}  // namespace autompc
