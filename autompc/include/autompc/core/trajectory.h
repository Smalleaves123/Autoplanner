#pragma once

#include <string>
#include <vector>

#include "autompc/core/types.h"

namespace autompc {

using Trajectory = std::vector<TrajectoryPoint>;

// Geometry and feasibility summary for a controller-ready trajectory.
struct TrajectoryQuality {
    bool finite = true;
    bool curvature_feasible = true;
    double max_abs_curvature = 0.0;
    double minimum_turning_radius = 0.0;
};

// Assess continuous geometry against an optional curvature limit. A zero
// limit disables the feasibility check while still reporting geometry.
TrajectoryQuality assessTrajectory(const Trajectory& trajectory,
                                   double max_curvature = 0.0);

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

}  // namespace autompc
