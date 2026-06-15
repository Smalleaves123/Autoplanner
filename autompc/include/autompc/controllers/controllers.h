#pragma once

#include <vector>
#include "autompc/core/types.h"
#include "autompc/core/trajectory.h"

namespace autompc {

// PID controller for trajectory tracking.
//
// Computes velocity and steering commands from cross-track error and heading
// error relative to a reference trajectory point.
class PIDController {
public:
    PIDController(double kp_vel = 1.0, double ki_vel = 0.0, double kd_vel = 0.0,
                  double kp_steer = 2.0, double ki_steer = 0.0, double kd_steer = 0.5,
                  double wheelbase = 1.0);

    Control compute(const State& state, const TrajectoryPoint& ref, double dt);

    void reset();

private:
    double kp_vel_, ki_vel_, kd_vel_;
    double kp_steer_, ki_steer_, kd_steer_;
    double wheelbase_;
    double prev_vel_err_ = 0.0, prev_steer_err_ = 0.0;
    double integral_vel_ = 0.0, integral_steer_ = 0.0;
};

// Pure Pursuit controller.
//
// Selects a lookahead point on the trajectory and computes steering to
// follow a constant-curvature arc to that point.
class PurePursuitController {
public:
    explicit PurePursuitController(double lookahead = 2.0, double wheelbase = 1.0);

    Control compute(const State& state, const Trajectory& traj, double target_vel);

private:
    double lookahead_;
    double wheelbase_;
};

// Stanley controller.
//
// Combines cross-track error correction and heading alignment.
class StanleyController {
public:
    StanleyController(double k = 0.5, double wheelbase = 1.0);

    Control compute(const State& state, const TrajectoryPoint& ref, double target_vel);

private:
    double k_;
    double wheelbase_;
};

}  // namespace autompc
