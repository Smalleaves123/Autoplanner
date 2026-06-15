#pragma once

#include <Eigen/Dense>
#include "autompc/core/types.h"

namespace autompc {

// Linear Quadratic Regulator for trajectory tracking.
//
// Linearizes the unicycle model around the reference trajectory and computes
// an optimal feedback gain K via the discrete-time algebraic Riccati equation.
//
// Requires Eigen3 (brew install eigen / apt install libeigen3-dev).
class LQRController {
public:
    // Q: state cost weights [x, y, theta, v]
    // R: control cost weights [velocity, steering]
    // dt: discretization timestep
    LQRController(const Eigen::Vector4d& Q = Eigen::Vector4d(10, 10, 5, 1),
                  const Eigen::Vector2d& R = Eigen::Vector2d(0.1, 0.1),
                  double dt = 0.05,
                  double wheelbase = 1.0);

    // Compute control given current state and reference point.
    Control compute(const State& state, const TrajectoryPoint& ref);

private:
    Eigen::Matrix4d K_;  // feedback gain matrix
    double dt_;
    double wheelbase_;
};

}  // namespace autompc
