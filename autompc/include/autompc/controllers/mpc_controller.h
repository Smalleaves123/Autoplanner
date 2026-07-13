#pragma once

#include <Eigen/Dense>

#include "autompc/core/types.h"
#include "autompc/core/trajectory.h"

namespace autompc {

// Finite-horizon linearized MPC for the unicycle model.
//
// At every call the controller linearizes the dynamics along a local
// reference horizon, solves the finite-horizon Riccati recursion, applies
// only the first control, and repeats at the next control cycle.
class MPCController {
public:
    MPCController(int horizon = 15,
                  double dt = 0.05,
                  double wheelbase = 1.0,
                  double max_velocity = 2.0,
                  double max_steering = 0.7,
                  const Eigen::Vector4d& state_weights =
                      Eigen::Vector4d(10.0, 10.0, 5.0, 1.0),
                  const Eigen::Vector2d& input_weights =
                      Eigen::Vector2d(0.1, 0.1));

    // Compute the first control in the receding-horizon solution.
    Control compute(const State& state,
                    const Trajectory& reference,
                    double target_velocity);

    int horizon() const { return horizon_; }

private:
    int horizon_;
    double dt_;
    double wheelbase_;
    double max_velocity_;
    double max_steering_;
    Eigen::Vector4d state_weights_;
    Eigen::Vector2d input_weights_;
};

}  // namespace autompc
