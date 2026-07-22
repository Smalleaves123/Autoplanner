#pragma once

#include <cstddef>

#include <Eigen/Dense>

#include "autompc/core/types.h"
#include "autompc/core/trajectory.h"

namespace autompc {

// Finite-horizon linearized MPC for the unicycle model.
//
// At every call the controller linearizes the dynamics along a local
// reference horizon, solves the finite-horizon Riccati recursion with a
// terminal state cost, applies only the first control, and repeats at the next
// control cycle. Velocity/steering bounds and input-rate limits are enforced;
// steering-rate regularization and bounded tracking errors stabilize the
// linearized feedback around sharp path changes.
class MPCController {
public:
    MPCController(int horizon = 15,
                  double dt = 0.05,
                  double wheelbase = 1.0,
                  double max_velocity = 2.0,
                  double max_steering = 0.7,
                  double max_acceleration = 1.5,
                  double max_deceleration = 2.0,
                  double max_steering_rate = 1.5,
                  const Eigen::Vector4d& state_weights =
                      Eigen::Vector4d(10.0, 10.0, 5.0, 1.0),
                  const Eigen::Vector2d& input_weights =
                      Eigen::Vector2d(0.1, 0.1),
                  const Eigen::Vector4d& terminal_state_weights =
                      Eigen::Vector4d(20.0, 20.0, 10.0, 2.0),
                  double steering_rate_weight = 0.2,
                  double max_position_error = 10.0,
                  double max_heading_error = 3.141592653589793);

    // Compute the first control in the receding-horizon solution.
    Control compute(const State& state,
                    const Trajectory& reference,
                    double target_velocity);

    // Clear the previous steering command used by the rate constraint.
    void reset();

    // Reset only the monotonic reference-progress cursor after a new path is
    // installed. The previous steering command is intentionally preserved.
    void resetReferenceProgress();

    int horizon() const { return horizon_; }
    std::size_t referenceIndex() const { return last_reference_index_; }

private:
    int horizon_;
    double dt_;
    double wheelbase_;
    double max_velocity_;
    double max_steering_;
    double max_acceleration_;
    double max_deceleration_;
    double max_steering_rate_;
    double last_steering_ = 0.0;
    Eigen::Vector4d state_weights_;
    Eigen::Vector2d input_weights_;
    Eigen::Vector4d terminal_state_weights_;
    double steering_rate_weight_;
    double max_position_error_;
    double max_heading_error_;
    std::size_t last_reference_index_ = 0;
};

}  // namespace autompc
