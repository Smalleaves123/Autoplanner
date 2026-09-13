#pragma once

#include "autompc/core/types.h"

namespace autompc {

// Velocity and actuator limits for a ROS-free differential-drive backend.
// Wheel velocities are derived from the requested body twist and uniformly
// scaled when necessary, preserving the requested path curvature.
struct DifferentialDriveOptions {
    double dt = 0.05;
    double track_width = 0.5;
    double max_linear_velocity = 2.0;
    double max_reverse_velocity = 1.0;
    double max_linear_acceleration = 1.5;
    double max_linear_deceleration = 2.0;
    double max_angular_velocity = 3.0;
    double max_angular_acceleration = 4.0;
    double max_wheel_velocity = 2.5;
    bool allow_reverse = false;
};

struct DifferentialDriveCommand {
    double linear_velocity = 0.0;
    double angular_velocity = 0.0;
};

class DifferentialDriveSimulator {
public:
    explicit DifferentialDriveSimulator(
        const State& initial,
        DifferentialDriveOptions options = {});

    // Apply a desired body twist and return the constrained next state.
    State step(const DifferentialDriveCommand& command);

    void reset(const State& state);

    const State& state() const { return state_; }
    double angularVelocity() const { return angular_velocity_; }
    double leftWheelVelocity() const { return left_wheel_velocity_; }
    double rightWheelVelocity() const { return right_wheel_velocity_; }
    const DifferentialDriveOptions& options() const { return options_; }

private:
    State state_;
    DifferentialDriveOptions options_;
    double angular_velocity_ = 0.0;
    double left_wheel_velocity_ = 0.0;
    double right_wheel_velocity_ = 0.0;
};

}  // namespace autompc
