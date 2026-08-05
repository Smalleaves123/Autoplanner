#pragma once

#include "autompc/core/types.h"

namespace autompc {

// Execution limits for the local, ROS-free kinematic simulation backend.
// Commands are treated as desired velocity and steering values; the simulator
// applies actuator limits before integrating the vehicle state.
struct SimulationOptions {
    double dt = 0.05;
    double wheelbase = 1.0;
    double max_velocity = 2.0;
    double max_acceleration = 1.5;
    double max_deceleration = 2.0;
    double max_steering = 0.7;
    double max_steering_rate = 1.5;
    bool allow_reverse = false;
    double max_reverse_velocity = 1.0;
};

class KinematicBicycleSimulator {
public:
    explicit KinematicBicycleSimulator(
        const State& initial,
        SimulationOptions options = {});

    // Apply a desired control command and return the constrained next state.
    State step(const Control& command);

    void reset(const State& state);

    const State& state() const { return state_; }
    double steering() const { return steering_; }
    const SimulationOptions& options() const { return options_; }

private:
    State state_;
    SimulationOptions options_;
    double steering_ = 0.0;
};

}  // namespace autompc
