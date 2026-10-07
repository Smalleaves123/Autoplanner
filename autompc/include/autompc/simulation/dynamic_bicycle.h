#pragma once

#include "autompc/core/types.h"

namespace autompc {

// Actuator-limited dynamic single-track vehicle model.  The public State keeps
// the existing (x, y, heading, longitudinal velocity) ABI; lateral velocity
// and yaw rate are exposed separately because they are model states rather
// than planner-facing pose fields.
struct DynamicBicycleOptions {
    double dt = 0.05;
    double wheelbase = 1.0;
    double front_axle = 0.5;
    double rear_axle = 0.5;
    double mass = 20.0;
    double yaw_inertia = 10.0;
    double front_cornering_stiffness = 80.0;
    double rear_cornering_stiffness = 80.0;
    double max_velocity = 2.0;
    double max_acceleration = 1.5;
    double max_deceleration = 2.0;
    double max_steering = 0.7;
    double max_steering_rate = 1.5;
    bool allow_reverse = false;
    double max_reverse_velocity = 1.0;
};

class DynamicBicycleSimulator {
public:
    explicit DynamicBicycleSimulator(
        const State& initial, DynamicBicycleOptions options = {});

    State step(const Control& command);
    void reset(const State& state);
    void setSteering(double steering);

    const State& state() const { return state_; }
    double steering() const { return steering_; }
    double lateralVelocity() const { return lateral_velocity_; }
    double yawRate() const { return yaw_rate_; }
    const DynamicBicycleOptions& options() const { return options_; }

private:
    State state_;
    DynamicBicycleOptions options_;
    double steering_ = 0.0;
    double lateral_velocity_ = 0.0;
    double yaw_rate_ = 0.0;
};

}  // namespace autompc
