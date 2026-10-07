#pragma once

#include <cmath>
#include <variant>

#include "autompc/simulation/dynamic_bicycle.h"
#include "autompc/simulation/kinematic_bicycle.h"

namespace robotnav {

// Per-rollout vehicle state. Each candidate trajectory owns one instance so
// MPPI can evaluate rollouts concurrently without sharing hidden dynamics.
class RolloutSimulator {
public:
    RolloutSimulator(const autompc::State& state,
                     double dt,
                     const autompc::SimulationOptions& options)
        : simulator_(makeSimulator(state, dt, options)) {}

    void setSteering(double steering) {
        std::visit([steering](auto& simulator) {
            simulator.setSteering(steering);
        }, simulator_);
    }

    autompc::State step(const autompc::Control& command) {
        auto state = std::visit(
            [&command](auto& simulator) { return simulator.step(command); },
            simulator_);
        state.theta = std::atan2(std::sin(state.theta), std::cos(state.theta));
        return state;
    }

private:
    using Variant = std::variant<autompc::KinematicBicycleSimulator,
                                 autompc::DynamicBicycleSimulator>;

    static Variant makeSimulator(const autompc::State& state,
                                 double dt,
                                 const autompc::SimulationOptions& options) {
        if (options.execution_model == "dynamic_bicycle") {
            autompc::DynamicBicycleOptions dynamic;
            dynamic.dt = dt;
            dynamic.wheelbase = options.wheelbase;
            dynamic.front_axle = options.front_axle;
            dynamic.rear_axle = options.rear_axle;
            dynamic.mass = options.mass;
            dynamic.yaw_inertia = options.yaw_inertia;
            dynamic.front_cornering_stiffness =
                options.front_cornering_stiffness;
            dynamic.rear_cornering_stiffness = options.rear_cornering_stiffness;
            dynamic.max_velocity = options.max_velocity;
            dynamic.max_acceleration = options.max_acceleration;
            dynamic.max_deceleration = options.max_deceleration;
            dynamic.max_steering = options.max_steering;
            dynamic.max_steering_rate = options.max_steering_rate;
            dynamic.allow_reverse = options.allow_reverse;
            dynamic.max_reverse_velocity = options.max_reverse_velocity;
            return Variant(std::in_place_type<autompc::DynamicBicycleSimulator>,
                           state, dynamic);
        }
        auto kinematic = options;
        kinematic.dt = dt;
        return Variant(std::in_place_type<autompc::KinematicBicycleSimulator>,
                       state, kinematic);
    }

    Variant simulator_;
};

}  // namespace robotnav
