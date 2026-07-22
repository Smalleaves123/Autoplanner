#include "robotnav/safety_supervisor.h"

#include <cmath>

#include "autoplanner/collision/grid_collision_checker.h"

namespace robotnav {
namespace {

bool finiteState(const autompc::State& state) {
    return std::isfinite(state.x) && std::isfinite(state.y) &&
           std::isfinite(state.theta) && std::isfinite(state.v);
}

bool finiteCommand(const autompc::Control& command) {
    return std::isfinite(command.velocity) &&
           std::isfinite(command.steering);
}

}  // namespace

SafetySupervisor::SafetySupervisor(const autoplanner::GridMap& map,
                                   SafetyOptions options)
    : map_(map), options_(options) {}

SafetyDecision SafetySupervisor::validateTrajectory(
    const autompc::Trajectory& trajectory) const {
    if (trajectory.empty()) {
        return {false, StatusCode::InvalidTrajectory,
                "reference trajectory is empty"};
    }

    for (const auto& point : trajectory) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.theta) || !std::isfinite(point.v) ||
            !std::isfinite(point.curvature) ||
            !std::isfinite(point.acceleration) || point.v < 0.0) {
            return {false, StatusCode::InvalidTrajectory,
                    "reference trajectory contains a non-finite or negative value"};
        }
    }
    return {true, StatusCode::Success, "trajectory is valid"};
}

SafetyDecision SafetySupervisor::validateCommand(
    const autompc::Control& command) const {
    if (!finiteCommand(command)) {
        return {false, StatusCode::ControllerInfeasible,
                "controller produced a non-finite command"};
    }
    if (command.velocity < -1e-9 ||
        command.velocity > options_.max_velocity + 1e-9 ||
        std::abs(command.steering) > options_.max_steering + 1e-9) {
        return {false, StatusCode::ControllerInfeasible,
                "controller command exceeds configured limits"};
    }
    return {true, StatusCode::Success, "command is valid"};
}

SafetyDecision SafetySupervisor::validateState(
    const autompc::State& state) const {
    if (!finiteState(state)) {
        return {false, StatusCode::SafeStop,
                "simulator produced a non-finite state"};
    }
    if (state.v < -1e-9 || state.v > options_.max_velocity + 1e-9) {
        return {false, StatusCode::SafeStop,
                "state velocity exceeds configured limits"};
    }
    if (options_.enforce_collision) {
        autoplanner::GridCollisionChecker checker(map_);
        if (!checker.isStateValid({state.x, state.y})) {
            return {false, StatusCode::Collision,
                    "simulated state entered an occupied or out-of-bounds cell"};
        }
    }
    return {true, StatusCode::Success, "state is safe"};
}

bool SafetySupervisor::goalReached(
    const autompc::State& state,
    const autompc::Trajectory& trajectory) const {
    if (trajectory.empty()) return false;
    const auto& goal = trajectory.back();
    return std::hypot(state.x - goal.x, state.y - goal.y) <=
           options_.goal_tolerance;
}

}  // namespace robotnav
