#pragma once

#include <string_view>

namespace robotnav {

// Stable machine-readable outcome codes shared by planning, tracking, and
// execution layers. Human-readable messages remain useful for logs, but
// callers should branch on this enum instead of parsing text.
enum class StatusCode {
    Success,
    InvalidStart,
    InvalidGoal,
    NoPath,
    Timeout,
    InvalidConfiguration,
    Collision,
    InvalidTrajectory,
    ControllerInfeasible,
    SafeStop,
    ReplanningFailed,
    InternalError,
};

constexpr std::string_view toString(StatusCode code) noexcept {
    switch (code) {
        case StatusCode::Success: return "success";
        case StatusCode::InvalidStart: return "invalid_start";
        case StatusCode::InvalidGoal: return "invalid_goal";
        case StatusCode::NoPath: return "no_path";
        case StatusCode::Timeout: return "timeout";
        case StatusCode::InvalidConfiguration: return "invalid_configuration";
        case StatusCode::Collision: return "collision";
        case StatusCode::InvalidTrajectory: return "invalid_trajectory";
        case StatusCode::ControllerInfeasible: return "controller_infeasible";
        case StatusCode::SafeStop: return "safe_stop";
        case StatusCode::ReplanningFailed: return "replanning_failed";
        case StatusCode::InternalError: return "internal_error";
    }
    return "internal_error";
}

}  // namespace robotnav
