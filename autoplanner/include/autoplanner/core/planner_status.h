#pragma once

namespace autoplanner {

enum class PlannerStatus {
    SUCCESS,
    FAILURE,
    TIMEOUT,
    INVALID_START,
    INVALID_GOAL,
};

}  // namespace autoplanner
