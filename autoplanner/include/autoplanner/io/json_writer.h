#pragma once

#include <string>
#include "autoplanner/core/planner_result.h"

namespace autoplanner {
namespace io {

// Save PlannerResult metrics to a JSON file.
// Delegates to saveMetricsJson.
inline bool saveMetrics(const PlannerResult& result, const std::string& path) {
    return saveMetricsJson(result, path);
}

}  // namespace io
}  // namespace autoplanner
