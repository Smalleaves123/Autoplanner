#pragma once

#include <string>
#include <vector>

#include "autoplanner/core/planner_result.h"

namespace autoplanner {

// Collects benchmark results and can output them as CSV.
class BenchmarkMetrics {
public:
    BenchmarkMetrics();

    // Record one planner run.
    void addResult(const std::string& planner, const std::string& map,
                   const PlannerResult& result);

    // Return all results as CSV-formatted string.
    std::string toCsv() const;

private:
    struct Entry {
        std::string planner;
        std::string map;
        bool success = false;
        double time_ms = 0.0;
        double path_length = 0.0;
        int expanded_nodes = 0;
    };
    std::vector<Entry> entries_;
};

}  // namespace autoplanner
