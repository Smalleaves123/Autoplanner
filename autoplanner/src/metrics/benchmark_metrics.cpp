#include "autoplanner/metrics/benchmark_metrics.h"

namespace autoplanner {

BenchmarkMetrics::BenchmarkMetrics() = default;

void BenchmarkMetrics::addResult(const std::string& planner,
                                  const std::string& map,
                                  const PlannerResult& result) {
    Entry e;
    e.planner = planner;
    e.map = map;
    e.success = result.success;
    e.time_ms = result.planning_time_ms;
    e.path_length = result.path_length;
    e.expanded_nodes = result.expanded_nodes;
    entries_.push_back(e);
}

std::string BenchmarkMetrics::toCsv() const {
    std::string csv = "planner,map,success,time_ms,path_length,expanded_nodes\n";
    for (auto& e : entries_) {
        csv += e.planner + "," + e.map + "," +
               std::to_string(e.success) + "," +
               std::to_string(e.time_ms) + "," +
               std::to_string(e.path_length) + "," +
               std::to_string(e.expanded_nodes) + "\n";
    }
    return csv;
}

}  // namespace autoplanner
