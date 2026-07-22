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
    e.collision_free = result.collision_free;
    e.turning_count = result.turning_count;
    e.average_curvature = result.average_curvature;
    e.smoothness_score = result.smoothness_score;
    e.minimum_obstacle_distance = result.minimum_obstacle_distance;
    entries_.push_back(e);
}

std::string BenchmarkMetrics::toCsv() const {
    std::string csv =
        "planner,map,success,collision_free,planning_time_ms,path_length,"
        "expanded_nodes,turning_count,average_curvature,smoothness_score,"
        "minimum_obstacle_distance\n";
    for (auto& e : entries_) {
        csv += e.planner + "," + e.map + "," +
               std::to_string(e.success) + "," +
               std::to_string(e.collision_free) + "," +
               std::to_string(e.time_ms) + "," +
               std::to_string(e.path_length) + "," +
               std::to_string(e.expanded_nodes) + "," +
               std::to_string(e.turning_count) + "," +
               std::to_string(e.average_curvature) + "," +
               std::to_string(e.smoothness_score) + "," +
               std::to_string(e.minimum_obstacle_distance) + "\n";
    }
    return csv;
}

}  // namespace autoplanner
