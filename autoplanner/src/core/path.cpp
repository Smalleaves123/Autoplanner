#include "autoplanner/core/path.h"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <string>

#include "autoplanner/core/planner_result.h"
#include "autoplanner/metrics/path_metrics.h"

namespace autoplanner {

namespace {

std::string escapeJsonString(const std::string& value) {
    std::string escaped;
    escaped.reserve(value.size());
    for (const char character : value) {
        switch (character) {
            case '"': escaped += "\\\""; break;
            case '\\': escaped += "\\\\"; break;
            case '\n': escaped += "\\n"; break;
            case '\r': escaped += "\\r"; break;
            case '\t': escaped += "\\t"; break;
            default: escaped += character; break;
        }
    }
    return escaped;
}

void writeJsonNumber(std::ofstream& output, double value) {
    if (std::isfinite(value)) {
        output << value;
    } else {
        // JSON has no representation for NaN or infinity. Null preserves the
        // fact that the metric is unavailable without emitting invalid JSON.
        output << "null";
    }
}

}  // namespace

// Sum of Euclidean distances between consecutive waypoints.
double computePathLength(const Path2d& path) {
    if (path.size() < 2) {
        return 0.0;
    }

    double length = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) {
        length += distance(path[i - 1], path[i]);
    }

    return length;
}

// Write a standard CSV with header "x,y" and 6-decimal precision.
bool savePathCsv(const Path2d& path, const std::string& file_path) {
    std::ofstream fout(file_path);
    if (!fout.is_open()) {
        return false;
    }

    fout << "x,y\n";
    fout << std::fixed << std::setprecision(6);

    for (const auto& p : path) {
        fout << p.x << "," << p.y << "\n";
    }

    return true;
}

// Write a minimal JSON metrics file by hand (avoids pulling in a JSON library).
bool saveMetricsJson(const PlannerResult& result, const std::string& file_path) {
    std::ofstream fout(file_path);
    if (!fout.is_open()) {
        return false;
    }

    fout << "{\n";
    fout << "  \"planner_name\": \""
         << escapeJsonString(result.planner_name) << "\",\n";
    fout << "  \"status_code\": \"" << result.statusCodeString() << "\",\n";
    fout << "  \"success\": " << (result.success ? "true" : "false") << ",\n";
    fout << "  \"path_length\": ";
    writeJsonNumber(fout, result.path_length);
    fout << ",\n";
    fout << "  \"planning_time_ms\": ";
    writeJsonNumber(fout, result.planning_time_ms);
    fout << ",\n";
    fout << "  \"expanded_nodes\": " << result.expanded_nodes << ",\n";
    fout << "  \"iterations\": " << result.iterations << ",\n";
    fout << "  \"path_points\": " << result.path.size() << ",\n";
    fout << "  \"collision_free\": "
         << (result.collision_free ? "true" : "false") << ",\n";
    fout << "  \"turning_count\": " << result.turning_count << ",\n";
    fout << "  \"total_turning\": ";
    writeJsonNumber(fout, result.total_turning);
    fout << ",\n";
    fout << "  \"average_curvature\": ";
    writeJsonNumber(fout, result.average_curvature);
    fout << ",\n";
    fout << "  \"smoothness_score\": ";
    writeJsonNumber(fout, result.smoothness_score);
    fout << ",\n";
    fout << "  \"minimum_obstacle_distance\": ";
    writeJsonNumber(fout, result.minimum_obstacle_distance);
    fout << ",\n";
    fout << "  \"message\": \"" << escapeJsonString(result.message)
         << "\"\n";
    fout << "}\n";

    return true;
}

}  // namespace autoplanner
