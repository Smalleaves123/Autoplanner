#include "autoplanner/core/path.h"

#include <fstream>
#include <iomanip>

#include "autoplanner/core/planner_result.h"

namespace autoplanner {

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
    fout << "  \"planner_name\": \"" << result.planner_name << "\",\n";
    fout << "  \"success\": " << (result.success ? "true" : "false") << ",\n";
    fout << "  \"path_length\": " << result.path_length << ",\n";
    fout << "  \"planning_time_ms\": " << result.planning_time_ms << ",\n";
    fout << "  \"expanded_nodes\": " << result.expanded_nodes << ",\n";
    fout << "  \"iterations\": " << result.iterations << ",\n";
    fout << "  \"path_points\": " << result.path.size() << ",\n";
    fout << "  \"message\": \"" << result.message << "\"\n";
    fout << "}\n";

    return true;
}

}  // namespace autoplanner
