#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

#include "autoplanner/core/grid_map.h"
#include "autoplanner/planners/graph_search/dstar_lite.h"

using namespace autoplanner;

int main(int argc, char** argv) {
    std::string map_path = "data/maps/simple_50x50.txt";
    std::string output = "results/dynamic_replanning.csv";
    std::string summary_output;
    int frames = 5;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--map" && i + 1 < argc) map_path = argv[++i];
        else if (arg == "--output" && i + 1 < argc) output = argv[++i];
        else if (arg == "--summary" && i + 1 < argc) summary_output = argv[++i];
        else if (arg == "--frames" && i + 1 < argc) frames = std::stoi(argv[++i]);
        else if (arg == "--help") {
            std::cout << "Dynamic D* Lite demo\n"
                      << "  --map PATH       occupancy grid map\n"
                      << "  --frames N       number of updates\n"
                      << "  --output PATH    CSV result path\n"
                      << "  --summary PATH   JSON summary path\n";
            return 0;
        }
    }

    GridMap map;
    if (!map.loadFromTxt(map_path)) {
        std::cerr << "Failed to load map: " << map_path << "\n";
        return 1;
    }

    const Point2i start{1, 1};
    const Point2i goal{map.width() - 2, map.height() - 2};
    DStarLitePlanner planner(true);
    auto result = planner.plan(map, start, goal);
    if (!result.success) {
        std::cerr << "Initial planning failed: " << result.message << "\n";
        return 2;
    }
    const double initial_planning_time_ms = result.planning_time_ms;

    const std::filesystem::path output_path(output);
    if (!output_path.parent_path().empty()) {
        std::filesystem::create_directories(output_path.parent_path());
    }
    std::ofstream file(output);
    if (!file.is_open()) {
        std::cerr << "Failed to open output: " << output << "\n";
        return 1;
    }

    if (summary_output.empty()) {
        auto default_summary_path = output_path;
        default_summary_path.replace_extension(".json");
        summary_output = default_summary_path.string();
    }

    file << "frame,success,planning_time_ms,expanded_nodes,path_length,obstacle_x,obstacle_y,replanned\n";
    file << std::fixed << std::setprecision(3);
    file << 0 << "," << result.success << "," << result.planning_time_ms << ","
         << result.expanded_nodes << "," << result.path_length << ",-1,-1,0\n";

    int replanning_attempts = 0;
    int replanning_successes = 0;
    double total_replanning_time_ms = 0.0;
    int frames_run = 1;
    for (int frame = 1; frame < frames; ++frame) {
        if (result.path.size() > 4) {
            const auto& point = result.path[result.path.size() / 2];
            const int obstacle_x = static_cast<int>(point.x);
            const int obstacle_y = static_cast<int>(point.y);
            map.setOccupied(obstacle_x, obstacle_y, true);

            result = planner.replan(map, start);
            ++replanning_attempts;
            ++frames_run;
            if (result.success) ++replanning_successes;
            total_replanning_time_ms += result.planning_time_ms;
            file << frame << "," << result.success << ","
                 << result.planning_time_ms << "," << result.expanded_nodes << ","
                 << result.path_length << "," << obstacle_x << "," << obstacle_y
                 << ",1\n";
            if (!result.success) break;
        }
    }

    const std::filesystem::path summary_path(summary_output);
    if (!summary_path.parent_path().empty()) {
        std::filesystem::create_directories(summary_path.parent_path());
    }
    std::ofstream summary(summary_output);
    if (!summary.is_open()) {
        std::cerr << "Failed to open summary: " << summary_output << "\n";
        return 1;
    }
    summary << std::fixed << std::setprecision(6);
    summary << "{\n"
            << "  \"map\": \"" << map_path << "\",\n"
            << "  \"frames_requested\": " << frames << ",\n"
            << "  \"frames_run\": " << frames_run << ",\n"
            << "  \"initial_planning_time_ms\": "
            << initial_planning_time_ms << ",\n"
            << "  \"replanning_attempts\": " << replanning_attempts << ",\n"
            << "  \"replanning_successes\": " << replanning_successes << ",\n"
            << "  \"total_replanning_time_ms\": "
            << total_replanning_time_ms << ",\n"
            << "  \"mean_replanning_time_ms\": "
            << (replanning_attempts > 0
                    ? total_replanning_time_ms / replanning_attempts : 0.0)
            << ",\n"
            << "  \"final_success\": "
            << (result.success ? "true" : "false") << ",\n"
            << "  \"final_path_length\": " << result.path_length << "\n"
            << "}\n";

    std::cout << "Dynamic replanning results: " << output << "\n"
              << "Summary: " << summary_output << "\n";
    return 0;
}
