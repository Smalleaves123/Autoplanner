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
    int frames = 5;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--map" && i + 1 < argc) map_path = argv[++i];
        else if (arg == "--output" && i + 1 < argc) output = argv[++i];
        else if (arg == "--frames" && i + 1 < argc) frames = std::stoi(argv[++i]);
        else if (arg == "--help") {
            std::cout << "Dynamic D* Lite demo\n"
                      << "  --map PATH       occupancy grid map\n"
                      << "  --frames N       number of updates\n"
                      << "  --output PATH    CSV result path\n";
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

    const std::filesystem::path output_path(output);
    if (!output_path.parent_path().empty()) {
        std::filesystem::create_directories(output_path.parent_path());
    }
    std::ofstream file(output);
    if (!file.is_open()) {
        std::cerr << "Failed to open output: " << output << "\n";
        return 1;
    }

    file << "frame,success,planning_time_ms,expanded_nodes,path_length,obstacle_x,obstacle_y\n";
    file << std::fixed << std::setprecision(3);
    file << 0 << "," << result.success << "," << result.planning_time_ms << ","
         << result.expanded_nodes << "," << result.path_length << ",-1,-1\n";

    for (int frame = 1; frame < frames; ++frame) {
        if (result.path.size() > 4) {
            const auto& point = result.path[result.path.size() / 2];
            const int obstacle_x = static_cast<int>(point.x);
            const int obstacle_y = static_cast<int>(point.y);
            map.setOccupied(obstacle_x, obstacle_y, true);

            result = planner.replan(map, start);
            file << frame << "," << result.success << ","
                 << result.planning_time_ms << "," << result.expanded_nodes << ","
                 << result.path_length << "," << obstacle_x << "," << obstacle_y << "\n";
            if (!result.success) break;
        }
    }

    std::cout << "Dynamic replanning results: " << output << "\n";
    return 0;
}
