#include <filesystem>
#include <iostream>
#include <memory>
#include <string>

#include "autoplanner/core/grid_map.h"
#include "autoplanner/core/path.h"
#include "autoplanner/planners/graph_search/astar.h"
#include "autoplanner/planners/graph_search/dijkstra.h"
#include "autoplanner/planners/graph_search/weighted_astar.h"
#include "autoplanner/planners/graph_search/improved_astar.h"
#include "autoplanner/planners/graph_search/jps.h"
#include "autoplanner/planners/sampling/rrt.h"
#include "autoplanner/planners/sampling/rrt_star.h"
#include "autoplanner/planners/sampling/bi_rrt.h"
#include "autoplanner/costmap/costmap_2d.h"
#include "autoplanner/costmap/obstacle_inflation.h"

using namespace autoplanner;

int main(int argc, char** argv) {
    std::string planner_name = "astar";
    std::string map_path = "data/maps/simple_50x50.txt";
    std::string output_dir = "results";
    Point2i start{1, 1}, goal{48, 48};
    double weight = 1.5;
    double robot_radius = 1.0;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--planner" && i+1 < argc) planner_name = argv[++i];
        else if (arg == "--map" && i+1 < argc) map_path = argv[++i];
        else if (arg == "--output" && i+1 < argc) output_dir = argv[++i];
        else if (arg == "--start" && i+2 < argc) {
            start = {std::stoi(argv[++i]), std::stoi(argv[++i])}; }
        else if (arg == "--goal" && i+2 < argc) {
            goal = {std::stoi(argv[++i]), std::stoi(argv[++i])}; }
        else if (arg == "--weight" && i+1 < argc) weight = std::stod(argv[++i]);
        else if (arg == "--robot-radius" && i+1 < argc) robot_radius = std::stod(argv[++i]);
        else if (arg == "--help") {
            std::cout << "AutoPlanner CLI\n"
                      << "  --planner <name>   astar|dijkstra|weighted_astar|improved_astar|jps|rrt|rrt_star|bi_rrt\n"
                      << "  --map <path>       grid map file\n"
                      << "  --start <x> <y>    start coordinates\n"
                      << "  --goal <x> <y>     goal coordinates\n"
                      << "  --output <dir>     output directory\n";
            return 0;
        }
    }

    GridMap map;
    if (!map.loadFromTxt(map_path)) {
        std::cerr << "Failed to load map: " << map_path << "\n"; return 1;
    }

    Costmap2D costmap;
    if (planner_name == "improved_astar") {
        costmap.buildFromGridMap(map);
        costmap.inflateObstacles(robot_radius);
    }

    std::unique_ptr<PlannerBase> planner;
    if (planner_name == "astar") planner = std::make_unique<AStarPlanner>(true);
    else if (planner_name == "dijkstra") planner = std::make_unique<DijkstraPlanner>(true);
    else if (planner_name == "weighted_astar") planner = std::make_unique<WeightedAStarPlanner>(weight, true);
    else if (planner_name == "jps") planner = std::make_unique<JPSPlanner>(true);
    else if (planner_name == "rrt") planner = std::make_unique<RRTPlanner>(2.0, 5000, 0.1, 2.0);
    else if (planner_name == "rrt_star") planner = std::make_unique<RRTStarPlanner>(2.0, 5000, 0.1, 2.0, 5.0);
    else if (planner_name == "bi_rrt") planner = std::make_unique<BiRRTPlanner>(2.0, 5000, 2.0);
    else if (planner_name == "improved_astar") {
        auto p = std::make_unique<ImprovedAStarPlanner>(1.0, 2.0, 0.5, true);
        p->setCostmap(&costmap);
        planner = std::move(p);
    } else {
        std::cerr << "Unknown planner: " << planner_name << "\n"; return 1;
    }

    std::cout << "Map: " << map.width() << "x" << map.height()
              << "  Planner: " << planner->name()
              << "  Start: (" << start.x << "," << start.y
              << ")  Goal: (" << goal.x << "," << goal.y << ")\n";

    auto result = planner->plan(map, start, goal);
    std::cout << (result.success ? "SUCCESS" : "FAIL")
              << "  Time: " << result.planning_time_ms << " ms"
              << "  Length: " << result.path_length
              << "  Nodes: " << result.expanded_nodes
              << "  Points: " << result.path.size() << "\n";

    std::filesystem::create_directories(output_dir);
    if (result.success)
        savePathCsv(result.path, output_dir + "/path.csv");
    saveMetricsJson(result, output_dir + "/metrics.json");
    std::cout << "Output: " << output_dir << "/\n";
    return result.success ? 0 : 2;
}
