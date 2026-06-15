// improved_astar_example.cpp — Improved A* vs standard A* comparison
//
// Demonstrates how Improved A* produces paths that stay farther from
// obstacles and have fewer turns, compared to standard A*.
//
// Build and run:
//   cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
//   cmake --build build -j
//   ./build/examples/improved_astar_example

#include <iostream>

#include "autoplanner/core/grid_map.h"
#include "autoplanner/core/path.h"
#include "autoplanner/costmap/costmap_2d.h"
#include "autoplanner/metrics/path_metrics.h"
#include "autoplanner/planners/graph_search/astar.h"
#include "autoplanner/planners/graph_search/improved_astar.h"

int main() {
    using namespace autoplanner;

    GridMap map;
    if (!map.loadFromTxt("data/maps/warehouse_100x100.txt")) {
        std::cerr << "Failed to load map\n";
        return 1;
    }

    // Build a costmap with inflated obstacles
    Costmap2D costmap;
    costmap.buildFromGridMap(map);
    costmap.inflateObstacles(1.0);  // robot radius in cells

    Point2i start{3, 5};
    Point2i goal{90, 80};

    // --- Standard A* ---
    AStarPlanner astar(true);
    auto r_astar = astar.plan(map, start, goal);

    // --- Improved A* ---
    ImprovedAStarPlanner improved(1.0, 2.0, 0.5, true);
    improved.setCostmap(&costmap);
    auto r_improved = improved.plan(map, start, goal);

    // --- Compare ---
    std::cout << "Map: " << map.width() << "x" << map.height() << "\n";
    std::cout << "Start: (" << start.x << "," << start.y
              << ")  Goal: (" << goal.x << "," << goal.y << ")\n\n";

    std::cout << "┌─────────────────┬────────────┬──────────────┐\n";
    std::cout << "│ Metric          │ A*         │ Improved A*  │\n";
    std::cout << "├─────────────────┼────────────┼──────────────┤\n";
    std::cout << "│ success         │ " << (r_astar.success ? "yes" : "no")
              << "         │ " << (r_improved.success ? "yes" : "no")
              << "          │\n";
    std::cout << "│ time (ms)       │ " << r_astar.planning_time_ms
              << "      │ " << r_improved.planning_time_ms << "        │\n";
    std::cout << "│ path length     │ " << r_astar.path_length
              << "      │ " << r_improved.path_length << "        │\n";
    std::cout << "│ expanded nodes  │ " << r_astar.expanded_nodes
              << "        │ " << r_improved.expanded_nodes << "          │\n";
    std::cout << "│ turning count   │ " << computeTurningCount(r_astar.path)
              << "          │ " << computeTurningCount(r_improved.path)
              << "            │\n";
    std::cout << "│ smoothness      │ " << computeSmoothnessScore(r_astar.path)
              << "       │ " << computeSmoothnessScore(r_improved.path)
              << "         │\n";
    std::cout << "└─────────────────┴────────────┴──────────────┘\n";

    // Save both paths for visualization
    if (r_astar.success)
        savePathCsv(r_astar.path, "results/astar_path.csv");
    if (r_improved.success)
        savePathCsv(r_improved.path, "results/improved_astar_path.csv");

    std::cout << "\nPaths saved to results/astar_path.csv and "
              << "results/improved_astar_path.csv\n";

    return 0;
}
