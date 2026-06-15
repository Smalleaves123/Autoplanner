// rrt_example.cpp — RRT planner demo with tree visualization data
//
// Build and run:
//   cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
//   cmake --build build -j
//   ./build/examples/rrt_example

#include <fstream>
#include <iostream>

#include "autoplanner/core/grid_map.h"
#include "autoplanner/core/path.h"
#include "autoplanner/planners/sampling/rrt.h"

int main() {
    using namespace autoplanner;

    GridMap map;
    if (!map.loadFromTxt("data/maps/warehouse_100x100.txt")) {
        std::cerr << "Failed to load map\n";
        return 1;
    }

    // RRT with goal bias — samples the goal 10% of the time
    RRTPlanner rrt(/*step_size=*/2.0,
                   /*max_iter=*/5000,
                   /*goal_sample_rate=*/0.1,
                   /*goal_tolerance=*/2.0);

    auto result = rrt.plan(map, {1, 1}, {98, 98});

    std::cout << "=== RRT on warehouse_100x100 ===\n";
    std::cout << "success: " << result.success << "\n";
    std::cout << "iterations: " << result.iterations << "\n";
    std::cout << "time: " << result.planning_time_ms << " ms\n";
    std::cout << "path length: " << result.path_length << "\n";
    std::cout << "path points: " << result.path.size() << "\n";

    // Save path
    if (result.success) {
        savePathCsv(result.path, "results/rrt_path.csv");
        std::cout << "\nPath saved to results/rrt_path.csv\n";
    }

    // Save tree edges for visualization
    const auto& edges = rrt.treeEdges();
    std::ofstream tree_file("results/rrt_tree.csv");
    tree_file << "x1,y1,x2,y2\n";
    for (const auto& [a, b] : edges) {
        tree_file << a.x << "," << a.y << ","
                  << b.x << "," << b.y << "\n";
    }
    std::cout << "Tree edges (" << edges.size()
              << ") saved to results/rrt_tree.csv\n";

    return 0;
}
