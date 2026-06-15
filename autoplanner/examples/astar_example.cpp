// astar_example.cpp — A* planner demo
//
// Build and run:
//   cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
//   cmake --build build -j
//   ./build/examples/astar_example

#include <iostream>

#include "autoplanner/core/grid_map.h"
#include "autoplanner/core/path.h"
#include "autoplanner/heuristics/euclidean.h"
#include "autoplanner/heuristics/manhattan.h"
#include "autoplanner/planners/graph_search/astar.h"

int main() {
    using namespace autoplanner;

    GridMap map;
    if (!map.loadFromTxt("data/maps/simple_50x50.txt")) {
        std::cerr << "Failed to load map\n";
        return 1;
    }

    std::cout << "Map: " << map.width() << "x" << map.height() << "\n\n";

    AStarPlanner astar(true);  // allow diagonal moves

    // --- Euclidean heuristic (admissible for all grids) ---
    astar.setHeuristic(std::make_unique<EuclideanHeuristic>());
    auto result = astar.plan(map, {1, 1}, {48, 48});

    std::cout << "=== A* with Euclidean ===\n";
    std::cout << "success: " << result.success << "\n";
    std::cout << "time: " << result.planning_time_ms << " ms\n";
    std::cout << "path length: " << result.path_length << "\n";
    std::cout << "expanded nodes: " << result.expanded_nodes << "\n";
    std::cout << "path points: " << result.path.size() << "\n\n";

    // --- Manhattan heuristic (only admissible for 4-connected) ---
    astar.setHeuristic(std::make_unique<ManhattanHeuristic>());
    auto result2 = astar.plan(map, {1, 1}, {48, 48});

    std::cout << "=== A* with Manhattan ===\n";
    std::cout << "time: " << result2.planning_time_ms << " ms\n";
    std::cout << "path length: " << result2.path_length << "\n";
    std::cout << "expanded nodes: " << result2.expanded_nodes << "\n\n";

    // Save path
    if (result.success) {
        savePathCsv(result.path, "results/astar_example_path.csv");
        std::cout << "Path saved to results/astar_example_path.csv\n";
    }

    return 0;
}
