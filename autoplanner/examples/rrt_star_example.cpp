// rrt_star_example.cpp — RRT* planner demo showing path cost improvement
//
// Demonstrates that RRT* converges to a better path than RRT by using
// parent reselection and tree rewiring.
//
// Build and run:
//   cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
//   cmake --build build -j
//   ./build/examples/rrt_star_example

#include <fstream>
#include <iostream>

#include "autoplanner/core/grid_map.h"
#include "autoplanner/core/path.h"
#include "autoplanner/planners/sampling/rrt.h"
#include "autoplanner/planners/sampling/rrt_star.h"

int main() {
    using namespace autoplanner;

    GridMap map;
    if (!map.loadFromTxt("data/maps/warehouse_100x100.txt")) {
        std::cerr << "Failed to load map\n";
        return 1;
    }

    Point2i start{1, 1};
    Point2i goal{98, 98};

    // --- RRT (baseline) ---
    RRTPlanner rrt(2.0, 8000, 0.1, 2.0);
    auto r_rrt = rrt.plan(map, start, goal);

    // --- RRT* (asymptotically optimal) ---
    RRTStarPlanner rrt_star(/*step_size=*/2.0,
                            /*max_iter=*/8000,
                            /*goal_sample_rate=*/0.1,
                            /*goal_tolerance=*/2.0,
                            /*rewire_radius=*/5.0);
    auto r_rrt_star = rrt_star.plan(map, start, goal);

    // --- Compare ---
    std::cout << "Map: " << map.width() << "x" << map.height() << "\n";
    std::cout << "Start: (" << start.x << "," << start.y
              << ")  Goal: (" << goal.x << "," << goal.y << ")\n\n";

    std::cout << "┌─────────────────┬────────────┬──────────────┐\n";
    std::cout << "│ Metric          │ RRT        │ RRT*         │\n";
    std::cout << "├─────────────────┼────────────┼──────────────┤\n";
    std::cout << "│ success         │ " << (r_rrt.success ? "yes" : "no")
              << "         │ " << (r_rrt_star.success ? "yes" : "no")
              << "          │\n";
    std::cout << "│ time (ms)       │ " << r_rrt.planning_time_ms
              << "      │ " << r_rrt_star.planning_time_ms
              << "        │\n";
    std::cout << "│ path length     │ " << r_rrt.path_length
              << "      │ " << r_rrt_star.path_length
              << "        │\n";
    std::cout << "│ path points     │ " << r_rrt.path.size()
              << "          │ " << r_rrt_star.path.size()
              << "          │\n";
    std::cout << "└─────────────────┴────────────┴──────────────┘\n\n";

    std::cout << "RRT* path is "
              << (r_rrt_star.path_length < r_rrt.path_length ? "shorter" : "not shorter")
              << " than RRT path.\n";

    // Save paths
    if (r_rrt.success)
        savePathCsv(r_rrt.path, "results/rrt_path.csv");
    if (r_rrt_star.success)
        savePathCsv(r_rrt_star.path, "results/rrt_star_path.csv");

    // Save RRT* tree edges
    const auto& edges = rrt_star.treeEdges();
    std::ofstream tree("results/rrt_star_tree.csv");
    tree << "x1,y1,x2,y2\n";
    for (const auto& [a, b] : edges)
        tree << a.x << "," << a.y << "," << b.x << "," << b.y << "\n";
    std::cout << "Tree edges (" << edges.size()
              << ") saved to results/rrt_star_tree.csv\n";

    return 0;
}
