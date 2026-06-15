// smoothing_example.cpp — Path smoothing demo (shortcut + Bezier)
//
// Shows how to post-process a raw A* path with shortcut smoothing
// followed by Bezier curve fitting.
//
// Build and run:
//   cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
//   cmake --build build -j
//   ./build/examples/smoothing_example

#include <iostream>

#include "autoplanner/core/grid_map.h"
#include "autoplanner/core/path.h"
#include "autoplanner/collision/line_collision_checker.h"
#include "autoplanner/metrics/path_metrics.h"
#include "autoplanner/planners/graph_search/astar.h"
#include "autoplanner/smoothing/bezier_smoother.h"
#include "autoplanner/smoothing/shortcut_smoother.h"

int main() {
    using namespace autoplanner;

    GridMap map;
    if (!map.loadFromTxt("data/maps/maze_100x100.txt")) {
        std::cerr << "Failed to load map\n";
        return 1;
    }

    LineCollisionChecker checker(map);

    // Plan a raw path with A*
    AStarPlanner astar(true);
    auto result = astar.plan(map, {1, 1}, {98, 98});

    if (!result.success) {
        std::cerr << "No path found\n";
        return 1;
    }

    auto raw = result.path;
    std::cout << "=== Path Smoothing Demo ===\n\n";
    std::cout << "Raw path:\n";
    std::cout << "  points: " << raw.size() << "\n";
    std::cout << "  length: " << computePathLength(raw) << "\n";
    std::cout << "  turns:  " << computeTurningCount(raw) << "\n\n";

    // Step 1: Shortcut smoothing (collision-aware)
    ShortcutSmoother shortcut(checker, 100);
    auto smoothed = shortcut.smooth(raw);

    std::cout << "After shortcut smoothing:\n";
    std::cout << "  points: " << smoothed.size() << "\n";
    std::cout << "  length: " << computePathLength(smoothed) << "\n";
    std::cout << "  turns:  " << computeTurningCount(smoothed) << "\n\n";

    // Step 2: Bezier curve fitting (visual smoothing)
    BezierSmoother bezier(10);
    auto bezier_smoothed = bezier.smooth(smoothed);

    std::cout << "After Bezier smoothing:\n";
    std::cout << "  points: " << bezier_smoothed.size() << "\n";
    std::cout << "  length: " << computePathLength(bezier_smoothed) << "\n\n";

    // Save results
    savePathCsv(raw, "results/raw_path.csv");
    savePathCsv(smoothed, "results/shortcut_smoothed_path.csv");
    savePathCsv(bezier_smoothed, "results/bezier_smoothed_path.csv");

    std::cout << "Paths saved to results/:\n";
    std::cout << "  raw_path.csv\n";
    std::cout << "  shortcut_smoothed_path.csv\n";
    std::cout << "  bezier_smoothed_path.csv\n";

    return 0;
}
