// costmap_example.cpp — Costmap2D and obstacle inflation demo
//
// Demonstrates building a costmap from a grid map, inflating obstacles
// for a given robot radius, and inspecting the resulting cost values.
//
// Build and run:
//   cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
//   cmake --build build -j
//   ./build/examples/costmap_example

#include <iomanip>
#include <iostream>

#include "autoplanner/core/grid_map.h"
#include "autoplanner/costmap/costmap_2d.h"

int main() {
    using namespace autoplanner;

    GridMap map;
    if (!map.loadFromTxt("data/maps/simple_50x50.txt")) {
        std::cerr << "Failed to load map\n";
        return 1;
    }

    // Build costmap from grid map
    Costmap2D costmap;
    costmap.buildFromGridMap(map);

    std::cout << "=== Costmap2D Demo ===\n\n";
    std::cout << "Map size: " << map.width() << "x" << map.height() << "\n\n";

    // Inspect costs before inflation
    std::cout << "Before inflation (obstacle=1.0, free=0.0):\n";
    for (int y = 0; y < 5; ++y) {
        for (int x = 0; x < 10; ++x) {
            std::cout << std::fixed << std::setprecision(1)
                      << costmap.getCost(x, y) << " ";
        }
        std::cout << "\n";
    }
    std::cout << "\n";

    // Inflate obstacles by robot radius (2 cells)
    costmap.inflateObstacles(2.0);

    std::cout << "After inflation (radius=2.0 cells):\n";
    std::cout << "Cells near obstacles get graduated cost values.\n";
    for (int y = 0; y < 5; ++y) {
        for (int x = 0; x < 10; ++x) {
            std::cout << std::fixed << std::setprecision(3)
                      << costmap.getCost(x, y) << " ";
        }
        std::cout << "\n";
    }

    // Count free vs high-cost cells
    int free = 0, occupied = 0, intermediate = 0;
    for (int y = 0; y < map.height(); ++y) {
        for (int x = 0; x < map.width(); ++x) {
            double c = costmap.getCost(x, y);
            if (c < 0.01) free++;
            else if (c >= 1.0) occupied++;
            else intermediate++;
        }
    }
    std::cout << "\nCell statistics:\n";
    std::cout << "  free:         " << free << "\n";
    std::cout << "  intermediate: " << intermediate << "\n";
    std::cout << "  occupied:     " << occupied << "\n";

    return 0;
}
