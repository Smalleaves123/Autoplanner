#include <iomanip>
#include <iostream>
#include <memory>

#include "autoplanner/core/grid_map.h"
#include "autoplanner/planners/graph_search/astar.h"
#include "autoplanner/planners/graph_search/dijkstra.h"
#include "autoplanner/planners/graph_search/weighted_astar.h"
#include "autoplanner/planners/graph_search/jps.h"
#include "autoplanner/planners/sampling/rrt.h"
#include "autoplanner/planners/sampling/rrt_star.h"

using namespace autoplanner;

int main(int argc, char** argv) {
    std::string map_path = "data/maps/simple_50x50.txt";
    Point2i start{1,1}, goal{48,48};
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--map" && i+1<argc) map_path = argv[++i];
        else if (a == "--start" && i+2<argc) start = {std::stoi(argv[++i]), std::stoi(argv[++i])};
        else if (a == "--goal" && i+2<argc) goal = {std::stoi(argv[++i]), std::stoi(argv[++i])};
    }

    GridMap map;
    if (!map.loadFromTxt(map_path)) { std::cerr << "Map error\n"; return 1; }

    std::cout << std::left << std::setw(18) << "Planner"
              << std::setw(8) << "OK" << std::setw(12) << "Time(ms)"
              << std::setw(12) << "Length" << std::setw(12) << "Nodes"
              << std::setw(10) << "Points\n";
    std::cout << std::string(70, '-') << "\n";

    auto run = [&](std::unique_ptr<PlannerBase> p, const std::string& name) {
        auto r = p->plan(map, start, goal);
        std::cout << std::left << std::setw(18) << name
                  << std::setw(8) << (r.success ? "OK" : "FAIL")
                  << std::setw(12) << r.planning_time_ms
                  << std::setw(12) << r.path_length
                  << std::setw(12) << r.expanded_nodes
                  << std::setw(10) << r.path.size() << "\n";
    };

    run(std::make_unique<DijkstraPlanner>(true), "dijkstra");
    run(std::make_unique<AStarPlanner>(true), "astar");
    run(std::make_unique<WeightedAStarPlanner>(1.5, true), "weighted_astar(1.5)");
    run(std::make_unique<JPSPlanner>(true), "jps");
    run(std::make_unique<RRTPlanner>(2.0, 5000, 0.1, 2.0), "rrt");
    run(std::make_unique<RRTStarPlanner>(2.0, 5000, 0.1, 2.0, 5.0), "rrt_star");

    return 0;
}
