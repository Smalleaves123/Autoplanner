#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "autoplanner/core/grid_map.h"
#include "autoplanner/core/planner_base.h"
#include "autoplanner/planners/graph_search/astar.h"
#include "autoplanner/planners/graph_search/dijkstra.h"
#include "autoplanner/planners/graph_search/improved_astar.h"
#include "autoplanner/planners/graph_search/jps.h"
#include "autoplanner/planners/graph_search/weighted_astar.h"
#include "autoplanner/planners/sampling/rrt.h"
#include "autoplanner/planners/sampling/rrt_star.h"
#include "autoplanner/planners/sampling/informed_rrt_star.h"
#include "autoplanner/costmap/costmap_2d.h"
#include "autoplanner/costmap/obstacle_inflation.h"

using namespace autoplanner;

struct BenchResult {
    std::string planner;
    std::string map_name;
    bool success = false;
    double time_ms = 0.0;
    double path_len = 0.0;
    int expanded = 0;
};

int main() {
    std::vector<std::string> maps = {
        "data/maps/simple_50x50.txt",
        "data/maps/maze_100x100.txt",
        "data/maps/warehouse_100x100.txt",
        "data/maps/random_100x100_density_20.txt",
    };
    std::vector<Point2i> starts = {{1,1},{1,1},{1,1},{1,1}};
    std::vector<Point2i> goals  = {{48,48},{98,98},{98,98},{98,98}};

    std::vector<BenchResult> results;

    for (size_t mi = 0; mi < maps.size(); ++mi) {
        GridMap map;
        if (!map.loadFromTxt(maps[mi])) {
            std::cerr << "Skip: " << maps[mi] << "\n";
            continue;
        }
        Costmap2D costmap;
        costmap.buildFromGridMap(map);
        costmap.inflateObstacles(1.0);

        std::cout << "\n=== Map: " << maps[mi]
                  << " (" << map.width() << "x" << map.height() << ") ===\n";

        auto run = [&](std::unique_ptr<PlannerBase> p, const std::string& name) {
            auto r = p->plan(map, starts[mi], goals[mi]);
            results.push_back({name, maps[mi], r.success,
                               r.planning_time_ms, r.path_length,
                               r.expanded_nodes});
            std::cout << std::left << std::setw(20) << name
                      << (r.success ? " OK " : "FAIL")
                      << std::setw(10) << r.planning_time_ms << " ms  "
                      << std::setw(10) << r.path_length << " len  "
                      << std::setw(8) << r.expanded_nodes << " nodes\n";
        };

        run(std::make_unique<DijkstraPlanner>(true), "dijkstra");
        run(std::make_unique<AStarPlanner>(true), "astar");
        run(std::make_unique<WeightedAStarPlanner>(1.5, true), "weighted_astar");
        {
            auto p = std::make_unique<ImprovedAStarPlanner>(1.0, 2.0, 0.5, true);
            p->setCostmap(&costmap);
            run(std::move(p), "improved_astar");
        }
        run(std::make_unique<JPSPlanner>(true), "jps");
        run(std::make_unique<RRTPlanner>(2.0, 5000, 0.1, 2.0), "rrt");
        run(std::make_unique<RRTStarPlanner>(2.0, 5000, 0.1, 2.0, 5.0), "rrt_star");
        run(std::make_unique<InformedRRTStarPlanner>(2.0, 5000, 0.1, 2.0, 5.0), "informed_rrt_star");
    }

    // Write CSV
    std::ofstream fout("results/benchmark/benchmark_all.csv");
    fout << "planner,map,success,time_ms,path_length,expanded_nodes\n";
    fout << std::fixed << std::setprecision(3);
    for (auto& r : results)
        fout << r.planner << "," << r.map_name << ","
             << r.success << "," << r.time_ms << ","
             << r.path_len << "," << r.expanded << "\n";
    std::cout << "\nResults written to results/benchmark/benchmark_all.csv\n";
    return 0;
}
