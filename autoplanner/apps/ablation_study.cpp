// ablation_study.cpp — Improved A* ablation experiment
//
// Runs 5 configurations to isolate the effect of each cost term:
//   1. A* baseline       f = g + h
//   2. Weighted A*       f = g + w_h * h
//   3. A* + obstacle     f = g + h + w_obs * obstacle_cost
//   4. A* + turning      f = g + h + w_turn * turning_cost
//   5. Improved A*       f = g + w_h * h + w_obs * obstacle_cost + w_turn * turning_cost
//
// Build: cmake --build build -j
// Run:   ./build/apps/ablation_study

#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <vector>

#include "autoplanner/core/grid_map.h"
#include "autoplanner/costmap/costmap_2d.h"
#include "autoplanner/metrics/path_metrics.h"
#include "autoplanner/planners/graph_search/astar.h"
#include "autoplanner/planners/graph_search/improved_astar.h"
#include "autoplanner/planners/graph_search/weighted_astar.h"

using namespace autoplanner;

struct AblationRow {
    std::string name;
    bool success = false;
    double time_ms = 0.0;
    double path_len = 0.0;
    int expanded = 0;
    int turns = 0;
    double smoothness = 0.0;
    double min_obs_dist = 0.0;
};

// Extract obstacle positions from map for min-distance calculation.
std::vector<Point2i> extractObstacles(const GridMap& map) {
    std::vector<Point2i> obs;
    for (int y = 0; y < map.height(); ++y)
        for (int x = 0; x < map.width(); ++x)
            if (map.isOccupied(x, y))
                obs.emplace_back(x, y);
    return obs;
}

int main() {
    std::vector<std::string> maps = {
        "data/maps/warehouse_100x100.txt",
        "data/maps/maze_100x100.txt",
    };
    std::vector<Point2i> starts = {{3, 5}, {1, 1}};
    std::vector<Point2i> goals  = {{90, 80}, {98, 98}};

    std::ofstream csv("results/benchmark/ablation.csv");
    csv << "config,map,success,time_ms,path_length,expanded_nodes,"
        << "turning_count,smoothness,min_obs_dist\n";
    csv << std::fixed << std::setprecision(4);

    std::cout << std::left << std::setw(22) << "Configuration"
              << std::setw(13) << "Map"
              << std::setw(6) << "OK"
              << std::setw(10) << "T(ms)"
              << std::setw(10) << "Length"
              << std::setw(8) << "Nodes"
              << std::setw(8) << "Turns"
              << std::setw(10) << "Smooth"
              << std::setw(12) << "MinObsDist\n";
    std::cout << std::string(105, '-') << "\n";

    for (size_t mi = 0; mi < maps.size(); ++mi) {
        GridMap map;
        if (!map.loadFromTxt(maps[mi])) {
            std::cerr << "Skip: " << maps[mi] << "\n";
            continue;
        }

        Costmap2D costmap;
        costmap.buildFromGridMap(map);
        costmap.inflateObstacles(1.0);

        auto obstacles = extractObstacles(map);
        auto start = starts[mi];
        auto goal = goals[mi];
        std::string map_name = maps[mi];

        std::vector<AblationRow> rows;

        // --- 1. A* baseline ---
        {
            AStarPlanner p(true);
            auto r = p.plan(map, start, goal);
            rows.push_back({
                "A*",
                r.success, r.planning_time_ms, r.path_length,
                r.expanded_nodes,
                computeTurningCount(r.path),
                computeSmoothnessScore(r.path),
                computeMinObstacleDistance(r.path, obstacles),
            });
        }

        // --- 2. Weighted A* ---
        {
            WeightedAStarPlanner p(1.5, true);
            auto r = p.plan(map, start, goal);
            rows.push_back({
                "Weighted A* (w=1.5)",
                r.success, r.planning_time_ms, r.path_length,
                r.expanded_nodes,
                computeTurningCount(r.path),
                computeSmoothnessScore(r.path),
                computeMinObstacleDistance(r.path, obstacles),
            });
        }

        // --- 3. A* + obstacle cost ---
        {
            ImprovedAStarPlanner p(1.0, 2.0, 0.0, true);
            p.setCostmap(&costmap);
            auto r = p.plan(map, start, goal);
            rows.push_back({
                "A* + obstacle",
                r.success, r.planning_time_ms, r.path_length,
                r.expanded_nodes,
                computeTurningCount(r.path),
                computeSmoothnessScore(r.path),
                computeMinObstacleDistance(r.path, obstacles),
            });
        }

        // --- 4. A* + turning cost ---
        {
            ImprovedAStarPlanner p(1.0, 0.0, 0.5, true);
            p.setCostmap(nullptr);
            auto r = p.plan(map, start, goal);
            rows.push_back({
                "A* + turning",
                r.success, r.planning_time_ms, r.path_length,
                r.expanded_nodes,
                computeTurningCount(r.path),
                computeSmoothnessScore(r.path),
                computeMinObstacleDistance(r.path, obstacles),
            });
        }

        // --- 5. Improved A* (full) ---
        {
            ImprovedAStarPlanner p(1.0, 2.0, 0.5, true);
            p.setCostmap(&costmap);
            auto r = p.plan(map, start, goal);
            rows.push_back({
                "Improved A* (full)",
                r.success, r.planning_time_ms, r.path_length,
                r.expanded_nodes,
                computeTurningCount(r.path),
                computeSmoothnessScore(r.path),
                computeMinObstacleDistance(r.path, obstacles),
            });
        }

        for (auto& row : rows) {
            std::cout << std::left << std::setw(22) << row.name
                      << std::setw(13) << map_name
                      << std::setw(6) << (row.success ? "OK" : "FAIL")
                      << std::setw(10) << row.time_ms
                      << std::setw(10) << row.path_len
                      << std::setw(8) << row.expanded
                      << std::setw(8) << row.turns
                      << std::setw(10) << row.smoothness
                      << std::setw(12) << row.min_obs_dist << "\n";

            csv << row.name << "," << map_name << ","
                << row.success << "," << row.time_ms << ","
                << row.path_len << "," << row.expanded << ","
                << row.turns << "," << row.smoothness << ","
                << row.min_obs_dist << "\n";
        }
        std::cout << "\n";
    }

    std::cout << "Results saved to results/benchmark/ablation.csv\n";
    return 0;
}
