#include <filesystem>
#include <iostream>
#include <memory>
#include <string>

#include "autoplanner/core/grid_map.h"
#include "autoplanner/core/path.h"
#include "autoplanner/core/planner_base.h"
#include "autoplanner/planners/graph_search/astar.h"
#include "autoplanner/planners/graph_search/dijkstra.h"
#include "autoplanner/planners/graph_search/weighted_astar.h"

namespace {

struct CliOptions {
    std::string planner = "astar";
    std::string map_path = "data/maps/simple_50x50.txt";
    std::string output_dir = "results/mvp_astar";
    autoplanner::Point2i start{1, 1};
    autoplanner::Point2i goal{48, 48};
    double weight = 1.5;
};

bool parseInt(const char* text, int& value) {
    try {
        value = std::stoi(text);
        return true;
    } catch (...) {
        return false;
    }
}

bool parseDouble(const char* text, double& value) {
    try {
        value = std::stod(text);
        return true;
    } catch (...) {
        return false;
    }
}

CliOptions parseArgs(int argc, char** argv) {
    CliOptions options;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];

        if (arg == "--planner" && i + 1 < argc) {
            options.planner = argv[++i];
        } else if (arg == "--map" && i + 1 < argc) {
            options.map_path = argv[++i];
        } else if (arg == "--output" && i + 1 < argc) {
            options.output_dir = argv[++i];
        } else if (arg == "--start" && i + 2 < argc) {
            int x = 0;
            int y = 0;
            if (parseInt(argv[++i], x) && parseInt(argv[++i], y)) {
                options.start = autoplanner::Point2i{x, y};
            }
        } else if (arg == "--goal" && i + 2 < argc) {
            int x = 0;
            int y = 0;
            if (parseInt(argv[++i], x) && parseInt(argv[++i], y)) {
                options.goal = autoplanner::Point2i{x, y};
            }
        } else if (arg == "--weight" && i + 1 < argc) {
            parseDouble(argv[++i], options.weight);
        } else if (arg == "--help") {
            std::cout
                << "Usage:\n"
                << "  run_single_planner --planner <name> --map <path> "
                << "--start <x> <y> --goal <x> <y> --output <dir>\n"
                << "\n"
                << "  --planner  astar | dijkstra | weighted_astar (default: astar)\n"
                << "  --weight   heuristic multiplier for weighted_astar (default: 1.5)\n"
                << "  --map      path to .txt occupancy grid\n"
                << "  --output   directory for path.csv and metrics.json\n\n";
            std::exit(0);
        }
    }

    return options;
}

std::unique_ptr<autoplanner::PlannerBase> createPlanner(const CliOptions& options) {
    if (options.planner == "astar") {
        return std::make_unique<autoplanner::AStarPlanner>(true);
    }
    if (options.planner == "dijkstra") {
        return std::make_unique<autoplanner::DijkstraPlanner>(true);
    }
    if (options.planner == "weighted_astar") {
        return std::make_unique<autoplanner::WeightedAStarPlanner>(options.weight, true);
    }

    return nullptr;
}

}  // namespace

int main(int argc, char** argv) {
    const CliOptions options = parseArgs(argc, argv);

    autoplanner::GridMap map;
    if (!map.loadFromTxt(options.map_path)) {
        std::cerr << "Failed to load map: " << options.map_path << "\n";
        return 1;
    }

    std::unique_ptr<autoplanner::PlannerBase> planner = createPlanner(options);
    if (!planner) {
        std::cerr << "Unknown planner: " << options.planner << "\n";
        std::cerr << "Supported planners: astar, dijkstra, weighted_astar\n";
        return 1;
    }

    std::cout << "Map loaded: " << map.width() << " x " << map.height() << "\n";
    std::cout << "Planner:   " << planner->name() << "\n";
    std::cout << "Start:     (" << options.start.x << ", " << options.start.y << ")\n";
    std::cout << "Goal:      (" << options.goal.x << ", " << options.goal.y << ")\n";

    if (options.planner == "weighted_astar") {
        std::cout << "Weight:    " << options.weight << "\n";
    }

    autoplanner::PlannerResult result =
        planner->plan(map, options.start, options.goal);

    std::cout << "\nSuccess:        " << (result.success ? "true" : "false") << "\n";
    std::cout << "Message:        " << result.message << "\n";
    std::cout << "Path length:    " << result.path_length << "\n";
    std::cout << "Path points:    " << result.path.size() << "\n";
    std::cout << "Expanded nodes: " << result.expanded_nodes << "\n";
    std::cout << "Planning time:  " << result.planning_time_ms << " ms\n";

    std::filesystem::create_directories(options.output_dir);

    const std::string path_file = options.output_dir + "/path.csv";
    const std::string metrics_file = options.output_dir + "/metrics.json";

    if (result.success) {
        if (!autoplanner::savePathCsv(result.path, path_file)) {
            std::cerr << "Failed to save path file: " << path_file << "\n";
            return 1;
        }
        std::cout << "Saved path:    " << path_file << "\n";
    }

    if (!autoplanner::saveMetricsJson(result, metrics_file)) {
        std::cerr << "Failed to save metrics file: " << metrics_file << "\n";
        return 1;
    }

    std::cout << "Saved metrics: " << metrics_file << "\n";
    return result.success ? 0 : 2;
}
