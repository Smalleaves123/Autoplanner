#include <filesystem>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>

#include "autoplanner/collision/grid_collision_checker.h"
#include "autoplanner/collision/footprint_collision_checker.h"
#include "autoplanner/core/grid_map.h"
#include "autoplanner/core/path.h"
#include "autoplanner/core/planner_factory.h"
#include "autoplanner/io/config_loader.h"
#include "autoplanner/costmap/costmap_2d.h"
#include "autoplanner/costmap/obstacle_inflation.h"
#include "autoplanner/smoothing/shortcut_smoother.h"

using namespace autoplanner;

namespace {

std::vector<Pose2d> makePoses(const Path2d& path) {
    std::vector<Pose2d> poses;
    poses.reserve(path.size());
    for (std::size_t i = 0; i < path.size(); ++i) {
        std::size_t next = std::min(i + 1, path.size() - 1);
        std::size_t previous = i == 0 ? i : i - 1;
        const double dx = path[next].x - path[previous].x;
        const double dy = path[next].y - path[previous].y;
        poses.push_back({path[i].x, path[i].y, std::atan2(dy, dx)});
    }
    return poses;
}

}  // namespace

int main(int argc, char** argv) {
    std::string planner_name = "astar";
    std::string map_path = "data/maps/simple_50x50.txt";
    std::string output_dir = "results";
    std::string config_path;
    Point2i start{1, 1}, goal{48, 48};
    PlannerFactoryOptions planner_options;
    double robot_radius = 1.0;
    double map_resolution = 1.0;
    double inflation_radius = 0.0;
    double robot_length = 0.0;
    double robot_width = 0.0;
    std::string footprint_name = "point";
    bool inflate_map = false;
    std::string smoother_name = "none";
    int smoothing_iterations = 100;

    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--config" && i + 1 < argc) {
            config_path = argv[i + 1];
            break;
        }
    }

    if (!config_path.empty()) {
        io::ConfigLoader config;
        if (!config.load(config_path)) {
            std::cerr << "Failed to load config: " << config_path << "\n";
            return 1;
        }

        planner_name = config.getString("planner", planner_name);
        map_path = config.getString("map.path", map_path);
        map_resolution = config.getDouble("map.resolution", map_resolution);
        robot_radius = config.getDouble("robot.radius", robot_radius);
        footprint_name = config.getString("robot.footprint", footprint_name);
        robot_length = config.getDouble("robot.length", robot_length);
        robot_width = config.getDouble("robot.width", robot_width);
        inflation_radius = config.getDouble(
            "robot.inflation_radius", inflation_radius);
        inflate_map = config.getBool("robot.inflate", inflate_map) ||
                      inflation_radius > 0.0;
        planner_options.allow_diagonal = config.getBool(
            "astar.allow_diagonal", planner_options.allow_diagonal);
        planner_options.heuristic_weight = config.getDouble(
            "astar.heuristic_weight", planner_options.heuristic_weight);
        planner_options.weighted_astar_weight = config.getDouble(
            "astar.heuristic_weight", planner_options.weighted_astar_weight);
        planner_options.obstacle_weight = config.getDouble(
            "astar.obstacle_weight", planner_options.obstacle_weight);
        planner_options.turning_weight = config.getDouble(
            "astar.turning_weight", planner_options.turning_weight);
        planner_options.step_size = config.getDouble(
            "rrt.step_size", planner_options.step_size);
        planner_options.max_iterations = config.getInt(
            "rrt.max_iterations", planner_options.max_iterations);
        planner_options.goal_sample_rate = config.getDouble(
            "rrt.goal_sample_rate", planner_options.goal_sample_rate);
        planner_options.goal_tolerance = config.getDouble(
            "rrt.goal_tolerance", planner_options.goal_tolerance);
        planner_options.rewire_radius = config.getDouble(
            "rrt_star.rewire_radius", planner_options.rewire_radius);
        planner_options.turning_radius = config.getDouble(
            "hybrid_astar.turning_radius", planner_options.turning_radius);
        planner_options.angle_bins = config.getInt(
            "hybrid_astar.angle_bins", planner_options.angle_bins);
        output_dir = config.getString("output.directory", output_dir);
    }

    // Explicit CLI arguments override the optional configuration file.
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--config" && i+1 < argc) ++i;
        else if (arg == "--planner" && i+1 < argc) planner_name = argv[++i];
        else if (arg == "--map" && i+1 < argc) map_path = argv[++i];
        else if (arg == "--output" && i+1 < argc) output_dir = argv[++i];
        else if (arg == "--start" && i+2 < argc) {
            start = {std::stoi(argv[++i]), std::stoi(argv[++i])}; }
        else if (arg == "--goal" && i+2 < argc) {
            goal = {std::stoi(argv[++i]), std::stoi(argv[++i])}; }
        else if (arg == "--weight" && i+1 < argc)
            planner_options.weighted_astar_weight = std::stod(argv[++i]);
        else if (arg == "--step-size" && i+1 < argc)
            planner_options.step_size = std::stod(argv[++i]);
        else if (arg == "--max-iterations" && i+1 < argc)
            planner_options.max_iterations = std::stoi(argv[++i]);
        else if (arg == "--goal-sample-rate" && i+1 < argc)
            planner_options.goal_sample_rate = std::stod(argv[++i]);
        else if (arg == "--goal-tolerance" && i+1 < argc)
            planner_options.goal_tolerance = std::stod(argv[++i]);
        else if (arg == "--rewire-radius" && i+1 < argc)
            planner_options.rewire_radius = std::stod(argv[++i]);
        else if (arg == "--turning-radius" && i+1 < argc)
            planner_options.turning_radius = std::stod(argv[++i]);
        else if (arg == "--angle-bins" && i+1 < argc)
            planner_options.angle_bins = std::stoi(argv[++i]);
        else if (arg == "--robot-radius" && i+1 < argc) robot_radius = std::stod(argv[++i]);
        else if (arg == "--resolution" && i+1 < argc)
            map_resolution = std::stod(argv[++i]);
        else if (arg == "--footprint" && i+1 < argc)
            footprint_name = argv[++i];
        else if (arg == "--robot-length" && i+1 < argc)
            robot_length = std::stod(argv[++i]);
        else if (arg == "--robot-width" && i+1 < argc)
            robot_width = std::stod(argv[++i]);
        else if (arg == "--inflate-radius" && i+1 < argc) {
            inflation_radius = std::stod(argv[++i]);
            inflate_map = true;
        }
        else if (arg == "--inflate") inflate_map = true;
        else if (arg == "--no-diagonal") planner_options.allow_diagonal = false;
        else if (arg == "--smooth" && i+1 < argc) smoother_name = argv[++i];
        else if (arg == "--smooth-iterations" && i+1 < argc)
            smoothing_iterations = std::stoi(argv[++i]);
        else if (arg == "--help") {
            std::cout << "AutoPlanner CLI\n"
                      << "  --config <path>    YAML or key=value config\n"
                      << "  --planner <name>   astar|dijkstra|weighted_astar|improved_astar|jps|dstar_lite|rrt|rrt_star|informed_rrt_star|bi_rrt|hybrid_astar\n"
                      << "  --map <path>       grid map file\n"
                      << "  --start <x> <y>    start coordinates\n"
                      << "  --goal <x> <y>     goal coordinates\n"
                      << "  --resolution <m>   map metres per cell\n"
                      << "  --footprint <type> point|circle|rectangle\n"
                      << "  --robot-radius <m> circle footprint radius\n"
                      << "  --robot-length <m> rectangle length\n"
                      << "  --robot-width <m>  rectangle width\n"
                      << "  --inflate          inflate planning map for footprint\n"
                      << "  --inflate-radius <m> explicit circular inflation radius\n"
                      << "  --no-diagonal      use 4-connected graph search\n"
                      << "  --step-size <n>    sampling/hybrid step size\n"
                      << "  --max-iterations <n>  sampling iteration limit\n"
                      << "  --smooth <name>    none|shortcut (default: none)\n"
                      << "  --smooth-iterations <n>  smoothing iterations\n"
                      << "  --output <dir>     output directory\n";
            return 0;
        }
    }

    GridMap map;
    if (!map.loadFromTxt(map_path)) {
        std::cerr << "Failed to load map: " << map_path << "\n"; return 1;
    }
    map.setResolution(map_resolution);

    RobotFootprint footprint;
    double circumscribed_radius = robot_radius;
    if (footprint_name == "circle") {
        footprint = RobotFootprint::circle(robot_radius);
    } else if (footprint_name == "rectangle") {
        footprint = RobotFootprint::rectangle(robot_length, robot_width);
        circumscribed_radius = std::hypot(0.5 * robot_length,
                                          0.5 * robot_width);
    } else if (footprint_name != "point") {
        std::cerr << "Unknown footprint: " << footprint_name << "\n";
        return 1;
    }

    GridMap planning_map = map;
    if (inflate_map) {
        const double radius = inflation_radius > 0.0
            ? inflation_radius : circumscribed_radius;
        planning_map.inflateObstacles(radius);
    }

    Costmap2D costmap;
    if (planner_name == "improved_astar") {
        costmap.buildFromGridMap(map);
        costmap.inflateObstacles(robot_radius);
    }

    std::unique_ptr<PlannerBase> planner = createPlanner(
        planner_name, planner_options,
        planner_name == "improved_astar" ? &costmap : nullptr);
    if (!planner) {
        std::cerr << "Unknown planner: " << planner_name << "\n"; return 1;
    }

    std::cout << "Map: " << map.width() << "x" << map.height()
              << "  Planner: " << planner->name()
              << "  Start: (" << start.x << "," << start.y
              << ")  Goal: (" << goal.x << "," << goal.y << ")\n";

    auto result = planner->plan(planning_map, start, goal);

    std::unique_ptr<CollisionChecker> collision_checker;
    if (footprint_name == "point") {
        collision_checker = std::make_unique<GridCollisionChecker>(map);
    } else {
        collision_checker = std::make_unique<FootprintCollisionChecker>(
            map, footprint);
    }

    if (result.success && smoother_name != "none") {
        if (smoother_name != "shortcut") {
            std::cerr << "Unknown smoother: " << smoother_name << "\n";
            return 1;
        }

        std::unique_ptr<CollisionChecker> smoothing_checker;
        if (footprint_name == "rectangle") {
            smoothing_checker = std::make_unique<FootprintCollisionChecker>(
                map, RobotFootprint::circle(circumscribed_radius));
        }

        CollisionChecker* checker = smoothing_checker
            ? smoothing_checker.get() : collision_checker.get();
        ShortcutSmoother smoother(*checker, smoothing_iterations);
        result.path = smoother.smooth(result.path);
        result.path_length = computePathLength(result.path);
        result.message = "Path found and smoothed.";
    }

    if (result.success) {
        const bool collision_free = footprint_name == "rectangle"
            ? collision_checker->isPosePathValid(makePoses(result.path))
            : collision_checker->isPathValid(result.path);
        if (!collision_free) {
            result.success = false;
            result.path.clear();
            result.path_length = 0.0;
            result.message = "Path is not collision-free for the selected footprint.";
        }
    }

    std::cout << (result.success ? "SUCCESS" : "FAIL")
              << "  Time: " << result.planning_time_ms << " ms"
              << "  Length: " << result.path_length
              << "  Nodes: " << result.expanded_nodes
              << "  Points: " << result.path.size() << "\n";

    std::filesystem::create_directories(output_dir);
    if (result.success)
        savePathCsv(result.path, output_dir + "/path.csv");
    saveMetricsJson(result, output_dir + "/metrics.json");
    std::cout << "Output: " << output_dir << "/\n";
    return result.success ? 0 : 2;
}
