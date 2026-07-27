#include <filesystem>
#include <exception>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "autoplanner/core/grid_map.h"
#include "robotnav/dynamic_navigation_pipeline.h"
#include "robotnav/scenario_config.h"

namespace {

void printHelp() {
    std::cout
        << "RobotNav dynamic navigation pipeline CLI\n"
        << "  --scenario PATH       scalar YAML/INI scenario config\n"
        << "  --map PATH            occupancy grid map\n"
        << "  --start X Y           start grid cell override\n"
        << "  --goal X Y            goal grid cell override\n"
        << "  --planner NAME        initial planner label\n"
        << "  --controller NAME     pid|pure_pursuit|stanley|mpc\n"
        << "  --no-diagonal         use 4-connected replanning\n"
        << "  --footprint NAME      point|circle|rectangle\n"
        << "  --robot-radius N      circular footprint radius\n"
        << "  --robot-length N      rectangular footprint length\n"
        << "  --robot-width N       rectangular footprint width\n"
        << "  --inflate             inflate planning map\n"
        << "  --smooth NAME         none|shortcut\n"
        << "  --velocity N          target trajectory velocity\n"
        << "  --frames N            dynamic obstacle update frames\n"
        << "  --steps-per-frame N   control cycles per frame\n"
        << "  --obstacle-ahead N    path samples before inserted obstacle\n"
        << "  --obstacle-margin N   safety cells around generated obstacle\n"
        << "  --max-auto-obstacles N  maximum generated obstacles\n"
        << "  --obstacle FRAME X Y  externally occupy a cell at frame\n"
        << "  --clear-obstacle FRAME X Y  externally clear a cell at frame\n"
        << "  --moving-obstacle START END X Y DX DY  moving occupied cell\n"
        << "  --no-auto-obstacles   disable automatic obstacle insertion\n"
        << "  --output-dir PATH     output directory\n";
}

}  // namespace

int main(int argc, char** argv) {
    robotnav::ScenarioConfig scenario;
    robotnav::DynamicPipelineConfig dynamic;
    std::string scenario_path;
    std::string output_dir = "results/dynamic_navigation_pipeline";
    bool has_map = false;
    bool has_start = false;
    bool has_goal = false;
    bool has_planner = false;
    bool has_controller = false;
    bool no_diagonal = false;
    bool has_footprint = false;
    bool has_radius = false;
    bool has_length = false;
    bool has_width = false;
    bool has_inflate = false;
    bool has_smoother = false;
    bool has_velocity = false;
    std::string map_override;
    autoplanner::Point2i start_override;
    autoplanner::Point2i goal_override;
    std::string planner_override;
    std::string controller_override;
    std::string footprint_override;
    double radius_override = 0.0;
    double length_override = 0.0;
    double width_override = 0.0;
    bool inflate_override = false;
    std::string smoother_override;
    double velocity_override = 0.0;
    std::vector<robotnav::DynamicObstacleUpdate> obstacle_updates;
    std::vector<robotnav::MovingObstacle> moving_obstacles;

    try {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--scenario" && i + 1 < argc) {
            scenario_path = argv[++i];
        } else if (arg == "--map" && i + 1 < argc) {
            map_override = argv[++i];
            has_map = true;
        } else if (arg == "--start" && i + 2 < argc) {
            start_override = {std::stoi(argv[++i]), std::stoi(argv[++i])};
            has_start = true;
        } else if (arg == "--goal" && i + 2 < argc) {
            goal_override = {std::stoi(argv[++i]), std::stoi(argv[++i])};
            has_goal = true;
        } else if (arg == "--planner" && i + 1 < argc) {
            planner_override = argv[++i];
            has_planner = true;
        } else if (arg == "--controller" && i + 1 < argc) {
            controller_override = argv[++i];
            has_controller = true;
        } else if (arg == "--no-diagonal") {
            no_diagonal = true;
        } else if (arg == "--footprint" && i + 1 < argc) {
            footprint_override = argv[++i];
            has_footprint = true;
        } else if (arg == "--robot-radius" && i + 1 < argc) {
            radius_override = std::stod(argv[++i]);
            has_radius = true;
        } else if (arg == "--robot-length" && i + 1 < argc) {
            length_override = std::stod(argv[++i]);
            has_length = true;
        } else if (arg == "--robot-width" && i + 1 < argc) {
            width_override = std::stod(argv[++i]);
            has_width = true;
        } else if (arg == "--inflate") {
            inflate_override = true;
            has_inflate = true;
        } else if (arg == "--smooth" && i + 1 < argc) {
            smoother_override = argv[++i];
            has_smoother = true;
        } else if (arg == "--velocity" && i + 1 < argc) {
            velocity_override = std::stod(argv[++i]);
            has_velocity = true;
        } else if (arg == "--frames" && i + 1 < argc) {
            dynamic.frames = static_cast<std::size_t>(std::stoul(argv[++i]));
        } else if (arg == "--steps-per-frame" && i + 1 < argc) {
            dynamic.steps_per_frame =
                static_cast<std::size_t>(std::stoul(argv[++i]));
        } else if (arg == "--obstacle-ahead" && i + 1 < argc) {
            dynamic.obstacle_insertion_ahead =
                static_cast<std::size_t>(std::stoul(argv[++i]));
        } else if (arg == "--obstacle-margin" && i + 1 < argc) {
            dynamic.auto_obstacle_margin_cells =
                static_cast<std::size_t>(std::stoul(argv[++i]));
        } else if (arg == "--max-auto-obstacles" && i + 1 < argc) {
            dynamic.max_auto_obstacles =
                static_cast<std::size_t>(std::stoul(argv[++i]));
        } else if ((arg == "--obstacle" || arg == "--clear-obstacle") &&
                   i + 3 < argc) {
            robotnav::DynamicObstacleUpdate update;
            update.frame = static_cast<std::size_t>(
                std::stoul(argv[++i]));
            update.cell = {std::stoi(argv[++i]), std::stoi(argv[++i])};
            update.occupied = arg == "--obstacle";
            obstacle_updates.push_back(update);
        } else if (arg == "--moving-obstacle" && i + 6 < argc) {
            robotnav::MovingObstacle obstacle;
            obstacle.start_frame = static_cast<std::size_t>(
                std::stoul(argv[++i]));
            obstacle.end_frame = static_cast<std::size_t>(
                std::stoul(argv[++i]));
            obstacle.start_cell = {
                std::stoi(argv[++i]), std::stoi(argv[++i])};
            obstacle.dx_per_frame = std::stoi(argv[++i]);
            obstacle.dy_per_frame = std::stoi(argv[++i]);
            moving_obstacles.push_back(obstacle);
        } else if (arg == "--no-auto-obstacles") {
            dynamic.auto_insert_obstacles = false;
        } else if (arg == "--output-dir" && i + 1 < argc) {
            output_dir = argv[++i];
        } else if (arg == "--help") {
            printHelp();
            return 0;
        } else {
            std::cerr << "Unknown argument: " << arg << "\n";
            printHelp();
            return 1;
        }
    }
    } catch (const std::exception& error) {
        std::cerr << "Invalid command-line argument: " << error.what()
                  << "\n";
        printHelp();
        return 1;
    }

    if (!scenario_path.empty()) {
        if (!robotnav::loadScenarioConfig(scenario_path, scenario)) {
            std::cerr << "Failed to load scenario: " << scenario_path << "\n";
            return 1;
        }
    }
    if (has_map) scenario.map_path = map_override;
    if (has_start) scenario.start = start_override;
    if (has_goal) scenario.goal = goal_override;
    if (has_planner) scenario.pipeline.planner = planner_override;
    if (has_controller) scenario.pipeline.controller = controller_override;
    if (has_footprint) scenario.pipeline.footprint = footprint_override;
    if (has_radius) scenario.pipeline.robot_radius = radius_override;
    if (has_length) scenario.pipeline.robot_length = length_override;
    if (has_width) scenario.pipeline.robot_width = width_override;
    if (has_inflate) scenario.pipeline.inflate_map = inflate_override;
    if (has_smoother) scenario.pipeline.smoother = smoother_override;
    if (has_velocity) {
        scenario.pipeline.trajectory_options.target_velocity = velocity_override;
    }

    dynamic.pipeline = scenario.pipeline;
    dynamic.obstacle_updates = std::move(obstacle_updates);
    dynamic.moving_obstacles = std::move(moving_obstacles);
    if (no_diagonal) dynamic.pipeline.planner_options.allow_diagonal = false;
    autoplanner::GridMap map;
    if (!map.loadFromTxt(scenario.map_path)) {
        std::cerr << "Failed to load map: " << scenario.map_path << "\n";
        return 1;
    }
    map.setResolution(scenario.map_resolution);

    const robotnav::DynamicNavigationPipeline pipeline;
    const auto result = pipeline.run(
        map, scenario.start, scenario.goal, dynamic);

    std::filesystem::create_directories(output_dir);
    const auto output_path = std::filesystem::path(output_dir);
    const auto trace_path = output_path / "trace.csv";
    const auto metrics_path = output_path / "metrics.json";
    if (!robotnav::saveDynamicTraceCsv(result, trace_path.string()) ||
        !robotnav::saveDynamicMetricsJson(result, metrics_path.string())) {
        std::cerr << "Failed to write dynamic pipeline outputs\n";
        return 1;
    }

    std::cout << "Status: " << robotnav::toString(result.metrics.status) << "\n"
              << "Message: " << result.message << "\n"
              << "Trace: " << trace_path << "\n"
              << "Metrics: " << metrics_path << "\n";
    return result.metrics.status == robotnav::StatusCode::Success ? 0 : 3;
}
