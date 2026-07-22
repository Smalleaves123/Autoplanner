#include <filesystem>
#include <iostream>
#include <string>

#include "autoplanner/core/grid_map.h"
#include "robotnav/navigation_pipeline.h"
#include "robotnav/navigation_trace.h"
#include "robotnav/scenario_config.h"

namespace {

void printHelp() {
    std::cout
        << "RobotNav navigation pipeline CLI\n"
        << "  --scenario PATH       scalar YAML/INI scenario config\n"
        << "  --map PATH            occupancy grid map\n"
        << "  --planner NAME        planner name (default astar)\n"
        << "  --controller NAME     pid|pure_pursuit|stanley|mpc\n"
        << "  --start X Y           start cell\n"
        << "  --goal X Y            goal cell\n"
        << "  --max-steps N         controller execution limit\n"
        << "  --velocity N           target trajectory velocity\n"
        << "  --dt N                simulation timestep\n"
        << "  --output-dir PATH     output directory\n";
}

}  // namespace

int main(int argc, char** argv) {
    robotnav::ScenarioConfig scenario;
    std::string scenario_path;
    std::string output_dir = "results/navigation_pipeline";
    bool has_map = false;
    bool has_planner = false;
    bool has_controller = false;
    bool has_start = false;
    bool has_goal = false;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--scenario" && i + 1 < argc) {
            scenario_path = argv[++i];
        } else if (arg == "--map" && i + 1 < argc) {
            scenario.map_path = argv[++i];
            has_map = true;
        } else if (arg == "--planner" && i + 1 < argc) {
            scenario.pipeline.planner = argv[++i];
            has_planner = true;
        } else if (arg == "--controller" && i + 1 < argc) {
            scenario.pipeline.controller = argv[++i];
            has_controller = true;
        } else if (arg == "--start" && i + 2 < argc) {
            scenario.start = {std::stoi(argv[++i]), std::stoi(argv[++i])};
            has_start = true;
        } else if (arg == "--goal" && i + 2 < argc) {
            scenario.goal = {std::stoi(argv[++i]), std::stoi(argv[++i])};
            has_goal = true;
        } else if (arg == "--max-steps" && i + 1 < argc) {
            scenario.pipeline.max_steps =
                static_cast<std::size_t>(std::stoul(argv[++i]));
        } else if (arg == "--velocity" && i + 1 < argc) {
            scenario.pipeline.trajectory_options.target_velocity =
                std::stod(argv[++i]);
        } else if (arg == "--dt" && i + 1 < argc) {
            scenario.pipeline.simulation_options.dt = std::stod(argv[++i]);
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

    if (!scenario_path.empty()) {
        const std::string map_override = scenario.map_path;
        const std::string planner_override = scenario.pipeline.planner;
        const std::string controller_override = scenario.pipeline.controller;
        const auto start_override = scenario.start;
        const auto goal_override = scenario.goal;
        robotnav::ScenarioConfig loaded;
        if (!robotnav::loadScenarioConfig(scenario_path, loaded)) {
            std::cerr << "Failed to load scenario: " << scenario_path << "\n";
            return 1;
        }
        scenario = loaded;
        if (has_map) scenario.map_path = map_override;
        if (has_planner) scenario.pipeline.planner = planner_override;
        if (has_controller) scenario.pipeline.controller = controller_override;
        if (has_start) scenario.start = start_override;
        if (has_goal) scenario.goal = goal_override;
    }

    autoplanner::GridMap map;
    if (!map.loadFromTxt(scenario.map_path)) {
        std::cerr << "Failed to load map: " << scenario.map_path << "\n";
        return 1;
    }
    map.setResolution(scenario.map_resolution);

    const robotnav::NavigationPipeline pipeline;
    const robotnav::PipelineResult result = pipeline.run(
        map, scenario.start, scenario.goal, scenario.pipeline);

    std::filesystem::create_directories(output_dir);
    const std::filesystem::path output_path(output_dir);
    const auto trace_path = output_path / "trace.csv";
    const auto metrics_path = output_path / "metrics.json";
    if (!robotnav::saveNavigationTraceCsv(result.trace, trace_path.string()) ||
        !robotnav::savePipelineMetricsJson(result, metrics_path.string())) {
        std::cerr << "Failed to write pipeline outputs to: " << output_dir << "\n";
        return 1;
    }

    std::cout << "Status: " << robotnav::toString(result.metrics.status) << "\n"
              << "Message: " << result.message << "\n"
              << "Trace: " << trace_path << "\n"
              << "Metrics: " << metrics_path << "\n";
    return result.metrics.status == robotnav::StatusCode::Success ? 0 : 2;
}
