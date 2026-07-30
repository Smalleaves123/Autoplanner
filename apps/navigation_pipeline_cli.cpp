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
        << "  --footprint NAME      point|circle|rectangle\n"
        << "  --robot-radius N      circular footprint radius\n"
        << "  --robot-length N      rectangular footprint length\n"
        << "  --robot-width N       rectangular footprint width\n"
        << "  --inflate             inflate planning map\n"
        << "  --smooth NAME         none|shortcut|curvature\n"
        << "  --smooth-iterations N smoothing iterations\n"
        << "  --smooth-max-curvature N curvature smoother limit\n"
        << "  --local-planner NAME  none|dwa\n"
        << "  --dwa-prediction-time N  DWA rollout horizon seconds\n"
        << "  --dwa-velocity-samples N DWA velocity samples\n"
        << "  --dwa-steering-samples N DWA steering samples\n"
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
    bool has_footprint = false;
    bool has_robot_radius = false;
    bool has_robot_length = false;
    bool has_robot_width = false;
    bool has_inflate = false;
    bool has_smoother = false;
    bool has_smoothing_iterations = false;
    bool has_smoothing_max_curvature = false;
    bool has_local_planner = false;
    bool has_dwa_prediction_time = false;
    bool has_dwa_velocity_samples = false;
    bool has_dwa_steering_samples = false;
    bool has_start = false;
    bool has_goal = false;
    bool has_max_steps = false;
    bool has_velocity = false;
    bool has_dt = false;

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
        } else if (arg == "--footprint" && i + 1 < argc) {
            scenario.pipeline.footprint = argv[++i];
            has_footprint = true;
        } else if (arg == "--robot-radius" && i + 1 < argc) {
            scenario.pipeline.robot_radius = std::stod(argv[++i]);
            has_robot_radius = true;
        } else if (arg == "--robot-length" && i + 1 < argc) {
            scenario.pipeline.robot_length = std::stod(argv[++i]);
            has_robot_length = true;
        } else if (arg == "--robot-width" && i + 1 < argc) {
            scenario.pipeline.robot_width = std::stod(argv[++i]);
            has_robot_width = true;
        } else if (arg == "--inflate") {
            scenario.pipeline.inflate_map = true;
            has_inflate = true;
        } else if (arg == "--smooth" && i + 1 < argc) {
            scenario.pipeline.smoother = argv[++i];
            has_smoother = true;
        } else if (arg == "--smooth-iterations" && i + 1 < argc) {
            scenario.pipeline.smoothing_iterations = std::stoi(argv[++i]);
            has_smoothing_iterations = true;
        } else if (arg == "--smooth-max-curvature" && i + 1 < argc) {
            scenario.pipeline.smoothing_max_curvature =
                std::stod(argv[++i]);
            has_smoothing_max_curvature = true;
        } else if (arg == "--local-planner" && i + 1 < argc) {
            scenario.pipeline.local_planner = argv[++i];
            has_local_planner = true;
        } else if (arg == "--dwa-prediction-time" && i + 1 < argc) {
            scenario.pipeline.dwa_options.prediction_time =
                std::stod(argv[++i]);
            has_dwa_prediction_time = true;
        } else if (arg == "--dwa-velocity-samples" && i + 1 < argc) {
            scenario.pipeline.dwa_options.velocity_samples =
                std::stoi(argv[++i]);
            has_dwa_velocity_samples = true;
        } else if (arg == "--dwa-steering-samples" && i + 1 < argc) {
            scenario.pipeline.dwa_options.steering_samples =
                std::stoi(argv[++i]);
            has_dwa_steering_samples = true;
        } else if (arg == "--start" && i + 2 < argc) {
            scenario.start = {std::stoi(argv[++i]), std::stoi(argv[++i])};
            has_start = true;
        } else if (arg == "--goal" && i + 2 < argc) {
            scenario.goal = {std::stoi(argv[++i]), std::stoi(argv[++i])};
            has_goal = true;
        } else if (arg == "--max-steps" && i + 1 < argc) {
            scenario.pipeline.max_steps =
                static_cast<std::size_t>(std::stoul(argv[++i]));
            has_max_steps = true;
        } else if (arg == "--velocity" && i + 1 < argc) {
            scenario.pipeline.trajectory_options.target_velocity =
                std::stod(argv[++i]);
            has_velocity = true;
        } else if (arg == "--dt" && i + 1 < argc) {
            scenario.pipeline.simulation_options.dt = std::stod(argv[++i]);
            has_dt = true;
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
        const auto pipeline_override = scenario.pipeline;
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
        if (has_footprint) scenario.pipeline.footprint = pipeline_override.footprint;
        if (has_robot_radius) scenario.pipeline.robot_radius =
            pipeline_override.robot_radius;
        if (has_robot_length) scenario.pipeline.robot_length =
            pipeline_override.robot_length;
        if (has_robot_width) scenario.pipeline.robot_width =
            pipeline_override.robot_width;
        if (has_inflate) scenario.pipeline.inflate_map =
            pipeline_override.inflate_map;
        if (has_smoother) scenario.pipeline.smoother =
            pipeline_override.smoother;
        if (has_smoothing_iterations) scenario.pipeline.smoothing_iterations =
            pipeline_override.smoothing_iterations;
        if (has_smoothing_max_curvature)
            scenario.pipeline.smoothing_max_curvature =
                pipeline_override.smoothing_max_curvature;
        if (has_local_planner) scenario.pipeline.local_planner =
            pipeline_override.local_planner;
        if (has_dwa_prediction_time)
            scenario.pipeline.dwa_options.prediction_time =
                pipeline_override.dwa_options.prediction_time;
        if (has_dwa_velocity_samples)
            scenario.pipeline.dwa_options.velocity_samples =
                pipeline_override.dwa_options.velocity_samples;
        if (has_dwa_steering_samples)
            scenario.pipeline.dwa_options.steering_samples =
                pipeline_override.dwa_options.steering_samples;
        if (has_max_steps) scenario.pipeline.max_steps =
            pipeline_override.max_steps;
        if (has_velocity) scenario.pipeline.trajectory_options.target_velocity =
            pipeline_override.trajectory_options.target_velocity;
        if (has_dt) scenario.pipeline.simulation_options.dt =
            pipeline_override.simulation_options.dt;
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
