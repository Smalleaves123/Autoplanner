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
        << "  --smooth NAME         none|shortcut|curvature\n"
        << "  --smooth-max-curvature N curvature smoother limit\n"
        << "  --local-planner NAME  none|dwa|mppi\n"
        << "  --dwa-prediction-time N  DWA rollout horizon seconds\n"
        << "  --dwa-velocity-samples N DWA velocity samples\n"
        << "  --dwa-steering-samples N DWA steering samples\n"
        << "  --dwa-dynamic-obstacle-margin N  dynamic obstacle margin\n"
        << "  --dwa-dynamic-collision-samples N dynamic rollout samples\n"
        << "  --mppi-prediction-time N MPPI rollout horizon seconds\n"
        << "  --mppi-horizon N       MPPI control horizon\n"
        << "  --mppi-rollouts N      MPPI sampled rollouts\n"
        << "  --mppi-temperature N   MPPI soft-min temperature\n"
        << "  --mppi-velocity-noise N MPPI velocity noise\n"
        << "  --mppi-steering-noise N MPPI steering noise\n"
        << "  --prediction-risk-weight N Space-time prediction risk weight\n"
        << "  --prediction-risk-clearance N risk clearance in map cells\n"
        << "  --velocity N          target trajectory velocity\n"
        << "  --prediction-horizon N  space-time planner horizon frames\n"
        << "  --frames N            dynamic obstacle update frames\n"
        << "  --steps-per-frame N   control cycles per frame\n"
        << "  --obstacle-ahead N    path samples before inserted obstacle\n"
        << "  --obstacle-margin N   safety cells around generated obstacle\n"
        << "  --max-auto-obstacles N  maximum generated obstacles\n"
        << "  --obstacle FRAME X Y  externally occupy a cell at frame\n"
        << "  --clear-obstacle FRAME X Y  externally clear a cell at frame\n"
        << "  --moving-obstacle START END X Y DX DY  moving occupied cell\n"
        << "  --moving-obstacle-radius N  footprint radius for moving obstacles\n"
        << "  --moving-obstacle-uncertainty-growth N radius growth per frame\n"
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
    bool has_smoothing_max_curvature = false;
    bool has_local_planner = false;
    bool has_dwa_prediction_time = false;
    bool has_dwa_velocity_samples = false;
    bool has_dwa_steering_samples = false;
    bool has_dwa_dynamic_obstacle_margin = false;
    bool has_dwa_dynamic_collision_samples = false;
    bool has_mppi_prediction_time = false;
    bool has_mppi_horizon = false;
    bool has_mppi_rollouts = false;
    bool has_mppi_temperature = false;
    bool has_mppi_velocity_noise = false;
    bool has_mppi_steering_noise = false;
    bool has_prediction_risk_weight = false;
    bool has_prediction_risk_clearance = false;
    bool has_moving_obstacle_radius = false;
    bool has_moving_obstacle_uncertainty_growth = false;
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
    double smoothing_max_curvature_override = 0.0;
    std::string local_planner_override;
    double dwa_prediction_time_override = 0.0;
    int dwa_velocity_samples_override = 0;
    int dwa_steering_samples_override = 0;
    double dwa_dynamic_obstacle_margin_override = 0.0;
    int dwa_dynamic_collision_samples_override = 0;
    double mppi_prediction_time_override = 0.0;
    int mppi_horizon_override = 0;
    int mppi_rollouts_override = 0;
    double mppi_temperature_override = 0.0;
    double mppi_velocity_noise_override = 0.0;
    double mppi_steering_noise_override = 0.0;
    double prediction_risk_weight_override = 0.0;
    double prediction_risk_clearance_override = 0.0;
    double moving_obstacle_radius = 0.0;
    double moving_obstacle_uncertainty_growth = 0.0;
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
        } else if (arg == "--smooth-max-curvature" && i + 1 < argc) {
            smoothing_max_curvature_override = std::stod(argv[++i]);
            has_smoothing_max_curvature = true;
        } else if (arg == "--local-planner" && i + 1 < argc) {
            local_planner_override = argv[++i];
            has_local_planner = true;
        } else if (arg == "--dwa-prediction-time" && i + 1 < argc) {
            dwa_prediction_time_override = std::stod(argv[++i]);
            has_dwa_prediction_time = true;
        } else if (arg == "--dwa-velocity-samples" && i + 1 < argc) {
            dwa_velocity_samples_override = std::stoi(argv[++i]);
            has_dwa_velocity_samples = true;
        } else if (arg == "--dwa-steering-samples" && i + 1 < argc) {
            dwa_steering_samples_override = std::stoi(argv[++i]);
            has_dwa_steering_samples = true;
        } else if (arg == "--dwa-dynamic-obstacle-margin" && i + 1 < argc) {
            dwa_dynamic_obstacle_margin_override = std::stod(argv[++i]);
            has_dwa_dynamic_obstacle_margin = true;
        } else if (arg == "--dwa-dynamic-collision-samples" && i + 1 < argc) {
            dwa_dynamic_collision_samples_override = std::stoi(argv[++i]);
            has_dwa_dynamic_collision_samples = true;
        } else if (arg == "--mppi-prediction-time" && i + 1 < argc) {
            mppi_prediction_time_override = std::stod(argv[++i]);
            has_mppi_prediction_time = true;
        } else if (arg == "--mppi-horizon" && i + 1 < argc) {
            mppi_horizon_override = std::stoi(argv[++i]);
            has_mppi_horizon = true;
        } else if (arg == "--mppi-rollouts" && i + 1 < argc) {
            mppi_rollouts_override = std::stoi(argv[++i]);
            has_mppi_rollouts = true;
        } else if (arg == "--mppi-temperature" && i + 1 < argc) {
            mppi_temperature_override = std::stod(argv[++i]);
            has_mppi_temperature = true;
        } else if (arg == "--mppi-velocity-noise" && i + 1 < argc) {
            mppi_velocity_noise_override = std::stod(argv[++i]);
            has_mppi_velocity_noise = true;
        } else if (arg == "--mppi-steering-noise" && i + 1 < argc) {
            mppi_steering_noise_override = std::stod(argv[++i]);
            has_mppi_steering_noise = true;
        } else if (arg == "--prediction-risk-weight" && i + 1 < argc) {
            prediction_risk_weight_override = std::stod(argv[++i]);
            has_prediction_risk_weight = true;
        } else if (arg == "--prediction-risk-clearance" && i + 1 < argc) {
            prediction_risk_clearance_override = std::stod(argv[++i]);
            has_prediction_risk_clearance = true;
        } else if (arg == "--velocity" && i + 1 < argc) {
            velocity_override = std::stod(argv[++i]);
            has_velocity = true;
        } else if (arg == "--prediction-horizon" && i + 1 < argc) {
            dynamic.prediction_horizon_frames =
                static_cast<std::size_t>(std::stoul(argv[++i]));
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
        } else if (arg == "--moving-obstacle-radius" && i + 1 < argc) {
            moving_obstacle_radius = std::stod(argv[++i]);
            has_moving_obstacle_radius = true;
        } else if (arg == "--moving-obstacle-uncertainty-growth" &&
                   i + 1 < argc) {
            moving_obstacle_uncertainty_growth = std::stod(argv[++i]);
            has_moving_obstacle_uncertainty_growth = true;
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
    if (has_smoothing_max_curvature)
        scenario.pipeline.smoothing_max_curvature =
            smoothing_max_curvature_override;
    if (has_local_planner) scenario.pipeline.local_planner =
        local_planner_override;
    if (has_dwa_prediction_time)
        scenario.pipeline.dwa_options.prediction_time =
            dwa_prediction_time_override;
    if (has_dwa_velocity_samples)
        scenario.pipeline.dwa_options.velocity_samples =
            dwa_velocity_samples_override;
    if (has_dwa_steering_samples)
        scenario.pipeline.dwa_options.steering_samples =
            dwa_steering_samples_override;
    if (has_dwa_dynamic_obstacle_margin)
        scenario.pipeline.dwa_options.dynamic_obstacle_margin =
            dwa_dynamic_obstacle_margin_override;
    if (has_dwa_dynamic_collision_samples)
        scenario.pipeline.dwa_options.dynamic_collision_samples =
            dwa_dynamic_collision_samples_override;
    if (has_mppi_prediction_time)
        scenario.pipeline.mppi_options.prediction_time =
            mppi_prediction_time_override;
    if (has_mppi_horizon)
        scenario.pipeline.mppi_options.horizon = mppi_horizon_override;
    if (has_mppi_rollouts)
        scenario.pipeline.mppi_options.rollouts = mppi_rollouts_override;
    if (has_mppi_temperature)
        scenario.pipeline.mppi_options.temperature =
            mppi_temperature_override;
    if (has_mppi_velocity_noise)
        scenario.pipeline.mppi_options.velocity_noise =
            mppi_velocity_noise_override;
    if (has_mppi_steering_noise)
        scenario.pipeline.mppi_options.steering_noise =
            mppi_steering_noise_override;
    if (has_prediction_risk_weight)
        scenario.pipeline.dynamic_prediction_risk_weight =
            prediction_risk_weight_override;
    if (has_prediction_risk_clearance)
        scenario.pipeline.dynamic_prediction_risk_clearance =
            prediction_risk_clearance_override;
    if (has_velocity) {
        scenario.pipeline.trajectory_options.target_velocity = velocity_override;
    }

    dynamic.pipeline = scenario.pipeline;
    dynamic.obstacle_updates = std::move(obstacle_updates);
    for (auto& obstacle : moving_obstacles) {
        if (has_moving_obstacle_radius) {
            obstacle.radius = moving_obstacle_radius;
        }
        if (has_moving_obstacle_uncertainty_growth) {
            obstacle.uncertainty_growth_per_frame =
                moving_obstacle_uncertainty_growth;
        }
    }
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
