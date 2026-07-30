#include "robotnav/scenario_config.h"

#include "autoplanner/io/config_loader.h"

namespace robotnav {

bool loadScenarioConfig(const std::string& file_path,
                        ScenarioConfig& scenario) {
    autoplanner::io::ConfigLoader loader;
    if (!loader.load(file_path)) return false;

    scenario.map_path = loader.getString("map.path", scenario.map_path);
    scenario.map_resolution = loader.getDouble(
        "map.resolution", scenario.map_resolution);
    scenario.start.x = loader.getInt("start.x", scenario.start.x);
    scenario.start.y = loader.getInt("start.y", scenario.start.y);
    scenario.goal.x = loader.getInt("goal.x", scenario.goal.x);
    scenario.goal.y = loader.getInt("goal.y", scenario.goal.y);

    auto& pipeline = scenario.pipeline;
    pipeline.planner = loader.getString("planner.name", pipeline.planner);
    pipeline.controller = loader.getString("controller.name", pipeline.controller);
    pipeline.footprint = loader.getString(
        "robot.footprint", pipeline.footprint);
    pipeline.smoother = loader.getString(
        "smoothing.name", pipeline.smoother);
    pipeline.local_planner = loader.getString(
        "local_planner.name", pipeline.local_planner);
    pipeline.robot_radius = loader.getDouble(
        "robot.radius", pipeline.robot_radius);
    pipeline.robot_length = loader.getDouble(
        "robot.length", pipeline.robot_length);
    pipeline.robot_width = loader.getDouble(
        "robot.width", pipeline.robot_width);
    pipeline.inflate_map = loader.getBool(
        "robot.inflate", pipeline.inflate_map);
    pipeline.smoothing_iterations = loader.getInt(
        "smoothing.iterations", pipeline.smoothing_iterations);
    const int max_steps = loader.getInt(
        "pipeline.max_steps", static_cast<int>(pipeline.max_steps));
    if (max_steps < 0) return false;
    pipeline.max_steps = static_cast<std::size_t>(max_steps);

    pipeline.planner_options.allow_diagonal = loader.getBool(
        "planner.allow_diagonal", pipeline.planner_options.allow_diagonal);
    pipeline.planner_options.heuristic_weight = loader.getDouble(
        "planner.heuristic_weight", pipeline.planner_options.heuristic_weight);
    pipeline.planner_options.weighted_astar_weight = loader.getDouble(
        "planner.weighted_astar_weight",
        pipeline.planner_options.weighted_astar_weight);
    pipeline.planner_options.obstacle_weight = loader.getDouble(
        "planner.obstacle_weight", pipeline.planner_options.obstacle_weight);
    pipeline.planner_options.turning_weight = loader.getDouble(
        "planner.turning_weight", pipeline.planner_options.turning_weight);
    pipeline.planner_options.max_iterations = loader.getInt(
        "planner.max_iterations", pipeline.planner_options.max_iterations);

    pipeline.trajectory_options.sample_spacing = loader.getDouble(
        "trajectory.sample_spacing", pipeline.trajectory_options.sample_spacing);
    pipeline.trajectory_options.target_velocity = loader.getDouble(
        "trajectory.target_velocity", pipeline.trajectory_options.target_velocity);
    pipeline.trajectory_options.max_velocity = loader.getDouble(
        "trajectory.max_velocity", pipeline.trajectory_options.max_velocity);
    pipeline.trajectory_options.max_acceleration = loader.getDouble(
        "trajectory.max_acceleration",
        pipeline.trajectory_options.max_acceleration);
    pipeline.trajectory_options.max_deceleration = loader.getDouble(
        "trajectory.max_deceleration",
        pipeline.trajectory_options.max_deceleration);
    pipeline.trajectory_options.max_lateral_acceleration = loader.getDouble(
        "trajectory.max_lateral_acceleration",
        pipeline.trajectory_options.max_lateral_acceleration);

    pipeline.simulation_options.dt = loader.getDouble(
        "simulation.dt", pipeline.simulation_options.dt);
    pipeline.simulation_options.wheelbase = loader.getDouble(
        "simulation.wheelbase", pipeline.simulation_options.wheelbase);
    pipeline.simulation_options.max_velocity = loader.getDouble(
        "simulation.max_velocity", pipeline.simulation_options.max_velocity);
    pipeline.simulation_options.max_acceleration = loader.getDouble(
        "simulation.max_acceleration",
        pipeline.simulation_options.max_acceleration);
    pipeline.simulation_options.max_deceleration = loader.getDouble(
        "simulation.max_deceleration",
        pipeline.simulation_options.max_deceleration);
    pipeline.simulation_options.max_steering = loader.getDouble(
        "simulation.max_steering", pipeline.simulation_options.max_steering);
    pipeline.simulation_options.max_steering_rate = loader.getDouble(
        "simulation.max_steering_rate",
        pipeline.simulation_options.max_steering_rate);

    pipeline.dwa_options.prediction_time = loader.getDouble(
        "local_planner.dwa.prediction_time",
        pipeline.dwa_options.prediction_time);
    pipeline.dwa_options.velocity_samples = loader.getInt(
        "local_planner.dwa.velocity_samples",
        pipeline.dwa_options.velocity_samples);
    pipeline.dwa_options.steering_samples = loader.getInt(
        "local_planner.dwa.steering_samples",
        pipeline.dwa_options.steering_samples);
    pipeline.dwa_options.trajectory_weight = loader.getDouble(
        "local_planner.dwa.trajectory_weight",
        pipeline.dwa_options.trajectory_weight);
    pipeline.dwa_options.heading_weight = loader.getDouble(
        "local_planner.dwa.heading_weight",
        pipeline.dwa_options.heading_weight);
    pipeline.dwa_options.speed_weight = loader.getDouble(
        "local_planner.dwa.speed_weight",
        pipeline.dwa_options.speed_weight);
    pipeline.dwa_options.command_weight = loader.getDouble(
        "local_planner.dwa.command_weight",
        pipeline.dwa_options.command_weight);

    pipeline.safety_options.goal_tolerance = loader.getDouble(
        "safety.goal_tolerance", pipeline.safety_options.goal_tolerance);
    pipeline.safety_options.max_cross_track_error = loader.getDouble(
        "safety.max_cross_track_error",
        pipeline.safety_options.max_cross_track_error);
    pipeline.safety_options.max_velocity = loader.getDouble(
        "safety.max_velocity", pipeline.simulation_options.max_velocity);
    pipeline.safety_options.max_steering = loader.getDouble(
        "safety.max_steering", pipeline.simulation_options.max_steering);
    pipeline.safety_options.enforce_collision = loader.getBool(
        "safety.enforce_collision", pipeline.safety_options.enforce_collision);
    return true;
}

}  // namespace robotnav
