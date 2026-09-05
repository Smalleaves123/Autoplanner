#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "autoplanner/core/grid_map.h"
#include "robotnav/dynamic_navigation_pipeline.h"

namespace py = pybind11;

namespace {

struct MovingObstacleSpec {
    std::size_t start_frame = 0;
    std::size_t end_frame = 0;
    int x = -1;
    int y = -1;
    int dx = 0;
    int dy = 0;
    double radius = 0.0;
    double uncertainty_growth = 0.0;
    double acceleration_x = 0.0;
    double acceleration_y = 0.0;
    double covariance_xx = 0.0;
    double covariance_xy = 0.0;
    double covariance_yy = 0.0;
    double covariance_growth_xx = 0.0;
    double covariance_growth_xy = 0.0;
    double covariance_growth_yy = 0.0;
    double confidence_scale = 2.0;
};

struct ObstacleUpdateSpec {
    std::size_t frame = 0;
    int x = -1;
    int y = -1;
    bool occupied = true;
};

robotnav::MovingObstacle toMovingObstacle(const MovingObstacleSpec& spec) {
    robotnav::MovingObstacle obstacle;
    obstacle.start_frame = spec.start_frame;
    obstacle.end_frame = spec.end_frame;
    obstacle.start_cell = {spec.x, spec.y};
    obstacle.dx_per_frame = spec.dx;
    obstacle.dy_per_frame = spec.dy;
    obstacle.radius = spec.radius;
    obstacle.uncertainty_growth_per_frame = spec.uncertainty_growth;
    obstacle.acceleration_x_per_frame2 = spec.acceleration_x;
    obstacle.acceleration_y_per_frame2 = spec.acceleration_y;
    obstacle.covariance_xx = spec.covariance_xx;
    obstacle.covariance_xy = spec.covariance_xy;
    obstacle.covariance_yy = spec.covariance_yy;
    obstacle.covariance_growth_xx_per_frame = spec.covariance_growth_xx;
    obstacle.covariance_growth_xy_per_frame = spec.covariance_growth_xy;
    obstacle.covariance_growth_yy_per_frame = spec.covariance_growth_yy;
    obstacle.covariance_confidence_scale = spec.confidence_scale;
    return obstacle;
}

py::dict runDynamic(
    const std::string& map_path,
    int start_x,
    int start_y,
    int goal_x,
    int goal_y,
    const std::string& planner,
    const std::string& controller,
    const std::string& local_planner,
    std::size_t frames,
    std::size_t steps_per_frame,
    bool auto_insert_obstacles,
    double prediction_risk_weight,
    double prediction_risk_clearance,
    const std::vector<MovingObstacleSpec>& moving_obstacles,
    const std::vector<ObstacleUpdateSpec>& obstacle_updates,
    std::size_t max_replanning_retries,
    std::size_t replanning_cooldown_frames,
    std::size_t recovery_stop_steps) {
    if (frames == 0 || steps_per_frame == 0 || recovery_stop_steps == 0) {
        throw std::invalid_argument(
            "frames, steps_per_frame, and recovery_stop_steps must be positive");
    }

    autoplanner::GridMap map;
    if (!map.loadFromTxt(map_path)) {
        throw std::runtime_error("failed to load occupancy map: " + map_path);
    }

    robotnav::DynamicPipelineConfig config;
    config.frames = frames;
    config.steps_per_frame = steps_per_frame;
    config.auto_insert_obstacles = auto_insert_obstacles;
    config.max_replanning_retries = max_replanning_retries;
    config.replanning_cooldown_frames = replanning_cooldown_frames;
    config.recovery_stop_steps = recovery_stop_steps;
    config.pipeline.planner = planner;
    config.pipeline.controller = controller;
    config.pipeline.local_planner = local_planner;
    config.pipeline.dynamic_prediction_risk_weight = prediction_risk_weight;
    config.pipeline.dynamic_prediction_risk_clearance = prediction_risk_clearance;
    config.pipeline.max_steps = frames * steps_per_frame;

    for (const auto& spec : moving_obstacles) {
        config.moving_obstacles.push_back(toMovingObstacle(spec));
    }
    for (const auto& spec : obstacle_updates) {
        config.obstacle_updates.push_back({
            spec.frame, {spec.x, spec.y}, spec.occupied});
    }

    robotnav::DynamicNavigationPipeline pipeline;
    robotnav::DynamicPipelineResult result;
    {
        py::gil_scoped_release release;
        result = pipeline.run(map, {start_x, start_y}, {goal_x, goal_y}, config);
    }

    py::dict output;
    output["status_code"] = std::string(
        robotnav::toString(result.metrics.status));
    output["message"] = result.message;
    output["success"] = result.metrics.status == robotnav::StatusCode::Success;

    py::dict metrics;
    metrics["frames_requested"] = result.metrics.frames_requested;
    metrics["frames_run"] = result.metrics.frames_run;
    metrics["steps"] = result.metrics.steps;
    metrics["replanning_count"] = result.metrics.replanning_count;
    metrics["space_time_planning_count"] =
        result.metrics.space_time_planning_count;
    metrics["external_update_count"] = result.metrics.external_update_count;
    metrics["moving_obstacle_update_count"] =
        result.metrics.moving_obstacle_update_count;
    metrics["dynamic_local_collision_rejections"] =
        result.metrics.dynamic_local_collision_rejections;
    metrics["local_planner_warm_start_count"] =
        result.metrics.local_planner_warm_start_count;
    metrics["mppi_diagnostic_count"] =
        result.metrics.mppi_diagnostic_count;
    metrics["mean_mppi_effective_sample_size"] =
        result.metrics.mean_mppi_effective_sample_size;
    metrics["mean_mppi_effective_sample_ratio"] =
        result.metrics.mean_mppi_effective_sample_ratio;
    metrics["mppi_sampling_noise_scale"] =
        result.metrics.mppi_sampling_noise_scale;
    metrics["collision_steps"] = result.metrics.collision_steps;
    metrics["state_transition_count"] =
        result.metrics.state_transition_count;
    metrics["recovery_attempt_count"] =
        result.metrics.recovery_attempt_count;
    metrics["yielding_steps"] = result.metrics.yielding_steps;
    metrics["suppressed_replanning_count"] =
        result.metrics.suppressed_replanning_count;
    metrics["final_state"] = std::string(
        robotnav::toString(result.metrics.final_state));
    metrics["total_space_time_planning_time_ms"] =
        result.metrics.total_space_time_planning_time_ms;
    metrics["total_dstar_replanning_time_ms"] =
        result.metrics.total_dstar_replanning_time_ms;
    metrics["total_astar_replanning_time_ms"] =
        result.metrics.total_astar_replanning_time_ms;
    metrics["local_planner_time_ms"] = result.metrics.local_planner_time_ms;
    metrics["minimum_dynamic_obstacle_clearance"] =
        result.metrics.minimum_dynamic_obstacle_clearance;
    metrics["goal_distance"] = result.metrics.goal_distance;
    metrics["goal_reached"] = result.metrics.goal_reached;
    metrics["safe_stop"] = result.metrics.safe_stop;
    metrics["kinematic_feasible"] = result.metrics.kinematic_feasible;
    output["metrics"] = metrics;

    py::list path;
    for (const auto& point : result.final_path) {
        path.append(py::make_tuple(point.x, point.y));
    }
    output["path"] = path;

    py::list trace;
    for (const auto& sample : result.trace) {
        py::dict item;
        item["frame"] = sample.frame;
        item["step"] = sample.step;
        item["time"] = sample.time;
        item["x"] = sample.state.x;
        item["y"] = sample.state.y;
        item["theta"] = sample.state.theta;
        item["velocity"] = sample.state.v;
        item["command_velocity"] = sample.command.velocity;
        item["command_steering"] = sample.command.steering;
        item["replanned"] = sample.replanned;
        item["safe_stop"] = sample.safe_stop;
        item["navigation_state"] = std::string(
            robotnav::toString(sample.navigation_state));
        trace.append(item);
    }
    output["trace"] = trace;

    py::list state_transitions;
    for (const auto& transition : result.state_transitions) {
        py::dict item;
        item["frame"] = transition.frame;
        item["step"] = transition.step;
        item["time"] = transition.time;
        item["from"] = std::string(robotnav::toString(transition.from));
        item["to"] = std::string(robotnav::toString(transition.to));
        item["reason"] = transition.reason;
        state_transitions.append(item);
    }
    output["state_transitions"] = state_transitions;
    return output;
}

}  // namespace

PYBIND11_MODULE(_robotnav, module) {
    module.doc() = "C++ dynamic navigation backend for the RobotNav Python API";

    py::class_<MovingObstacleSpec>(module, "MovingObstacle")
        .def(py::init<>())
        .def_readwrite("start_frame", &MovingObstacleSpec::start_frame)
        .def_readwrite("end_frame", &MovingObstacleSpec::end_frame)
        .def_readwrite("x", &MovingObstacleSpec::x)
        .def_readwrite("y", &MovingObstacleSpec::y)
        .def_readwrite("dx", &MovingObstacleSpec::dx)
        .def_readwrite("dy", &MovingObstacleSpec::dy)
        .def_readwrite("radius", &MovingObstacleSpec::radius)
        .def_readwrite("uncertainty_growth", &MovingObstacleSpec::uncertainty_growth)
        .def_readwrite("acceleration_x", &MovingObstacleSpec::acceleration_x)
        .def_readwrite("acceleration_y", &MovingObstacleSpec::acceleration_y)
        .def_readwrite("covariance_xx", &MovingObstacleSpec::covariance_xx)
        .def_readwrite("covariance_xy", &MovingObstacleSpec::covariance_xy)
        .def_readwrite("covariance_yy", &MovingObstacleSpec::covariance_yy)
        .def_readwrite("covariance_growth_xx", &MovingObstacleSpec::covariance_growth_xx)
        .def_readwrite("covariance_growth_xy", &MovingObstacleSpec::covariance_growth_xy)
        .def_readwrite("covariance_growth_yy", &MovingObstacleSpec::covariance_growth_yy)
        .def_readwrite("confidence_scale", &MovingObstacleSpec::confidence_scale);

    py::class_<ObstacleUpdateSpec>(module, "ObstacleUpdate")
        .def(py::init<>())
        .def_readwrite("frame", &ObstacleUpdateSpec::frame)
        .def_readwrite("x", &ObstacleUpdateSpec::x)
        .def_readwrite("y", &ObstacleUpdateSpec::y)
        .def_readwrite("occupied", &ObstacleUpdateSpec::occupied);

    module.def("run_dynamic", &runDynamic,
               py::arg("map_path"), py::arg("start_x"), py::arg("start_y"),
               py::arg("goal_x"), py::arg("goal_y"),
               py::arg("planner") = "astar",
               py::arg("controller") = "stanley",
               py::arg("local_planner") = "none",
               py::arg("frames") = 5,
               py::arg("steps_per_frame") = 40,
               py::arg("auto_insert_obstacles") = true,
               py::arg("prediction_risk_weight") = 0.0,
               py::arg("prediction_risk_clearance") = 0.0,
               py::arg("moving_obstacles") = std::vector<MovingObstacleSpec>{},
               py::arg("obstacle_updates") = std::vector<ObstacleUpdateSpec>{},
               py::arg("max_replanning_retries") = 1,
               py::arg("replanning_cooldown_frames") = 0,
               py::arg("recovery_stop_steps") = 10);
}
