#pragma once

#include <string>

#include "autompc/core/trajectory.h"
#include "autompc/core/types.h"
#include "autoplanner/core/grid_map.h"
#include "autoplanner/core/planner_result.h"
#include "robotnav/navigation_trace.h"
#include "robotnav/pipeline_config.h"
#include "robotnav/status.h"

namespace robotnav {

struct PipelineMetrics {
    StatusCode status = StatusCode::InternalError;
    std::string footprint = "point";
    std::string smoother = "none";
    std::string local_planner = "none";
    double planning_time_ms = 0.0;
    double path_length = 0.0;
    double trajectory_length = 0.0;
    double max_trajectory_curvature = 0.0;
    double minimum_turning_radius = 0.0;
    bool kinematic_feasible = false;
    double max_cross_track_error = 0.0;
    double mean_cross_track_error = 0.0;
    double max_heading_error = 0.0;
    double mean_heading_error = 0.0;
    double goal_distance = 0.0;
    std::size_t steps = 0;
    std::size_t local_planner_adjustments = 0;
    std::size_t local_planner_rollouts = 0;
    std::size_t local_planner_warm_start_count = 0;
    std::size_t mppi_diagnostic_count = 0;
    double mean_mppi_effective_sample_size = 0.0;
    double mean_mppi_effective_sample_ratio = 0.0;
    double mppi_sampling_noise_scale = 1.0;
    double maximum_mppi_collision_probability = 0.0;
    std::size_t local_planner_collision_rejections = 0;
    double local_planner_time_ms = 0.0;
    double minimum_dynamic_obstacle_clearance = 0.0;
    bool collision_free = false;
    bool goal_reached = false;
    bool safe_stop = false;
};

struct PipelineResult {
    PipelineMetrics metrics;
    std::string message;
    autoplanner::PlannerResult planning;
    autompc::Trajectory trajectory;
    NavigationTrace trace;
};

class NavigationPipeline {
public:
    PipelineResult run(const autoplanner::GridMap& map,
                       const autoplanner::Point2i& start,
                       const autoplanner::Point2i& goal,
                       const PipelineConfig& config = {}) const;
};

bool savePipelineMetricsJson(const PipelineResult& result,
                             const std::string& file_path);

}  // namespace robotnav
