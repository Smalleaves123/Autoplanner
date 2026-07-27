#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "autompc/core/types.h"
#include "autoplanner/core/path.h"
#include "autoplanner/core/planner_result.h"
#include "autoplanner/core/point.h"
#include "robotnav/navigation_trace.h"
#include "robotnav/pipeline_config.h"
#include "robotnav/status.h"

namespace robotnav {

struct DynamicObstacleUpdate {
    std::size_t frame = 0;
    autoplanner::Point2i cell{-1, -1};
    bool occupied = true;
};

struct MovingObstacle {
    std::size_t start_frame = 0;
    std::size_t end_frame = 0;
    autoplanner::Point2i start_cell{-1, -1};
    int dx_per_frame = 0;
    int dy_per_frame = 0;
};

struct DynamicPipelineConfig {
    PipelineConfig pipeline;
    std::size_t frames = 5;
    std::size_t steps_per_frame = 40;
    std::size_t obstacle_insertion_ahead = 5;
    std::size_t auto_obstacle_margin_cells = 1;
    std::size_t max_auto_obstacles = 1;
    bool auto_insert_obstacles = true;
    bool compare_astar = true;
    std::vector<DynamicObstacleUpdate> obstacle_updates;
    std::vector<MovingObstacle> moving_obstacles;
};

struct DynamicTraceSample {
    std::size_t frame = 0;
    std::size_t step = 0;
    double time = 0.0;
    autompc::State state;
    autompc::Control command;
    bool replanned = false;
    autoplanner::Point2i obstacle{-1, -1};
    double dstar_replan_ms = 0.0;
    double astar_replan_ms = 0.0;
    double cross_track_error = 0.0;
    double steering_delta = 0.0;
    double velocity_delta = 0.0;
    bool safe_stop = false;
};

struct DynamicPipelineMetrics {
    StatusCode status = StatusCode::InternalError;
    std::string footprint = "point";
    std::string smoother = "none";
    std::size_t frames_requested = 0;
    std::size_t frames_run = 0;
    std::size_t steps = 0;
    std::size_t replanning_count = 0;
    std::size_t external_update_count = 0;
    std::size_t moving_obstacle_update_count = 0;
    std::size_t moving_obstacle_conflict_count = 0;
    std::size_t dstar_failure_count = 0;
    std::size_t astar_fallback_count = 0;
    std::size_t collision_steps = 0;
    double total_dstar_replanning_time_ms = 0.0;
    double total_astar_replanning_time_ms = 0.0;
    double max_control_jump = 0.0;
    double mean_control_jump = 0.0;
    double goal_distance = 0.0;
    bool goal_reached = false;
    bool safe_stop = false;
};

struct DynamicPipelineResult {
    DynamicPipelineMetrics metrics;
    std::string message;
    autoplanner::PlannerResult initial_planning;
    autoplanner::Path2d final_path;
    std::vector<DynamicTraceSample> trace;
};

class DynamicNavigationPipeline {
public:
    DynamicPipelineResult run(const autoplanner::GridMap& map,
                              const autoplanner::Point2i& start,
                              const autoplanner::Point2i& goal,
                              const DynamicPipelineConfig& config = {}) const;
};

bool saveDynamicTraceCsv(const DynamicPipelineResult& result,
                         const std::string& file_path);
bool saveDynamicMetricsJson(const DynamicPipelineResult& result,
                            const std::string& file_path);

}  // namespace robotnav
