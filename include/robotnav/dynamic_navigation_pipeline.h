#pragma once

#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

#include "autompc/core/types.h"
#include "autoplanner/core/path.h"
#include "autoplanner/core/planner_result.h"
#include "autoplanner/core/point.h"
#include "robotnav/dynamic_obstacle_prediction.h"
#include "robotnav/navigation_trace.h"
#include "robotnav/pipeline_config.h"
#include "robotnav/status.h"

namespace robotnav {

enum class NavigationState {
    Initializing,
    Tracking,
    Yielding,
    Replanning,
    Recovery,
    SafeStop,
    GoalReached,
    Failed,
};

constexpr std::string_view toString(NavigationState state) noexcept {
    switch (state) {
        case NavigationState::Initializing: return "initializing";
        case NavigationState::Tracking: return "tracking";
        case NavigationState::Yielding: return "yielding";
        case NavigationState::Replanning: return "replanning";
        case NavigationState::Recovery: return "recovery";
        case NavigationState::SafeStop: return "safe_stop";
        case NavigationState::GoalReached: return "goal_reached";
        case NavigationState::Failed: return "failed";
    }
    return "failed";
}

struct NavigationStateTransition {
    std::size_t frame = 0;
    std::size_t step = 0;
    double time = 0.0;
    NavigationState from = NavigationState::Initializing;
    NavigationState to = NavigationState::Initializing;
    std::string reason;
};

struct DynamicObstacleUpdate {
    std::size_t frame = 0;
    autoplanner::Point2i cell{-1, -1};
    bool occupied = true;
};

struct DynamicPipelineConfig {
    PipelineConfig pipeline;
    std::size_t frames = 5;
    std::size_t steps_per_frame = 40;
    std::size_t obstacle_insertion_ahead = 5;
    std::size_t auto_obstacle_margin_cells = 1;
    std::size_t max_auto_obstacles = 1;
    std::size_t prediction_horizon_frames = 120;
    std::size_t max_replanning_retries = 1;
    std::size_t replanning_cooldown_frames = 0;
    std::size_t recovery_stop_steps = 10;
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
    NavigationState navigation_state = NavigationState::Initializing;
};

struct DynamicPipelineMetrics {
    StatusCode status = StatusCode::InternalError;
    std::string footprint = "point";
    std::string smoother = "none";
    std::string local_planner = "none";
    std::size_t frames_requested = 0;
    std::size_t frames_run = 0;
    std::size_t steps = 0;
    std::size_t replanning_count = 0;
    std::size_t space_time_planning_count = 0;
    std::size_t local_planner_adjustments = 0;
    std::size_t local_planner_rollouts = 0;
    std::size_t local_planner_warm_start_count = 0;
    std::size_t mppi_diagnostic_count = 0;
    double mean_mppi_effective_sample_size = 0.0;
    double mean_mppi_effective_sample_ratio = 0.0;
    double mppi_sampling_noise_scale = 1.0;
    double maximum_mppi_collision_probability = 0.0;
    std::size_t external_update_count = 0;
    std::size_t moving_obstacle_update_count = 0;
    std::size_t moving_obstacle_conflict_count = 0;
    std::size_t dynamic_local_collision_rejections = 0;
    std::size_t dstar_failure_count = 0;
    std::size_t astar_fallback_count = 0;
    std::size_t collision_steps = 0;
    std::size_t state_transition_count = 0;
    std::size_t recovery_attempt_count = 0;
    std::size_t yielding_steps = 0;
    std::size_t suppressed_replanning_count = 0;
    double total_space_time_planning_time_ms = 0.0;
    double total_dstar_replanning_time_ms = 0.0;
    double total_astar_replanning_time_ms = 0.0;
    double local_planner_time_ms = 0.0;
    double max_control_jump = 0.0;
    double mean_control_jump = 0.0;
    double max_trajectory_curvature = 0.0;
    double minimum_turning_radius = 0.0;
    bool kinematic_feasible = false;
    double minimum_dynamic_obstacle_clearance = 0.0;
    double prediction_risk_weight = 0.0;
    double prediction_risk_clearance = 0.0;
    double goal_distance = 0.0;
    bool goal_reached = false;
    bool safe_stop = false;
    NavigationState final_state = NavigationState::Initializing;
};

struct DynamicPipelineResult {
    DynamicPipelineMetrics metrics;
    std::string message;
    autoplanner::PlannerResult initial_planning;
    autoplanner::Path2d final_path;
    std::vector<DynamicTraceSample> trace;
    std::vector<NavigationStateTransition> state_transitions;
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
