#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <string>

#include <gtest/gtest.h>

#include "autoplanner/core/grid_map.h"
#include "robotnav/dynamic_navigation_pipeline.h"
#include "robotnav/navigation_pipeline.h"
#include "robotnav/navigation_trace.h"
#include "robotnav/safety_supervisor.h"
#include "robotnav/scenario_config.h"

namespace {

autoplanner::GridMap loadSimpleMap() {
    autoplanner::GridMap map;
    EXPECT_TRUE(map.loadFromTxt(
        "autoplanner/data/maps/simple_50x50.txt"));
    return map;
}

}  // namespace

TEST(SafetySupervisorTest, RejectsInvalidTrajectoryAndCommand) {
    const auto map = loadSimpleMap();
    robotnav::SafetySupervisor supervisor(map);

    EXPECT_EQ(supervisor.validateTrajectory({}).status,
              robotnav::StatusCode::InvalidTrajectory);
    EXPECT_EQ(supervisor.validateCommand(
                  {std::numeric_limits<double>::quiet_NaN(), 0.0}).status,
              robotnav::StatusCode::ControllerInfeasible);
}

TEST(NavigationPipelineTest, RunsPlanningTrackingAndTrace) {
    const auto map = loadSimpleMap();
    robotnav::PipelineConfig config;
    config.planner = "astar";
    config.controller = "stanley";
    config.max_steps = 2500;

    const robotnav::NavigationPipeline pipeline;
    const auto result = pipeline.run(
        map, {1, 1}, {48, 48}, config);

    EXPECT_EQ(result.metrics.status, robotnav::StatusCode::Success)
        << result.message;
    EXPECT_TRUE(result.metrics.goal_reached);
    EXPECT_FALSE(result.trace.empty());
    EXPECT_EQ(result.metrics.steps, result.trace.size());
    EXPECT_TRUE(std::isfinite(result.metrics.max_cross_track_error));
}

TEST(NavigationPipelineTest, SupportsRectangleFootprintAndSmoothing) {
    const auto map = loadSimpleMap();
    robotnav::PipelineConfig config;
    config.planner = "astar";
    config.controller = "stanley";
    config.footprint = "rectangle";
    config.robot_length = 0.8;
    config.robot_width = 0.5;
    config.inflate_map = true;
    config.smoother = "shortcut";
    config.smoothing_iterations = 100;
    config.max_steps = 2500;

    const robotnav::NavigationPipeline pipeline;
    const auto result = pipeline.run(
        map, {1, 1}, {48, 48}, config);

    EXPECT_EQ(result.metrics.status, robotnav::StatusCode::Success)
        << result.message;
    EXPECT_TRUE(result.metrics.collision_free);
    EXPECT_TRUE(result.metrics.goal_reached);
    EXPECT_EQ(result.metrics.footprint, "rectangle");
    EXPECT_EQ(result.metrics.smoother, "shortcut");
}

TEST(DynamicNavigationPipelineTest, ReplansAndReachesGoalWithoutCollision) {
    const auto map = loadSimpleMap();
    robotnav::DynamicPipelineConfig config;
    config.pipeline.controller = "stanley";
    config.pipeline.smoother = "shortcut";
    config.pipeline.max_steps = 1000;
    config.frames = 20;
    config.steps_per_frame = 40;
    config.obstacle_insertion_ahead = 15;
    config.auto_obstacle_margin_cells = 1;
    config.max_auto_obstacles = 1;

    const robotnav::DynamicNavigationPipeline pipeline;
    const auto result = pipeline.run(
        map, {1, 1}, {20, 20}, config);

    EXPECT_EQ(result.metrics.status, robotnav::StatusCode::Success)
        << result.message;
    EXPECT_TRUE(result.metrics.goal_reached);
    EXPECT_FALSE(result.metrics.safe_stop);
    EXPECT_EQ(result.metrics.collision_steps, 0u);
    EXPECT_GE(result.metrics.replanning_count, 1u);
    EXPECT_EQ(result.metrics.steps, result.trace.size());
    EXPECT_FALSE(result.final_path.empty());
}

TEST(DynamicNavigationPipelineTest, AcceptsExternalObstacleUpdates) {
    const auto map = loadSimpleMap();
    robotnav::DynamicPipelineConfig config;
    config.pipeline.controller = "stanley";
    config.pipeline.max_steps = 1000;
    config.frames = 20;
    config.steps_per_frame = 40;
    config.auto_insert_obstacles = false;
    config.obstacle_updates.push_back({1, {3, 10}, true});

    const robotnav::DynamicNavigationPipeline pipeline;
    const auto result = pipeline.run(
        map, {1, 1}, {20, 20}, config);

    EXPECT_EQ(result.metrics.status, robotnav::StatusCode::Success)
        << result.message;
    EXPECT_EQ(result.metrics.external_update_count, 1u);
    EXPECT_GE(result.metrics.replanning_count, 1u);
    EXPECT_TRUE(result.metrics.goal_reached);
    EXPECT_FALSE(result.metrics.safe_stop);
}

TEST(DynamicNavigationPipelineTest, TracksMovingObstacleUpdates) {
    const auto map = loadSimpleMap();
    robotnav::DynamicPipelineConfig config;
    config.pipeline.controller = "stanley";
    config.pipeline.max_steps = 1000;
    config.frames = 20;
    config.steps_per_frame = 40;
    config.auto_insert_obstacles = false;
    config.moving_obstacles.push_back({1, 3, {3, 10}, 1, 0});

    const robotnav::DynamicNavigationPipeline pipeline;
    const auto result = pipeline.run(
        map, {1, 1}, {20, 20}, config);

    EXPECT_EQ(result.metrics.status, robotnav::StatusCode::Success)
        << result.message;
    EXPECT_GT(result.metrics.moving_obstacle_update_count, 0u);
    EXPECT_GE(result.metrics.replanning_count, 1u);
    EXPECT_TRUE(result.metrics.goal_reached);
    EXPECT_FALSE(result.metrics.safe_stop);

    const auto path = std::filesystem::temp_directory_path() /
                      "robotnav_dynamic_metrics_test.json";
    ASSERT_TRUE(robotnav::saveDynamicMetricsJson(result, path.string()));
    std::ifstream input(path);
    ASSERT_TRUE(input.is_open());
    const std::string json(
        (std::istreambuf_iterator<char>(input)),
        std::istreambuf_iterator<char>());
    EXPECT_NE(json.find("moving_obstacle_update_count"), std::string::npos);
    std::error_code error;
    std::filesystem::remove(path, error);
}

TEST(DynamicNavigationPipelineTest, PreservesExternalOccupancyWhenObstacleMoves) {
    const auto map = loadSimpleMap();
    robotnav::DynamicPipelineConfig config;
    config.pipeline.controller = "stanley";
    config.pipeline.max_steps = 1000;
    config.frames = 20;
    config.steps_per_frame = 40;
    config.auto_insert_obstacles = false;
    config.moving_obstacles.push_back({1, 3, {3, 10}, 1, 0});
    config.obstacle_updates.push_back({1, {3, 10}, true});

    const robotnav::DynamicNavigationPipeline pipeline;
    const auto result = pipeline.run(
        map, {1, 1}, {20, 20}, config);

    EXPECT_EQ(result.metrics.status, robotnav::StatusCode::Success)
        << result.message;
    EXPECT_GE(result.metrics.moving_obstacle_conflict_count, 1u);
    EXPECT_TRUE(result.metrics.goal_reached);
    EXPECT_FALSE(result.metrics.safe_stop);
}

TEST(ScenarioConfigTest, LoadsPipelineValues) {
    const auto path = std::filesystem::temp_directory_path() /
                      "robotnav_pipeline_test.yaml";
    {
        std::ofstream output(path);
        ASSERT_TRUE(output.is_open());
        output << "map:\n"
               << "  path: custom_map.txt\n"
               << "start:\n"
               << "  x: 2\n"
               << "  y: 3\n"
               << "goal:\n"
               << "  x: 20\n"
               << "  y: 21\n"
               << "planner:\n"
               << "  name: dijkstra\n"
               << "controller:\n"
               << "  name: pid\n"
               << "robot:\n"
               << "  footprint: rectangle\n"
               << "  length: 0.8\n"
               << "  width: 0.5\n"
               << "smoothing:\n"
               << "  name: shortcut\n"
               << "  iterations: 12\n"
               << "pipeline:\n"
               << "  max_steps: 123\n"
               << "safety:\n"
               << "  enforce_collision: false\n";
    }

    robotnav::ScenarioConfig scenario;
    ASSERT_TRUE(robotnav::loadScenarioConfig(path.string(), scenario));
    EXPECT_EQ(scenario.map_path, "custom_map.txt");
    EXPECT_EQ(scenario.start, (autoplanner::Point2i{2, 3}));
    EXPECT_EQ(scenario.goal, (autoplanner::Point2i{20, 21}));
    EXPECT_EQ(scenario.pipeline.planner, "dijkstra");
    EXPECT_EQ(scenario.pipeline.controller, "pid");
    EXPECT_EQ(scenario.pipeline.footprint, "rectangle");
    EXPECT_DOUBLE_EQ(scenario.pipeline.robot_length, 0.8);
    EXPECT_DOUBLE_EQ(scenario.pipeline.robot_width, 0.5);
    EXPECT_EQ(scenario.pipeline.smoother, "shortcut");
    EXPECT_EQ(scenario.pipeline.smoothing_iterations, 12);
    EXPECT_EQ(scenario.pipeline.max_steps, 123u);
    EXPECT_FALSE(scenario.pipeline.safety_options.enforce_collision);

    std::error_code error;
    std::filesystem::remove(path, error);
}

TEST(NavigationTraceTest, SavesCsv) {
    robotnav::NavigationTrace trace;
    trace.append({0.05, {1.0, 2.0, 0.0, 0.5}, {0.5, 0.1}, 0.2, 0.3});
    const auto path = std::filesystem::temp_directory_path() /
                      "robotnav_trace_test.csv";
    ASSERT_TRUE(robotnav::saveNavigationTraceCsv(trace, path.string()));
    std::ifstream input(path);
    ASSERT_TRUE(input.is_open());
    std::string header;
    std::getline(input, header);
    EXPECT_EQ(header, "time,x,y,theta,velocity,command_velocity,command_steering,cross_track_error,heading_error");
    std::error_code error;
    std::filesystem::remove(path, error);
}
