#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>

#include <gtest/gtest.h>

#include "autoplanner/core/grid_map.h"
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
