#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <memory>
#include <string>
#include <utility>

#include <gtest/gtest.h>

#include "autoplanner/collision/grid_collision_checker.h"
#include "autoplanner/core/grid_map.h"
#include "autompc/core/trajectory.h"
#include "autompc/trajectory/trajectory_generator.h"
#include "robotnav/dwa_local_planner.h"
#include "robotnav/dynamic_navigation_pipeline.h"
#include "robotnav/local_planner_registry.h"
#include "robotnav/mppi_local_planner.h"
#include "robotnav/navigation_pipeline.h"
#include "robotnav/navigation_trace.h"
#include "robotnav/safety_supervisor.h"
#include "robotnav/scenario_config.h"
#include "robotnav/space_time_astar.h"
#include "test_file_utils.h"

namespace {

autoplanner::GridMap loadSimpleMap() {
    autoplanner::GridMap map;
    EXPECT_TRUE(map.loadFromTxt(
        "autoplanner/data/maps/simple_50x50.txt"));
    return map;
}

autoplanner::GridMap loadCorridorMap() {
    const auto path = robotnav_test::artifactPath(
        "robotnav_dynamic_corridor_test.txt");
    {
        std::ofstream output(path);
        EXPECT_TRUE(output.is_open());
        output << "111111111111111111111111111111\n";
        for (int row = 0; row < 5; ++row) {
            output << "100000000000000000000000000001\n";
        }
        output << "111111111111111111111111111111\n";
    }
    autoplanner::GridMap map;
    EXPECT_TRUE(map.loadFromTxt(path.string()));
    std::error_code error;
    std::filesystem::remove(path, error);
    return map;
}

robotnav::DynamicPipelineConfig corridorDynamicConfig() {
    robotnav::DynamicPipelineConfig config;
    config.pipeline.planner = "space_time_astar";
    config.pipeline.controller = "stanley";
    config.pipeline.max_steps = 1600;
    config.frames = 30;
    config.steps_per_frame = 40;
    config.auto_insert_obstacles = false;
    return config;
}

class ClockRecordingLocalPlanner final : public robotnav::LocalPlanner {
public:
    explicit ClockRecordingLocalPlanner(
        std::shared_ptr<std::vector<double>> prediction_frames)
        : prediction_frames_(std::move(prediction_frames)) {}

    robotnav::LocalPlannerDecision computeCommand(
        const autompc::State&,
        double,
        const autompc::Trajectory&,
        const autompc::Control& nominal_command,
        const robotnav::DynamicObstacleContext& dynamic_context) const override {
        prediction_frames_->push_back(
            dynamic_context.predictionFrameAfter(0.0));
        return {true, nominal_command};
    }

private:
    std::shared_ptr<std::vector<double>> prediction_frames_;
};

}  // namespace

TEST(DynamicObstaclePredictionTest, ComputesContinuousClearance) {
    const robotnav::MovingObstacle obstacle{0, 10, {2, 2}, 1, 0};
    const std::vector<robotnav::MovingObstacle> obstacles = {obstacle};

    autoplanner::Point2d predicted;
    ASSERT_TRUE(robotnav::predictMovingObstaclePosition(
        obstacle, 0.5, predicted));
    EXPECT_DOUBLE_EQ(predicted.x, 2.5);
    EXPECT_DOUBLE_EQ(predicted.y, 2.0);
    EXPECT_DOUBLE_EQ(
        robotnav::predictedObstacleClearance(obstacles, {2.5, 2.5}, 0.5),
        0.0);
    EXPECT_TRUE(robotnav::isPredictedCollision(
        obstacles, {2.5, 2.5}, 0.5));
    EXPECT_FALSE(robotnav::isPredictedCollision(
        obstacles, {1.5, 2.5}, 0.5));
    EXPECT_TRUE(robotnav::isPredictedCollision(
        obstacles, {2.0, 2.5}, 0.5, 0.5));
}

TEST(DynamicObstaclePredictionTest, ExpandsFootprintWithUncertainty) {
    const robotnav::MovingObstacle obstacle{
        0, 10, {2, 2}, 0, 0, 0.25, 0.1};
    const std::vector<robotnav::MovingObstacle> obstacles = {obstacle};

    EXPECT_NEAR(
        robotnav::predictedObstacleClearance(obstacles, {1.25, 2.5}, 5.0),
        0.75 - 0.25 - 0.5, 1e-9);
    EXPECT_TRUE(robotnav::isPredictedCollision(
        obstacles, {1.25, 2.5}, 5.0));
}

TEST(DynamicObstaclePredictionTest, PredictsConstantAcceleration) {
    robotnav::MovingObstacle obstacle{0, 10, {1, 2}, 2, -1};
    obstacle.acceleration_x_per_frame2 = 0.5;
    obstacle.acceleration_y_per_frame2 = 2.0;

    autoplanner::Point2d predicted;
    ASSERT_TRUE(robotnav::predictMovingObstaclePosition(
        obstacle, 4.0, predicted));
    EXPECT_DOUBLE_EQ(predicted.x, 13.0);
    EXPECT_DOUBLE_EQ(predicted.y, 14.0);
}

TEST(DynamicObstaclePredictionTest, UsesCovarianceEigenvalueForSafetyRadius) {
    robotnav::MovingObstacle obstacle{0, 10, {2, 2}, 0, 0};
    obstacle.radius = 0.5;
    obstacle.covariance_xx = 1.0;
    obstacle.covariance_xy = 0.0;
    obstacle.covariance_yy = 4.0;
    obstacle.covariance_growth_xx_per_frame = 0.25;
    obstacle.covariance_growth_yy_per_frame = 0.0;
    obstacle.covariance_confidence_scale = 2.0;

    EXPECT_TRUE(robotnav::isValidMovingObstacle(obstacle));
    EXPECT_NEAR(robotnav::largestCovarianceStandardDeviation(obstacle, 4.0),
                2.0, 1e-9);
    EXPECT_NEAR(robotnav::predictedObstacleSafetyRadius(obstacle, 4.0),
                4.5, 1e-9);
}

TEST(DynamicObstaclePredictionTest, CovarianceExpansionMakesCollisionConservative) {
    robotnav::MovingObstacle nominal{0, 10, {2, 2}, 0, 0};
    robotnav::MovingObstacle uncertain = nominal;
    uncertain.covariance_xx = 1.0;
    uncertain.covariance_yy = 1.0;
    const std::vector<robotnav::MovingObstacle> nominal_obstacles = {nominal};
    const std::vector<robotnav::MovingObstacle> uncertain_obstacles = {
        uncertain};

    EXPECT_FALSE(robotnav::isPredictedCollision(
        nominal_obstacles, {0.25, 2.5}, 0.0));
    EXPECT_TRUE(robotnav::isPredictedCollision(
        uncertain_obstacles, {0.25, 2.5}, 0.0));
    EXPECT_LT(robotnav::predictedObstacleClearance(
                  uncertain_obstacles, {0.25, 2.5}, 0.0),
              robotnav::predictedObstacleClearance(
                  nominal_obstacles, {0.25, 2.5}, 0.0));
}

TEST(DynamicObstaclePredictionTest, RejectsInvalidPredictionConfiguration) {
    const auto map = loadSimpleMap();
    robotnav::DynamicPipelineConfig config;
    config.pipeline.controller = "stanley";
    config.frames = 2;
    config.steps_per_frame = 2;
    config.auto_insert_obstacles = false;
    config.moving_obstacles.push_back({3, 1, {3, 3}, 0, 0});

    const auto result = robotnav::DynamicNavigationPipeline{}.run(
        map, {1, 1}, {5, 5}, config);
    EXPECT_EQ(result.metrics.status,
              robotnav::StatusCode::InvalidConfiguration);
}

TEST(DynamicObstaclePredictionTest, UsesSimulationTimeAsCanonicalClock) {
    const robotnav::DynamicObstacleContext timed_context{
        nullptr, 99, 2.0, 0.0, 3.0};
    EXPECT_DOUBLE_EQ(timed_context.predictionFrameAfter(1.0), 2.0);

    const robotnav::DynamicObstacleContext legacy_context{
        nullptr, 3, 2.0, 0.0};
    EXPECT_DOUBLE_EQ(legacy_context.predictionFrameAfter(1.0), 3.5);
    EXPECT_TRUE(std::isnan(robotnav::predictionFrameAtTime(-1.0, 2.0)));
    EXPECT_TRUE(std::isnan(robotnav::predictionFrameAtTime(1.0, 0.0)));
}

TEST(DynamicNavigationPipelineTest, AdvancesLocalPlannerPredictionClock) {
    constexpr const char* planner_name = "clock_recording_test";
    auto prediction_frames = std::make_shared<std::vector<double>>();
    auto& registry = robotnav::LocalPlannerRegistry::instance();
    ASSERT_TRUE(registry.registerLocalPlanner(
        planner_name,
        [prediction_frames](
            const autoplanner::CollisionChecker&,
            const autompc::SimulationOptions&,
            const robotnav::DwaOptions&,
            const robotnav::MppiOptions&) {
            return std::make_unique<ClockRecordingLocalPlanner>(
                prediction_frames);
        }));

    auto config = robotnav::DynamicPipelineConfig{};
    config.pipeline.local_planner = planner_name;
    config.pipeline.max_steps = 4;
    config.frames = 1;
    config.steps_per_frame = 4;
    config.auto_insert_obstacles = false;
    const robotnav::DynamicNavigationPipeline pipeline;
    const auto result = pipeline.run(
        loadSimpleMap(), {1, 1}, {20, 20}, config);
    EXPECT_TRUE(registry.unregisterLocalPlanner(planner_name));

    EXPECT_EQ(result.metrics.status, robotnav::StatusCode::Timeout)
        << result.message;
    ASSERT_EQ(prediction_frames->size(), 4u);
    EXPECT_DOUBLE_EQ((*prediction_frames)[0], 0.0);
    EXPECT_DOUBLE_EQ((*prediction_frames)[1], 0.25);
    EXPECT_DOUBLE_EQ((*prediction_frames)[2], 0.5);
    EXPECT_DOUBLE_EQ((*prediction_frames)[3], 0.75);
}

TEST(DynamicObstaclePredictionTest, RejectsNonPositiveSemidefiniteCovariance) {
    const auto map = loadSimpleMap();
    robotnav::DynamicPipelineConfig config;
    config.pipeline.controller = "stanley";
    config.frames = 2;
    config.steps_per_frame = 2;
    config.auto_insert_obstacles = false;
    robotnav::MovingObstacle obstacle{0, 2, {3, 3}, 0, 0};
    obstacle.covariance_xx = 1.0;
    obstacle.covariance_xy = 2.0;
    obstacle.covariance_yy = 1.0;
    config.moving_obstacles.push_back(obstacle);

    const auto result = robotnav::DynamicNavigationPipeline{}.run(
        map, {1, 1}, {5, 5}, config);
    EXPECT_EQ(result.metrics.status,
              robotnav::StatusCode::InvalidConfiguration);
}

TEST(DynamicNavigationPipelineTest, RejectsInvalidPredictionRisk) {
    const auto map = loadSimpleMap();
    robotnav::DynamicPipelineConfig config;
    config.pipeline.dynamic_prediction_risk_weight = -1.0;
    config.auto_insert_obstacles = false;
    config.frames = 1;
    config.steps_per_frame = 1;

    const auto result = robotnav::DynamicNavigationPipeline{}.run(
        map, {1, 1}, {5, 5}, config);
    EXPECT_EQ(result.metrics.status,
              robotnav::StatusCode::InvalidConfiguration);
}

TEST(SpaceTimeAStarTest, RiskCostPrefersExtraClearance) {
    const auto path = robotnav_test::artifactPath("space_time_risk_map.txt");
    {
        std::ofstream output(path);
        ASSERT_TRUE(output.is_open());
        for (int row = 0; row < 7; ++row) {
            output << std::string(9, '0') << '\n';
        }
    }

    autoplanner::GridMap map;
    ASSERT_TRUE(map.loadFromTxt(path.string()));
    const std::vector<robotnav::MovingObstacle> obstacles = {
        {0, 20, {4, 0}, 0, 0, 0.5, 0.0}};
    const robotnav::SpaceTimeAStarPlanner planner({
        false, true, 20, 0.0, 8.0, 2.0});
    const auto result = planner.plan(
        map, {1, 2}, {7, 2}, obstacles, 0);

    ASSERT_TRUE(result.success) << result.message;
    ASSERT_FALSE(result.path.empty());
    EXPECT_TRUE(std::any_of(
        result.path.begin(), result.path.end(),
        [](const autoplanner::Point2d& point) { return point.y > 2.0; }));

    std::error_code error;
    std::filesystem::remove(path, error);
}

TEST(DwaLocalPlannerTest, RejectsCommandsEnteringPredictedObstacle) {
    const auto path = robotnav_test::artifactPath("dwa_dynamic_map.txt");
    {
        std::ofstream output(path);
        ASSERT_TRUE(output.is_open());
        for (int row = 0; row < 6; ++row) {
            output << std::string(20, '0') << '\n';
        }
    }

    autoplanner::GridMap map;
    ASSERT_TRUE(map.loadFromTxt(path.string()));
    autoplanner::GridCollisionChecker checker(map);
    autompc::SimulationOptions simulation;
    simulation.dt = 0.1;
    simulation.max_acceleration = 10.0;
    simulation.max_deceleration = 10.0;
    simulation.max_steering_rate = 10.0;
    robotnav::DwaOptions options;
    options.prediction_time = 2.0;
    options.velocity_samples = 5;
    options.steering_samples = 3;
    options.dynamic_collision_samples = 5;
    robotnav::DwaLocalPlanner planner(checker, simulation, options);

    const auto trajectory = autompc::makeStraightLine(1.0, 2.5, 8.0, 2.5,
                                                       1.0, 30);
    const std::vector<robotnav::MovingObstacle> obstacles = {
        {0, 10, {3, 2}, 0, 0}};
    const robotnav::DwaDynamicContext context{
        &obstacles, 0, 1.0, 0.0};
    const auto decision = planner.computeCommand(
        {1.0, 2.5, 0.0, 1.0}, 0.0, trajectory, {1.0, 0.0}, context);

    EXPECT_TRUE(decision.feasible);
    EXPECT_GT(decision.dynamic_collision_rejections, 0u);
    EXPECT_TRUE(std::isfinite(decision.minimum_dynamic_clearance));
}

TEST(MppiLocalPlannerTest, SamplesDeterministicallyAndAvoidsPrediction) {
    const auto path = robotnav_test::artifactPath("mppi_dynamic_map.txt");
    {
        std::ofstream output(path);
        ASSERT_TRUE(output.is_open());
        for (int row = 0; row < 6; ++row) {
            output << std::string(20, '0') << '\n';
        }
    }

    autoplanner::GridMap map;
    ASSERT_TRUE(map.loadFromTxt(path.string()));
    autoplanner::GridCollisionChecker checker(map);
    autompc::SimulationOptions simulation;
    simulation.dt = 0.1;
    simulation.max_acceleration = 10.0;
    simulation.max_deceleration = 10.0;
    simulation.max_steering_rate = 10.0;
    robotnav::MppiOptions options;
    options.prediction_time = 2.0;
    options.horizon = 20;
    options.rollouts = 48;
    options.temperature = 0.4;
    options.velocity_noise = 0.4;
    options.steering_noise = 0.25;
    options.dynamic_collision_samples = 5;
    options.warm_start = false;
    robotnav::MppiLocalPlanner planner(checker, simulation, options);

    const auto trajectory = autompc::makeStraightLine(
        1.0, 2.5, 8.0, 2.5, 1.0, 30);
    const std::vector<robotnav::MovingObstacle> obstacles = {
        {0, 10, {3, 2}, 0, 0}};
    const robotnav::DynamicObstacleContext context{
        &obstacles, 0, 1.0, 0.0};
    const auto first = planner.computeCommand(
        {1.0, 2.5, 0.0, 1.0}, 0.0, trajectory, {1.0, 0.0}, context);
    const auto second = planner.computeCommand(
        {1.0, 2.5, 0.0, 1.0}, 0.0, trajectory, {1.0, 0.0}, context);

    EXPECT_TRUE(first.feasible);
    EXPECT_GT(first.feasible_rollouts, 0u);
    EXPECT_GT(first.dynamic_collision_rejections, 0u);
    EXPECT_TRUE(std::isfinite(first.minimum_dynamic_clearance));
    EXPECT_DOUBLE_EQ(first.command.velocity, second.command.velocity);
    EXPECT_DOUBLE_EQ(first.command.steering, second.command.steering);

    std::error_code error;
    std::filesystem::remove(path, error);
}

TEST(MppiLocalPlannerTest, WarmStartsFromPreviousOptimalRollout) {
    const auto map = loadSimpleMap();
    autoplanner::GridCollisionChecker checker(map);
    autompc::SimulationOptions simulation;
    simulation.dt = 0.1;
    robotnav::MppiOptions options;
    options.prediction_time = 0.8;
    options.horizon = 8;
    options.rollouts = 24;
    options.warm_start = true;
    robotnav::MppiLocalPlanner planner(checker, simulation, options);
    const auto trajectory = autompc::makeStraightLine(
        1.0, 1.0, 8.0, 1.0, 1.0, 30);

    const auto first = planner.computeCommand(
        {1.0, 1.0, 0.0, 0.0}, 0.0, trajectory, {1.0, 0.0});
    const auto second = planner.computeCommand(
        {1.0, 1.0, 0.0, 0.0}, 0.0, trajectory, {1.0, 0.0});
    planner.resetWarmStart();
    const auto reset = planner.computeCommand(
        {1.0, 1.0, 0.0, 0.0}, 0.0, trajectory, {1.0, 0.0});

    ASSERT_TRUE(first.feasible);
    ASSERT_TRUE(second.feasible);
    ASSERT_TRUE(reset.feasible);
    EXPECT_FALSE(first.warm_started);
    EXPECT_TRUE(second.warm_started);
    EXPECT_FALSE(reset.warm_started);
}

TEST(SafetySupervisorTest, RejectsInvalidTrajectoryAndCommand) {
    const auto map = loadSimpleMap();
    robotnav::SafetySupervisor supervisor(map);

    EXPECT_EQ(supervisor.validateTrajectory({}).status,
              robotnav::StatusCode::InvalidTrajectory);
    EXPECT_EQ(supervisor.validateCommand(
                  {std::numeric_limits<double>::quiet_NaN(), 0.0}).status,
              robotnav::StatusCode::ControllerInfeasible);
}

TEST(TrajectoryQualityTest, ReportsCurvatureAndTurningRadius) {
    autompc::Trajectory trajectory(3);
    trajectory[0].curvature = 0.0;
    trajectory[1].curvature = 0.25;
    trajectory[2].curvature = -0.5;

    const auto quality = autompc::assessTrajectory(trajectory, 0.4);
    EXPECT_TRUE(quality.finite);
    EXPECT_FALSE(quality.curvature_feasible);
    EXPECT_DOUBLE_EQ(quality.max_abs_curvature, 0.5);
    EXPECT_DOUBLE_EQ(quality.minimum_turning_radius, 2.0);

    const auto unconstrained = autompc::assessTrajectory(trajectory);
    EXPECT_TRUE(unconstrained.curvature_feasible);
}

TEST(TrajectoryGeneratorTest, StopsAtCurvatureLimitViolation) {
    autompc::Waypoints waypoints = {{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}};
    autompc::TrajectoryOptions options;
    options.sample_spacing = 0.25;
    options.target_velocity = 1.0;
    options.max_velocity = 1.0;
    options.max_curvature = 0.5;

    const auto trajectory = autompc::generateTrajectory(waypoints, options);
    ASSERT_FALSE(trajectory.empty());
    EXPECT_TRUE(std::any_of(
        trajectory.begin(), trajectory.end(),
        [](const autompc::TrajectoryPoint& point) {
            return std::abs(point.curvature) > 0.5 && point.v == 0.0;
        }));
}

TEST(TrajectoryGeneratorTest, RoundsWideCornersToTheCurvatureLimit) {
    const autompc::Waypoints waypoints = {
        {0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}};
    autompc::TrajectoryOptions options;
    options.sample_spacing = 0.2;
    options.target_velocity = 1.0;
    options.max_velocity = 1.0;
    options.max_curvature = 0.5;

    const auto trajectory = autompc::generateTrajectory(waypoints, options);
    ASSERT_FALSE(trajectory.empty());
    const auto quality = autompc::assessTrajectory(
        trajectory, options.max_curvature);
    EXPECT_TRUE(quality.finite);
    EXPECT_TRUE(quality.curvature_feasible);
    EXPECT_LE(quality.max_abs_curvature, 0.5 + 1e-9);
}

TEST(TrajectoryGeneratorTest, PreservesReverseSegmentsAndStopsForGearChange) {
    const autompc::Waypoints waypoints = {
        {0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}};
    const std::vector<int> directions = {1, -1, -1};
    autompc::TrajectoryOptions options;
    options.sample_spacing = 0.25;
    options.target_velocity = 1.0;
    options.max_velocity = 1.0;
    options.allow_reverse = true;
    options.max_reverse_velocity = 0.5;

    const auto trajectory = autompc::generateTrajectory(
        waypoints, directions, options);
    ASSERT_FALSE(trajectory.empty());
    EXPECT_TRUE(std::any_of(
        trajectory.begin(), trajectory.end(),
        [](const autompc::TrajectoryPoint& point) { return point.v < 0.0; }));
    EXPECT_TRUE(std::any_of(
        trajectory.begin(), trajectory.end(),
        [](const autompc::TrajectoryPoint& point) {
            return std::abs(point.v) < 1e-9;
        }));
    for (const auto& point : trajectory) {
        EXPECT_LE(std::abs(point.v), 1.0 + 1e-9);
        EXPECT_GE(point.v, -0.5 - 1e-9);
    }
}

TEST(SafetySupervisorTest, AllowsConfiguredReverseVelocityOnly) {
    const auto map = loadSimpleMap();
    autompc::Trajectory trajectory(1);
    trajectory.front().v = -0.5;

    robotnav::SafetyOptions options;
    options.allow_reverse = true;
    options.max_reverse_velocity = 0.5;
    robotnav::SafetySupervisor reverse_supervisor(map, options);
    EXPECT_TRUE(reverse_supervisor.validateTrajectory(trajectory).safe);
    EXPECT_TRUE(reverse_supervisor.validateCommand({-0.5, 0.0}).safe);

    options.allow_reverse = false;
    robotnav::SafetySupervisor forward_only_supervisor(map, options);
    EXPECT_FALSE(forward_only_supervisor.validateTrajectory(trajectory).safe);
    EXPECT_FALSE(forward_only_supervisor.validateCommand({-0.5, 0.0}).safe);
}

TEST(SpaceTimeAStarTest, AvoidsPredictedMovingObstacle) {
    const auto path = robotnav_test::artifactPath(
        "robotnav_spacetime_corridor.txt");
    {
        std::ofstream output(path);
        ASSERT_TRUE(output.is_open());
        output << "11111\n"
               << "11111\n"
               << "10001\n"
               << "11111\n"
               << "11111\n";
    }

    autoplanner::GridMap map;
    ASSERT_TRUE(map.loadFromTxt(path.string()));
    const std::vector<robotnav::MovingObstacle> obstacles = {
        {1, 3, {2, 2}, 0, 0}};

    EXPECT_TRUE(robotnav::isPredictedOccupied(obstacles, {2, 2}, 1));
    EXPECT_FALSE(robotnav::isPredictedOccupied(obstacles, {2, 2}, 4));

    const robotnav::SpaceTimeAStarPlanner short_horizon({
        false, true, 2, 0.0});
    EXPECT_FALSE(short_horizon.plan(
        map, {1, 2}, {3, 2}, obstacles, 0).success);

    const robotnav::SpaceTimeAStarPlanner long_horizon({
        false, true, 6, 0.0});
    const auto result = long_horizon.plan(
        map, {1, 2}, {3, 2}, obstacles, 0);
    EXPECT_EQ(result.planner_name, "space_time_astar");
    EXPECT_TRUE(result.success) << result.message;
    ASSERT_FALSE(result.path.empty());
    EXPECT_GT(result.path.size(), 3u);
    EXPECT_EQ(result.path.front().x, 1.0);
    EXPECT_EQ(result.path.back().x, 3.0);

    std::error_code error;
    std::filesystem::remove(path, error);
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

TEST(NavigationPipelineTest, SupportsDwaLocalPlanner) {
    const auto map = loadSimpleMap();
    robotnav::PipelineConfig config;
    config.planner = "astar";
    config.controller = "stanley";
    config.local_planner = "dwa";
    config.dwa_options.prediction_time = 0.8;
    config.dwa_options.velocity_samples = 5;
    config.dwa_options.steering_samples = 7;
    config.max_steps = 3000;

    const robotnav::NavigationPipeline pipeline;
    const auto result = pipeline.run(
        map, {1, 1}, {20, 20}, config);

    EXPECT_EQ(result.metrics.status, robotnav::StatusCode::Success)
        << result.message;
    EXPECT_TRUE(result.metrics.goal_reached);
    EXPECT_EQ(result.metrics.local_planner, "dwa");
    EXPECT_GT(result.metrics.local_planner_adjustments, 0u);
}

TEST(NavigationPipelineTest, SupportsMppiLocalPlanner) {
    const auto map = loadSimpleMap();
    robotnav::PipelineConfig config;
    config.planner = "astar";
    config.controller = "stanley";
    config.local_planner = "mppi";
    config.mppi_options.prediction_time = 0.8;
    config.mppi_options.horizon = 10;
    config.mppi_options.rollouts = 32;
    config.max_steps = 3000;

    const robotnav::NavigationPipeline pipeline;
    const auto result = pipeline.run(
        map, {1, 1}, {20, 20}, config);

    EXPECT_EQ(result.metrics.status, robotnav::StatusCode::Success)
        << result.message;
    EXPECT_TRUE(result.metrics.goal_reached);
    EXPECT_EQ(result.metrics.local_planner, "mppi");
    EXPECT_GT(result.metrics.local_planner_rollouts, 0u);
    EXPECT_GT(result.metrics.local_planner_warm_start_count, 0u);
    EXPECT_GT(result.metrics.local_planner_time_ms, 0.0);
}

TEST(NavigationPipelineTest, SupportsCurvatureConstrainedSmoothing) {
    const auto map = loadSimpleMap();
    robotnav::PipelineConfig config;
    config.planner = "astar";
    config.controller = "stanley";
    config.smoother = "curvature";
    config.smoothing_max_curvature = 0.3;
    config.smoothing_iterations = 80;
    config.max_steps = 3000;

    const robotnav::NavigationPipeline pipeline;
    const auto result = pipeline.run(
        map, {1, 1}, {20, 20}, config);

    EXPECT_EQ(result.metrics.status, robotnav::StatusCode::Success)
        << result.message;
    EXPECT_TRUE(result.metrics.goal_reached);
    EXPECT_TRUE(result.metrics.collision_free);
    EXPECT_EQ(result.metrics.smoother, "curvature");
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
    EXPECT_EQ(result.metrics.final_state,
              robotnav::NavigationState::GoalReached);
    EXPECT_GT(result.metrics.state_transition_count, 0u);
    EXPECT_EQ(result.metrics.state_transition_count,
              result.state_transitions.size());
    ASSERT_FALSE(result.trace.empty());
    EXPECT_NE(robotnav::toString(result.trace.front().navigation_state), "");
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

TEST(DynamicNavigationPipelineTest, RecoversAfterBlockedCorridorClears) {
    const auto map = loadCorridorMap();
    robotnav::DynamicPipelineConfig config;
    config.pipeline.controller = "stanley";
    config.pipeline.max_steps = 1200;
    config.frames = 30;
    config.steps_per_frame = 20;
    config.auto_insert_obstacles = false;
    config.max_replanning_retries = 2;
    config.recovery_stop_steps = 5;
    for (int y = 1; y <= 5; ++y) {
        config.obstacle_updates.push_back({1, {15, y}, true});
        config.obstacle_updates.push_back({2, {15, y}, false});
    }

    const robotnav::DynamicNavigationPipeline pipeline;
    const auto result = pipeline.run(map, {2, 3}, {27, 3}, config);

    EXPECT_EQ(result.metrics.status, robotnav::StatusCode::Success)
        << result.message;
    EXPECT_TRUE(result.metrics.goal_reached);
    EXPECT_FALSE(result.metrics.safe_stop);
    EXPECT_GE(result.metrics.recovery_attempt_count, 1u);
    EXPECT_EQ(result.metrics.yielding_steps, config.recovery_stop_steps);
    EXPECT_TRUE(std::any_of(
        result.trace.begin(), result.trace.end(),
        [](const robotnav::DynamicTraceSample& sample) {
            return sample.navigation_state == robotnav::NavigationState::Yielding;
        }));
    EXPECT_TRUE(std::any_of(
        result.state_transitions.begin(), result.state_transitions.end(),
        [](const robotnav::NavigationStateTransition& transition) {
            return transition.to == robotnav::NavigationState::Recovery &&
                   !transition.reason.empty();
        }));

    auto exhausted_config = config;
    exhausted_config.max_replanning_retries = 0;
    exhausted_config.obstacle_updates.erase(
        std::remove_if(
            exhausted_config.obstacle_updates.begin(),
            exhausted_config.obstacle_updates.end(),
            [](const robotnav::DynamicObstacleUpdate& update) {
                return !update.occupied;
            }),
        exhausted_config.obstacle_updates.end());
    const auto exhausted = pipeline.run(
        map, {2, 3}, {27, 3}, exhausted_config);
    EXPECT_EQ(exhausted.metrics.status,
              robotnav::StatusCode::ReplanningFailed);
    EXPECT_TRUE(exhausted.metrics.safe_stop);
    EXPECT_EQ(exhausted.metrics.recovery_attempt_count, 0u);
    EXPECT_EQ(exhausted.metrics.final_state,
              robotnav::NavigationState::SafeStop);

}

TEST(DynamicNavigationPipelineTest, SuppressesOffPathChangesDuringCooldown) {
    const auto map = loadSimpleMap();
    robotnav::DynamicPipelineConfig config;
    config.pipeline.controller = "stanley";
    config.pipeline.max_steps = 100;
    config.frames = 4;
    config.steps_per_frame = 1;
    config.auto_insert_obstacles = false;
    config.replanning_cooldown_frames = 3;
    config.obstacle_updates.push_back({1, {45, 45}, true});
    config.obstacle_updates.push_back({2, {44, 45}, true});

    const robotnav::DynamicNavigationPipeline pipeline;
    const auto result = pipeline.run(map, {1, 1}, {20, 20}, config);

    EXPECT_EQ(result.metrics.status, robotnav::StatusCode::Timeout)
        << result.message;
    EXPECT_EQ(result.metrics.replanning_count, 1u);
    EXPECT_EQ(result.metrics.suppressed_replanning_count, 1u);
}

TEST(DynamicNavigationPipelineTest, HandlesCrossingObstacleScenario) {
    const auto map = loadCorridorMap();
    auto config = corridorDynamicConfig();
    config.moving_obstacles.push_back({0, 4, {12, 1}, 0, 1});

    const robotnav::DynamicNavigationPipeline pipeline;
    const auto result = pipeline.run(map, {2, 3}, {27, 3}, config);

    EXPECT_EQ(result.metrics.status, robotnav::StatusCode::Success)
        << result.message;
    EXPECT_TRUE(result.metrics.goal_reached);
    EXPECT_FALSE(result.metrics.safe_stop);
    EXPECT_EQ(result.metrics.collision_steps, 0u);
    EXPECT_GT(result.metrics.moving_obstacle_update_count, 0u);
}

TEST(DynamicNavigationPipelineTest, HandlesOvertakingObstacleScenario) {
    const auto map = loadCorridorMap();
    auto config = corridorDynamicConfig();
    config.moving_obstacles.push_back({0, 12, {7, 3}, 1, 0});

    const robotnav::DynamicNavigationPipeline pipeline;
    const auto result = pipeline.run(map, {2, 3}, {27, 3}, config);

    EXPECT_EQ(result.metrics.status, robotnav::StatusCode::Success)
        << result.message;
    EXPECT_TRUE(result.metrics.goal_reached);
    EXPECT_FALSE(result.metrics.safe_stop);
    EXPECT_EQ(result.metrics.collision_steps, 0u);
    EXPECT_GT(result.metrics.moving_obstacle_update_count, 0u);
}

TEST(DynamicNavigationPipelineTest, ReplansWhenObstacleDisappears) {
    const auto map = loadCorridorMap();
    auto config = corridorDynamicConfig();
    config.moving_obstacles.push_back({1, 2, {15, 3}, 0, 0});

    const robotnav::DynamicNavigationPipeline pipeline;
    const auto result = pipeline.run(map, {2, 3}, {27, 3}, config);

    EXPECT_EQ(result.metrics.status, robotnav::StatusCode::Success)
        << result.message;
    EXPECT_TRUE(result.metrics.goal_reached);
    EXPECT_FALSE(result.metrics.safe_stop);
    EXPECT_EQ(result.metrics.collision_steps, 0u);
    EXPECT_GE(result.metrics.moving_obstacle_update_count, 2u);
    EXPECT_GE(result.metrics.replanning_count, 2u);
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

    const auto path = robotnav_test::artifactPath(
        "robotnav_dynamic_metrics_test.json");
    ASSERT_TRUE(robotnav::saveDynamicMetricsJson(result, path.string()));
    std::ifstream input(path);
    ASSERT_TRUE(input.is_open());
    const std::string json(
        (std::istreambuf_iterator<char>(input)),
        std::istreambuf_iterator<char>());
    EXPECT_NE(json.find("moving_obstacle_update_count"), std::string::npos);
    EXPECT_NE(json.find("state_transitions"), std::string::npos);
    EXPECT_NE(json.find("initial path ready"), std::string::npos);
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

TEST(DynamicNavigationPipelineTest, SupportsDwaWithMovingObstacles) {
    const auto map = loadSimpleMap();
    robotnav::DynamicPipelineConfig config;
    config.pipeline.controller = "stanley";
    config.pipeline.local_planner = "dwa";
    config.pipeline.dwa_options.prediction_time = 0.8;
    config.pipeline.max_steps = 1200;
    config.frames = 20;
    config.steps_per_frame = 40;
    config.auto_insert_obstacles = false;
    config.moving_obstacles.push_back({1, 3, {3, 10}, 1, 0});

    const robotnav::DynamicNavigationPipeline pipeline;
    const auto result = pipeline.run(
        map, {1, 1}, {20, 20}, config);

    EXPECT_EQ(result.metrics.status, robotnav::StatusCode::Success)
        << result.message;
    EXPECT_EQ(result.metrics.local_planner, "dwa");
    EXPECT_GT(result.metrics.local_planner_adjustments, 0u);
    EXPECT_GT(result.metrics.moving_obstacle_update_count, 0u);
    EXPECT_TRUE(result.metrics.goal_reached);
    EXPECT_FALSE(result.metrics.safe_stop);
}

TEST(DynamicNavigationPipelineTest, SupportsMppiWithMovingObstacles) {
    const auto map = loadSimpleMap();
    robotnav::DynamicPipelineConfig config;
    config.pipeline.controller = "stanley";
    config.pipeline.local_planner = "mppi";
    config.pipeline.mppi_options.prediction_time = 0.8;
    config.pipeline.mppi_options.horizon = 10;
    config.pipeline.mppi_options.rollouts = 32;
    config.pipeline.max_steps = 1200;
    config.frames = 20;
    config.steps_per_frame = 40;
    config.auto_insert_obstacles = false;
    config.moving_obstacles.push_back({1, 3, {3, 10}, 1, 0});

    const robotnav::DynamicNavigationPipeline pipeline;
    const auto result = pipeline.run(
        map, {1, 1}, {20, 20}, config);

    EXPECT_EQ(result.metrics.status, robotnav::StatusCode::Success)
        << result.message;
    EXPECT_EQ(result.metrics.local_planner, "mppi");
    EXPECT_GT(result.metrics.local_planner_adjustments, 0u);
    EXPECT_GT(result.metrics.local_planner_rollouts, 0u);
    EXPECT_GT(result.metrics.local_planner_warm_start_count, 0u);
    EXPECT_GT(result.metrics.local_planner_time_ms, 0.0);
    EXPECT_GT(result.metrics.moving_obstacle_update_count, 0u);
    EXPECT_TRUE(result.metrics.goal_reached);
    EXPECT_FALSE(result.metrics.safe_stop);
}

TEST(DynamicNavigationPipelineTest, SupportsSpaceTimeAStarPrediction) {
    const auto map = loadSimpleMap();
    robotnav::DynamicPipelineConfig config;
    config.pipeline.planner = "space_time_astar";
    config.pipeline.controller = "stanley";
    config.pipeline.max_steps = 1200;
    config.prediction_horizon_frames = 80;
    config.frames = 20;
    config.steps_per_frame = 40;
    config.auto_insert_obstacles = false;
    config.moving_obstacles.push_back({1, 3, {3, 10}, 1, 0});

    const robotnav::DynamicNavigationPipeline pipeline;
    const auto result = pipeline.run(
        map, {1, 1}, {20, 20}, config);

    EXPECT_EQ(result.metrics.status, robotnav::StatusCode::Success)
        << result.message;
    EXPECT_EQ(result.initial_planning.planner_name, "space_time_astar");
    EXPECT_GT(result.metrics.space_time_planning_count, 0u);
    EXPECT_GT(result.metrics.total_space_time_planning_time_ms, 0.0);
    EXPECT_GT(result.metrics.moving_obstacle_update_count, 0u);
    EXPECT_TRUE(result.metrics.goal_reached);
    EXPECT_FALSE(result.metrics.safe_stop);
}

TEST(DynamicNavigationPipelineTest, SupportsCurvatureSmoothing) {
    const auto map = loadSimpleMap();
    robotnav::DynamicPipelineConfig config;
    config.pipeline.controller = "stanley";
    config.pipeline.smoother = "curvature";
    config.pipeline.smoothing_max_curvature = 0.3;
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
    EXPECT_EQ(result.metrics.smoother, "curvature");
    EXPECT_TRUE(result.metrics.goal_reached);
    EXPECT_FALSE(result.metrics.safe_stop);
}

TEST(ScenarioConfigTest, LoadsPipelineValues) {
    const auto path = robotnav_test::artifactPath(
        "robotnav_pipeline_test.yaml");
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
               << "  allow_reverse: false\n"
               << "  reverse_penalty: 1.8\n"
               << "  collision_check_resolution: 0.1\n"
               << "controller:\n"
               << "  name: pid\n"
               << "robot:\n"
               << "  footprint: rectangle\n"
               << "  length: 0.8\n"
               << "  width: 0.5\n"
               << "smoothing:\n"
               << "  name: shortcut\n"
               << "  iterations: 12\n"
               << "  max_curvature: 0.4\n"
               << "trajectory:\n"
               << "  max_curvature: 0.25\n"
               << "  enforce_kinematic_constraints: true\n"
               << "local_planner:\n"
               << "  name: dwa\n"
               << "  dwa:\n"
               << "    prediction_time: 0.7\n"
               << "    velocity_samples: 3\n"
               << "    steering_samples: 5\n"
               << "    dynamic_obstacle_margin: 0.2\n"
               << "    dynamic_collision_samples: 5\n"
               << "  mppi:\n"
               << "    prediction_time: 0.9\n"
               << "    horizon: 12\n"
               << "    rollouts: 24\n"
               << "    temperature: 0.35\n"
               << "    velocity_noise: 0.3\n"
               << "    steering_noise: 0.2\n"
               << "    dynamic_clearance: 0.6\n"
               << "    warm_start: false\n"
               << "    warm_start_blend: 0.4\n"
               << "dynamic_prediction:\n"
               << "  risk_weight: 2.5\n"
               << "  risk_clearance: 1.2\n"
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
    EXPECT_DOUBLE_EQ(scenario.pipeline.smoothing_max_curvature, 0.4);
    EXPECT_FALSE(scenario.pipeline.planner_options.allow_reverse);
    EXPECT_DOUBLE_EQ(scenario.pipeline.planner_options.reverse_penalty, 1.8);
    EXPECT_DOUBLE_EQ(
        scenario.pipeline.planner_options.collision_check_resolution, 0.1);
    EXPECT_DOUBLE_EQ(scenario.pipeline.trajectory_options.max_curvature, 0.25);
    EXPECT_TRUE(scenario.pipeline.enforce_kinematic_constraints);
    EXPECT_EQ(scenario.pipeline.local_planner, "dwa");
    EXPECT_DOUBLE_EQ(scenario.pipeline.dwa_options.prediction_time, 0.7);
    EXPECT_EQ(scenario.pipeline.dwa_options.velocity_samples, 3);
    EXPECT_EQ(scenario.pipeline.dwa_options.steering_samples, 5);
    EXPECT_DOUBLE_EQ(
        scenario.pipeline.dwa_options.dynamic_obstacle_margin, 0.2);
    EXPECT_EQ(scenario.pipeline.dwa_options.dynamic_collision_samples, 5);
    EXPECT_DOUBLE_EQ(scenario.pipeline.mppi_options.prediction_time, 0.9);
    EXPECT_EQ(scenario.pipeline.mppi_options.horizon, 12);
    EXPECT_EQ(scenario.pipeline.mppi_options.rollouts, 24);
    EXPECT_DOUBLE_EQ(scenario.pipeline.mppi_options.temperature, 0.35);
    EXPECT_DOUBLE_EQ(scenario.pipeline.mppi_options.velocity_noise, 0.3);
    EXPECT_DOUBLE_EQ(scenario.pipeline.mppi_options.steering_noise, 0.2);
    EXPECT_DOUBLE_EQ(scenario.pipeline.mppi_options.dynamic_clearance, 0.6);
    EXPECT_FALSE(scenario.pipeline.mppi_options.warm_start);
    EXPECT_DOUBLE_EQ(scenario.pipeline.mppi_options.warm_start_blend, 0.4);
    EXPECT_DOUBLE_EQ(
        scenario.pipeline.dynamic_prediction_risk_weight, 2.5);
    EXPECT_DOUBLE_EQ(
        scenario.pipeline.dynamic_prediction_risk_clearance, 1.2);
    EXPECT_EQ(scenario.pipeline.max_steps, 123u);
    EXPECT_FALSE(scenario.pipeline.safety_options.enforce_collision);

    std::error_code error;
    std::filesystem::remove(path, error);
}

TEST(NavigationTraceTest, SavesCsv) {
    robotnav::NavigationTrace trace;
    trace.append({0.05, {1.0, 2.0, 0.0, 0.5}, {0.5, 0.1}, 0.2, 0.3});
    const auto path = robotnav_test::artifactPath(
        "robotnav_trace_test.csv");
    ASSERT_TRUE(robotnav::saveNavigationTraceCsv(trace, path.string()));
    std::ifstream input(path);
    ASSERT_TRUE(input.is_open());
    std::string header;
    std::getline(input, header);
    EXPECT_EQ(header, "time,x,y,theta,velocity,command_velocity,command_steering,cross_track_error,heading_error");
    std::error_code error;
    std::filesystem::remove(path, error);
}
