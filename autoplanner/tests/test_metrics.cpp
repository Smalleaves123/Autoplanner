#include <gtest/gtest.h>

#include <fstream>
#include <limits>

#include "autoplanner/core/grid_map.h"
#include "autoplanner/core/path.h"
#include "autoplanner/core/planner_result.h"
#include "autoplanner/metrics/path_metrics.h"
#include "autoplanner/planners/graph_search/astar.h"
#include "test_file_utils.h"

using namespace autoplanner;

TEST(PathMetrics, PathLength) {
    Path2d path = {
        {0.0, 0.0}, {3.0, 0.0}, {3.0, 4.0}
    };
    // 3 + 4 = 7
    EXPECT_DOUBLE_EQ(computePathLength(path), 7.0);
}

TEST(PathMetrics, EmptyPathLength) {
    Path2d path;
    EXPECT_DOUBLE_EQ(computePathLength(path), 0.0);
}

TEST(PathMetrics, SinglePointPathLength) {
    Path2d path = {{5.0, 5.0}};
    EXPECT_DOUBLE_EQ(computePathLength(path), 0.0);
}

TEST(PathMetrics, TurningCount) {
    Path2d straight = {{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}};
    EXPECT_EQ(computeTurningCount(straight), 0);

    Path2d turn = {{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}};
    EXPECT_EQ(computeTurningCount(turn), 1);
}

TEST(PathMetrics, TurningCountShort) {
    EXPECT_EQ(computeTurningCount({{0.0, 0.0}}), 0);
    EXPECT_EQ(computeTurningCount({{0.0, 0.0}, {1.0, 1.0}}), 0);
}

TEST(PathMetrics, TotalTurning) {
    Path2d straight = {{0.0, 0.0}, {0.0, 1.0}, {0.0, 2.0}};
    EXPECT_NEAR(computeTotalTurning(straight), 0.0, 1e-9);

    Path2d right_angle = {{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}};
    double total = computeTotalTurning(right_angle);
    EXPECT_NEAR(total, M_PI / 2.0, 1e-6);
}

TEST(PathMetrics, AverageCurvature) {
    Path2d straight = {{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}};
    EXPECT_NEAR(computeAverageCurvature(straight), 0.0, 1e-9);

    Path2d curved = {{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}};
    // Curvature should be > 0 for a turn.
    EXPECT_GT(computeAverageCurvature(curved), 0.0);
}

TEST(PathMetrics, SmoothnessScore) {
    Path2d straight = {{0.0, 0.0}, {5.0, 0.0}};
    EXPECT_NEAR(computeSmoothnessScore(straight), 1.0, 1e-9);

    Path2d winding = {{0.0, 0.0}, {1.0, 0.0}, {1.0, 5.0}};
    double score = computeSmoothnessScore(winding);
    EXPECT_LT(score, 1.0);
}

TEST(PathMetrics, MinObstacleDistance) {
    std::vector<Point2d> path = {{5.0, 5.0}, {6.0, 6.0}};
    std::vector<Point2i> obstacles = {{0, 0}, {50, 50}};

    double min_dist = computeMinObstacleDistance(path, obstacles);

    // The closest obstacle to (5,5) is (0,0) at distance sqrt(50) ≈ 7.07
    EXPECT_GT(min_dist, 5.0);
    EXPECT_LT(min_dist, 10.0);
}

TEST(PathMetrics, MinObstacleDistanceEmpty) {
    // Empty path or obstacles should return infinity.
    double d1 = computeMinObstacleDistance({}, {{0, 0}});
    EXPECT_TRUE(std::isinf(d1));

    double d2 = computeMinObstacleDistance({{0.0, 0.0}}, {});
    EXPECT_TRUE(std::isinf(d2));
}

TEST(PlannerResult, ExposesStableStatusCode) {
    GridMap map;
    ASSERT_TRUE(map.loadFromTxt("data/maps/simple_50x50.txt"));

    AStarPlanner planner(false);
    const auto success = planner.plan(map, {1, 1}, {48, 48});
    EXPECT_EQ(success.statusCodeString(), "success");

    const auto invalid_start = planner.plan(map, {0, 0}, {48, 48});
    EXPECT_EQ(invalid_start.statusCodeString(), "invalid_start");
}

TEST(PlannerResult, MetricsJsonEscapesTextAndRejectsNonFiniteNumbers) {
    PlannerResult result;
    result.planner_name = "planner\"name";
    result.message = "line one\nline two";
    result.minimum_obstacle_distance =
        std::numeric_limits<double>::infinity();

    const auto output_path = autoplanner_test::artifactPath(
        "robotnav_metrics.json");
    ASSERT_TRUE(saveMetricsJson(result, output_path.string()));

    std::ifstream input(output_path);
    const std::string json((std::istreambuf_iterator<char>(input)),
                           std::istreambuf_iterator<char>());
    EXPECT_NE(json.find("planner\\\"name"), std::string::npos);
    EXPECT_NE(json.find("line one\\nline two"), std::string::npos);
    EXPECT_NE(json.find("\"minimum_obstacle_distance\": null"),
              std::string::npos);
}
