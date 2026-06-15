// test_edge_cases.cpp — Industrial-strength edge case and robustness tests.

#include <gtest/gtest.h>
#include <cmath>
#include <fstream>
#include <limits>

#include "autoplanner/core/grid_map.h"
#include "autoplanner/core/path.h"
#include "autoplanner/costmap/costmap_2d.h"
#include "autoplanner/metrics/path_metrics.h"
#include "autoplanner/planners/graph_search/astar.h"
#include "autoplanner/planners/graph_search/dijkstra.h"
#include "autoplanner/planners/graph_search/improved_astar.h"
#include "autoplanner/planners/sampling/rrt.h"
#include "autoplanner/planners/sampling/rrt_star.h"
#include "autoplanner/collision/grid_collision_checker.h"
#include "autoplanner/smoothing/shortcut_smoother.h"

using namespace autoplanner;

// ── GridMap edge cases ──────────────────────────────────────────────────

TEST(GridMapEdge, EmptyMapIsEmpty) {
    GridMap m;
    EXPECT_TRUE(m.isEmpty());
    EXPECT_EQ(m.width(), 0);
    EXPECT_EQ(m.height(), 0);
}

TEST(GridMapEdge, IndexOutOfBoundsReturnsMinusOne) {
    GridMap m;
    EXPECT_EQ(m.index(-1, 0), -1);
    EXPECT_EQ(m.index(0, -1), -1);
    EXPECT_EQ(m.index(100, 100), -1);
}

TEST(GridMapEdge, NegativeResolutionClamped) {
    GridMap m;
    m.setResolution(-0.05);
    EXPECT_DOUBLE_EQ(m.resolution(), 0.0);
}

TEST(GridMapEdge, AllObstacleMap) {
    std::ofstream f("/tmp/all_wall.txt");
    for (int y = 0; y < 10; ++y) {
        for (int x = 0; x < 10; ++x) f << '1';
        f << '\n';
    }
    f.close();
    GridMap m;
    ASSERT_TRUE(m.loadFromTxt("/tmp/all_wall.txt"));
    EXPECT_EQ(m.width(), 10);
    // Every cell should be occupied
    EXPECT_TRUE(m.isOccupied(5, 5));
    EXPECT_FALSE(m.isFree(0, 0));
}

// ── Planner edge cases ─────────────────────────────────────────────────

TEST(PlannerEdge, AStarStartEqualsGoal) {
    std::ofstream f("/tmp/open_5x5.txt");
    for (int y = 0; y < 5; ++y) {
        std::string row(5, '0');
        f << row << '\n';
    }
    f.close();
    GridMap m;
    m.loadFromTxt("/tmp/open_5x5.txt");

    AStarPlanner astar(false);
    auto r = astar.plan(m, {2, 2}, {2, 2});
    EXPECT_TRUE(r.success);
    EXPECT_EQ(r.path.size(), 1u);
    EXPECT_NEAR(r.path_length, 0.0, 1e-9);
}

TEST(PlannerEdge, AStarEmptyMap) {
    GridMap m;
    AStarPlanner astar(false);
    auto r = astar.plan(m, {0, 0}, {10, 10});
    EXPECT_FALSE(r.success);
    EXPECT_FALSE(r.message.empty());
}

TEST(PlannerEdge, AStarNoPathExists) {
    std::ofstream f("/tmp/blocked.txt");
    f << "00100\n00100\n00100\n00100\n00100\n";  // wall down the middle
    f.close();
    GridMap m;
    m.loadFromTxt("/tmp/blocked.txt");
    AStarPlanner astar(false);
    auto r = astar.plan(m, {0, 0}, {4, 0});
    EXPECT_FALSE(r.success);
    EXPECT_GT(r.expanded_nodes, 0);  // should still have searched
}

TEST(PlannerEdge, StartInObstacle) {
    std::ofstream f("/tmp/tiny.txt");
    f << "10\n01\n";
    f.close();
    GridMap m;
    m.loadFromTxt("/tmp/tiny.txt");
    AStarPlanner astar(false);
    auto r = astar.plan(m, {0, 0}, {1, 1});
    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.path_length, 0.0);
}

TEST(PlannerEdge, GoalInObstacle) {
    std::ofstream f("/tmp/tiny.txt");
    f << "01\n10\n";
    f.close();
    GridMap m;
    m.loadFromTxt("/tmp/tiny.txt");
    AStarPlanner astar(false);
    auto r = astar.plan(m, {1, 0}, {0, 1});
    EXPECT_FALSE(r.success);
}

// ── RRT edge cases ─────────────────────────────────────────────────────

TEST(RRTEdge, NegativeStepSize) {
    std::ofstream f("/tmp/open_10x10.txt");
    for (int y = 0; y < 10; ++y) { std::string row(10, '0'); f << row << '\n'; }
    f.close();
    GridMap m;
    m.loadFromTxt("/tmp/open_10x10.txt");

    RRTPlanner rrt(-1.0, 100, 0.1, 1.0);  // negative step size
    auto r = rrt.plan(m, {0, 0}, {9, 9});
    // Should not crash — may fail gracefully
    EXPECT_FALSE(r.success);
}

TEST(RRTEdge, AllObstacles) {
    std::ofstream f("/tmp/all_wall_10.txt");
    for (int y = 0; y < 10; ++y) { std::string row(10, '1'); f << row << '\n'; }
    f.close();
    GridMap m;
    m.loadFromTxt("/tmp/all_wall_10.txt");

    RRTPlanner rrt(1.0, 500, 0.1, 1.0);
    auto r = rrt.plan(m, {0, 0}, {9, 9});
    EXPECT_FALSE(r.success);
}

TEST(RRTEdge, ZeroIterations) {
    std::ofstream f("/tmp/open_5.txt");
    for (int y = 0; y < 5; ++y) { std::string row(5, '0'); f << row << '\n'; }
    f.close();
    GridMap m;
    m.loadFromTxt("/tmp/open_5.txt");

    RRTPlanner rrt(1.0, 0, 0.1, 1.0);
    auto r = rrt.plan(m, {0, 0}, {4, 4});
    EXPECT_FALSE(r.success);
}

// ── Costmap2D edge cases ───────────────────────────────────────────────

TEST(CostmapEdge, EmptyMap) {
    GridMap m;
    Costmap2D cm;
    cm.buildFromGridMap(m);
    EXPECT_EQ(cm.width(), 0);
    EXPECT_EQ(cm.height(), 0);
    EXPECT_DOUBLE_EQ(cm.getCost(0, 0), 1.0);  // out-of-bounds = lethal
}

TEST(CostmapEdge, NegativeInflateRadius) {
    std::ofstream f("/tmp/open_10.txt");
    for (int y = 0; y < 10; ++y) { std::string row(10, '0'); f << row << '\n'; }
    f.close();
    GridMap m;
    m.loadFromTxt("/tmp/open_10.txt");
    Costmap2D cm;
    cm.buildFromGridMap(m);
    EXPECT_NO_THROW(cm.inflateObstacles(-1.0));  // should not crash
}

TEST(CostmapEdge, ZeroResolution) {
    GridMap m;
    m.setResolution(0.0);
    EXPECT_DOUBLE_EQ(m.resolution(), 0.0);
}

// ── PathMetrics edge cases ─────────────────────────────────────────────

TEST(MetricsEdge, EmptyPath) {
    Path2d empty;
    EXPECT_EQ(computeTurningCount(empty), 0);
    EXPECT_NEAR(computeSmoothnessScore(empty), 1.0, 1e-9);
    std::vector<Point2i> obs;
    EXPECT_TRUE(std::isinf(computeMinObstacleDistance(empty, obs)));
}

TEST(MetricsEdge, SinglePointPath) {
    Path2d single = {{0.0, 0.0}};
    EXPECT_EQ(computeTurningCount(single), 0);
    EXPECT_DOUBLE_EQ(computePathLength(single), 0.0);
    EXPECT_NEAR(computeSmoothnessScore(single), 1.0, 1e-9);
}

TEST(MetricsEdge, TwoPointPath) {
    Path2d two = {{0.0, 0.0}, {3.0, 4.0}};
    EXPECT_EQ(computeTurningCount(two), 0);
    EXPECT_NEAR(computePathLength(two), 5.0, 1e-9);
}

// ── CollisionChecker edge cases ─────────────────────────────────────────

TEST(CollisionEdge, EmptyMapChecker) {
    GridMap m;
    GridCollisionChecker cc(m);
    EXPECT_FALSE(cc.isStateValid({0, 0}));
    EXPECT_FALSE(cc.isSegmentValid({0, 0}, {1, 1}));
}

TEST(CollisionEdge, SamePointSegment) {
    std::ofstream f("/tmp/open_5.txt");
    for (int y = 0; y < 5; ++y) { std::string row(5, '0'); f << row << '\n'; }
    f.close();
    GridMap m;
    m.loadFromTxt("/tmp/open_5.txt");
    GridCollisionChecker cc(m);
    EXPECT_TRUE(cc.isSegmentValid({2.0, 2.0}, {2.0, 2.0}));
}

// ── Smoothing edge cases ────────────────────────────────────────────────

TEST(SmoothEdge, EmptyPath) {
    std::ofstream f("/tmp/open_5.txt");
    for (int y = 0; y < 5; ++y) { std::string row(5, '0'); f << row << '\n'; }
    f.close();
    GridMap m;
    m.loadFromTxt("/tmp/open_5.txt");
    GridCollisionChecker cc(m);
    ShortcutSmoother sm(cc);
    Path2d empty;
    auto result = sm.smooth(empty);
    EXPECT_TRUE(result.empty());
}

TEST(SmoothEdge, TwoPointPath) {
    std::ofstream f("/tmp/open_5.txt");
    for (int y = 0; y < 5; ++y) { std::string row(5, '0'); f << row << '\n'; }
    f.close();
    GridMap m;
    m.loadFromTxt("/tmp/open_5.txt");
    GridCollisionChecker cc(m);
    ShortcutSmoother sm(cc);
    Path2d two = {{0.0, 0.0}, {4.0, 4.0}};
    auto result = sm.smooth(two);
    EXPECT_GE(result.size(), 2u);  // at least start and end
}

// ── PlannerResult edge cases ────────────────────────────────────────────

TEST(ResultEdge, DefaultResultIsNotSuccessful) {
    PlannerResult r;
    EXPECT_FALSE(r.success);
    EXPECT_TRUE(r.path.empty());
    EXPECT_DOUBLE_EQ(r.path_length, 0.0);
}

// ── Improved A* edge cases ──────────────────────────────────────────────

TEST(ImprovedAStarEdge, NullCostmapIsSafe) {
    std::ofstream f("/tmp/open_5.txt");
    for (int y = 0; y < 5; ++y) { std::string row(5, '0'); f << row << '\n'; }
    f.close();
    GridMap m;
    m.loadFromTxt("/tmp/open_5.txt");

    ImprovedAStarPlanner p(1.0, 2.0, 0.5, true);  // no costmap set
    auto r = p.plan(m, {0, 0}, {4, 4});
    EXPECT_TRUE(r.success);  // should still work without costmap
}
