#include <gtest/gtest.h>

#include "autoplanner/collision/grid_collision_checker.h"
#include "autoplanner/core/grid_map.h"
#include "autoplanner/smoothing/shortcut_smoother.h"
#include "autoplanner/smoothing/bezier_smoother.h"
#include "autoplanner/smoothing/bspline_smoother.h"

using namespace autoplanner;

class SmoothingTest : public ::testing::Test {
protected:
    void SetUp() override {
        map_.loadFromTxt("data/maps/simple_50x50.txt");
    }

    GridMap map_;
};

TEST_F(SmoothingTest, ShortcutReducesPath) {
    GridCollisionChecker checker(map_);
    ShortcutSmoother smoother(checker, 50);

    std::vector<Point2d> path = {
        {1.0, 1.0}, {2.0, 2.0}, {3.0, 3.0}, {4.0, 4.0}, {5.0, 5.0}
    };

    auto smoothed = smoother.smooth(path);

    // Shortcut should at least not increase path size.
    EXPECT_LE(smoothed.size(), path.size());
    // Start and end should be preserved.
    EXPECT_DOUBLE_EQ(smoothed.front().x, path.front().x);
    EXPECT_DOUBLE_EQ(smoothed.front().y, path.front().y);
    EXPECT_DOUBLE_EQ(smoothed.back().x, path.back().x);
    EXPECT_DOUBLE_EQ(smoothed.back().y, path.back().y);
}

TEST_F(SmoothingTest, ShortcutPreservesCollisionFree) {
    GridCollisionChecker checker(map_);
    ShortcutSmoother smoother(checker, 100);

    // Path in the middle corridor (x in [16,29]) — free in simple_50x50 (avoid row 35).
    std::vector<Point2d> path;
    for (int y = 1; y < 34; ++y) {
        path.emplace_back(20.0, static_cast<double>(y));
    }

    auto smoothed = smoother.smooth(path);

    for (const auto& p : smoothed) {
        int x = static_cast<int>(p.x);
        int y = static_cast<int>(p.y);
        EXPECT_TRUE(map_.isFree(x, y))
            << "Smoothed point (" << x << ", " << y << ") is occupied";
    }
}

TEST_F(SmoothingTest, BezierSmootherWorks) {
    BezierSmoother smoother(10);

    std::vector<Point2d> path = {
        {1.0, 1.0}, {4.0, 4.0}, {8.0, 8.0}, {12.0, 12.0}
    };

    auto smoothed = smoother.smooth(path);

    // Bezier smoothing increases point density.
    EXPECT_GT(smoothed.size(), path.size());
    // Should have similar endpoints.
    EXPECT_NEAR(smoothed.front().x, 1.0, 1.0);
    EXPECT_NEAR(smoothed.back().x, 12.0, 1.0);
}

TEST_F(SmoothingTest, BSplineSmootherWorks) {
    BSplineSmoother smoother(10);

    std::vector<Point2d> path = {
        {1.0, 1.0}, {4.0, 4.0}, {8.0, 8.0}, {12.0, 12.0}
    };

    auto smoothed = smoother.smooth(path);

    EXPECT_GT(smoothed.size(), 0u);
}

TEST_F(SmoothingTest, SmoothShortPathUnchanged) {
    GridCollisionChecker checker(map_);
    ShortcutSmoother smoother(checker, 100);

    // Path with only 2 points cannot be shortened further.
    std::vector<Point2d> path = {{1.0, 1.0}, {48.0, 1.0}};
    auto smoothed = smoother.smooth(path);

    EXPECT_EQ(smoothed.size(), 2u);
}

TEST_F(SmoothingTest, BezierShortPathUnchanged) {
    BezierSmoother smoother(10);

    std::vector<Point2d> path = {{1.0, 1.0}, {48.0, 1.0}};
    auto smoothed = smoother.smooth(path);

    // Short paths should be returned as-is.
    EXPECT_EQ(smoothed.size(), 2u);
}
