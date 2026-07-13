#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <string>

#include "autoplanner/collision/grid_collision_checker.h"
#include "autoplanner/collision/line_collision_checker.h"
#include "autoplanner/collision/footprint_collision_checker.h"
#include "autoplanner/core/grid_map.h"

using namespace autoplanner;

class CollisionCheckerTest : public ::testing::Test {
protected:
    void SetUp() override {
        map_.loadFromTxt("data/maps/simple_50x50.txt");
    }

    GridMap map_;
};

TEST_F(CollisionCheckerTest, GridCheckerStateValid) {
    GridCollisionChecker checker(map_);

    // Interior is free.
    EXPECT_TRUE(checker.isStateValid(Point2d{5.0, 5.0}));
    EXPECT_TRUE(checker.isStateValid(Point2d{25.5, 30.3}));

    // Border is occupied.
    EXPECT_FALSE(checker.isStateValid(Point2d{0.0, 0.0}));

    // Out of bounds.
    EXPECT_FALSE(checker.isStateValid(Point2d{-1.0, 5.0}));
}

TEST_F(CollisionCheckerTest, GridCheckerSegmentValid) {
    GridCollisionChecker checker(map_);

    // Segment entirely in free space.
    EXPECT_TRUE(checker.isSegmentValid(Point2d{5.0, 5.0}, Point2d{10.0, 10.0}));

    // Segment crossing obstacle.
    EXPECT_FALSE(checker.isSegmentValid(Point2d{0.0, 0.0}, Point2d{10.0, 10.0}));
}

TEST_F(CollisionCheckerTest, GridCheckerPathValid) {
    GridCollisionChecker checker(map_);

    std::vector<Point2d> valid_path = {
        {5.0, 5.0}, {8.0, 8.0}, {12.0, 12.0}
    };
    EXPECT_TRUE(checker.isPathValid(valid_path));

    std::vector<Point2d> invalid_path = {
        {5.0, 5.0}, {0.0, 0.0}, {10.0, 10.0}
    };
    EXPECT_FALSE(checker.isPathValid(invalid_path));

    // Empty path is valid.
    EXPECT_TRUE(checker.isPathValid({}));
}

TEST_F(CollisionCheckerTest, LineCheckerStateValid) {
    LineCollisionChecker checker(map_);

    EXPECT_TRUE(checker.isStateValid(Point2d{5.0, 5.0}));
    EXPECT_FALSE(checker.isStateValid(Point2d{0.0, 0.0}));
}

TEST_F(CollisionCheckerTest, LineCheckerSegmentValid) {
    LineCollisionChecker checker(map_);

    EXPECT_TRUE(checker.isSegmentValid(Point2d{5.0, 5.0}, Point2d{10.0, 10.0}));
    EXPECT_FALSE(checker.isSegmentValid(Point2d{0.0, 0.0}, Point2d{10.0, 10.0}));
}

TEST_F(CollisionCheckerTest, LineCheckerCustomResolution) {
    LineCollisionChecker checker(map_);
    checker.setCheckResolution(0.1);

    // Finer resolution should still work.
    EXPECT_TRUE(checker.isSegmentValid(Point2d{5.0, 5.0}, Point2d{10.0, 10.0}));
}

TEST(FootprintCollisionCheckerTest, CircleRejectsNearbyObstacle) {
    std::ofstream file("/tmp/footprint_circle_map.txt");
    for (int y = 0; y < 10; ++y) {
        std::string row(10, '0');
        if (y == 5) row[6] = '1';
        file << row << '\n';
    }
    file.close();

    GridMap map;
    ASSERT_TRUE(map.loadFromTxt("/tmp/footprint_circle_map.txt"));
    FootprintCollisionChecker checker(map, RobotFootprint::circle(1.0));

    EXPECT_TRUE(checker.isPoseValid({2.0, 5.0, 0.0}));
    EXPECT_FALSE(checker.isPoseValid({5.5, 5.0, 0.0}));
}

TEST(FootprintCollisionCheckerTest, RectangleUsesHeading) {
    std::ofstream file("/tmp/footprint_rectangle_map.txt");
    for (int y = 0; y < 12; ++y) {
        std::string row(12, '0');
        if (y == 4) row[5] = '1';
        file << row << '\n';
    }
    file.close();

    GridMap map;
    ASSERT_TRUE(map.loadFromTxt("/tmp/footprint_rectangle_map.txt"));
    FootprintCollisionChecker checker(
        map, RobotFootprint::rectangle(4.0, 1.0));

    EXPECT_TRUE(checker.isPoseValid({2.0, 8.0, 0.0}));
    EXPECT_FALSE(checker.isPoseValid({5.0, 5.0, M_PI_2}));
}
