#include <gtest/gtest.h>

#include "autoplanner/core/grid_map.h"

using namespace autoplanner;

TEST(GridMap, LoadValidTxt) {
    GridMap map;
    EXPECT_TRUE(map.loadFromTxt("data/maps/simple_50x50.txt"));
    EXPECT_EQ(map.width(), 50);
    EXPECT_EQ(map.height(), 50);
}

TEST(GridMap, BoundsCheck) {
    GridMap map;
    map.loadFromTxt("data/maps/simple_50x50.txt");

    EXPECT_TRUE(map.isInside(0, 0));
    EXPECT_TRUE(map.isInside(49, 49));
    EXPECT_FALSE(map.isInside(-1, 0));
    EXPECT_FALSE(map.isInside(0, 50));
    EXPECT_FALSE(map.isInside(50, 0));
}

TEST(GridMap, FreeAndOccupied) {
    GridMap map;
    map.loadFromTxt("data/maps/simple_50x50.txt");

    // The outer border is all obstacles (1s) in simple_50x50.txt
    EXPECT_TRUE(map.isOccupied(0, 0));
    EXPECT_FALSE(map.isFree(0, 0));

    // Interior should be free.
    EXPECT_TRUE(map.isFree(1, 1));
    EXPECT_FALSE(map.isOccupied(1, 1));
}

TEST(GridMap, OutOfBoundsIsBlocked) {
    GridMap map;
    map.loadFromTxt("data/maps/simple_50x50.txt");

    EXPECT_FALSE(map.isFree(-1, 0));
    EXPECT_TRUE(map.isOccupied(-1, 0));
}

TEST(GridMap, Resolution) {
    GridMap map;
    EXPECT_DOUBLE_EQ(map.resolution(), 1.0);
    map.setResolution(0.05);
    EXPECT_DOUBLE_EQ(map.resolution(), 0.05);
}

TEST(GridMap, IndexFlattening) {
    GridMap map;
    map.loadFromTxt("data/maps/simple_50x50.txt");

    EXPECT_EQ(map.index(0, 0), 0);
    EXPECT_EQ(map.index(49, 0), 49);
    EXPECT_EQ(map.index(0, 1), 50);
    EXPECT_EQ(map.index(5, 3), 3 * 50 + 5);
}

TEST(GridMap, LoadNonexistentFile) {
    GridMap map;
    EXPECT_FALSE(map.loadFromTxt("nonexistent_file.txt"));
}
