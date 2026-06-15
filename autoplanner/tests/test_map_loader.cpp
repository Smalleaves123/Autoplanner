#include <gtest/gtest.h>

#include "autoplanner/core/grid_map.h"

using namespace autoplanner;

TEST(MapLoader, LoadSimpleMap) {
    GridMap map;
    EXPECT_TRUE(map.loadFromTxt("data/maps/simple_50x50.txt"));
    EXPECT_EQ(map.width(), 50);
    EXPECT_EQ(map.height(), 50);
}

TEST(MapLoader, LoadMaze) {
    GridMap map;
    EXPECT_TRUE(map.loadFromTxt("data/maps/maze_100x100.txt"));
    EXPECT_EQ(map.width(), 100);
    EXPECT_EQ(map.height(), 100);
}

TEST(MapLoader, LoadWarehouse) {
    GridMap map;
    EXPECT_TRUE(map.loadFromTxt("data/maps/warehouse_100x100.txt"));
    EXPECT_EQ(map.width(), 100);
    EXPECT_EQ(map.height(), 100);
}

TEST(MapLoader, LoadRandom) {
    GridMap map;
    EXPECT_TRUE(map.loadFromTxt("data/maps/random_100x100_density_20.txt"));
    EXPECT_EQ(map.width(), 100);
    EXPECT_EQ(map.height(), 100);
}

TEST(MapLoader, FileNotFound) {
    GridMap map;
    EXPECT_FALSE(map.loadFromTxt("nonexistent.txt"));
}

TEST(MapLoader, FreeCellCountReasonable) {
    GridMap map;
    map.loadFromTxt("data/maps/simple_50x50.txt");

    int free_count = 0;
    for (int y = 0; y < map.height(); ++y) {
        for (int x = 0; x < map.width(); ++x) {
            if (map.isFree(x, y)) free_count++;
        }
    }

    // A simple 50x50 map should have mostly free cells.
    EXPECT_GT(free_count, 1000);
}
