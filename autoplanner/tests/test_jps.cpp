#include <gtest/gtest.h>

#include "autoplanner/core/grid_map.h"
#include "autoplanner/planners/graph_search/jps.h"
#include "autoplanner/planners/graph_search/astar.h"

using namespace autoplanner;

namespace {

std::unique_ptr<GridMap> loadMap(const std::string& path) {
    auto map = std::make_unique<GridMap>();
    map->loadFromTxt(path);
    return map;
}

}  // namespace

TEST(JPS, FindsPathOnSimpleMap) {
    auto map = loadMap("data/maps/simple_50x50.txt");
    JPSPlanner planner(true);

    PlannerResult result = planner.plan(*map, Point2i{1, 1}, Point2i{48, 48});

    EXPECT_TRUE(result.success);
    EXPECT_GT(result.path.size(), 0u);
    EXPECT_GT(result.path_length, 0.0);
}

TEST(JPS, StartIsGoal) {
    auto map = loadMap("data/maps/simple_50x50.txt");
    JPSPlanner planner(true);

    PlannerResult result = planner.plan(*map, Point2i{5, 5}, Point2i{5, 5});

    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.path.size(), 1u);
}

TEST(JPS, InvalidStartFails) {
    auto map = loadMap("data/maps/simple_50x50.txt");
    JPSPlanner planner(true);

    PlannerResult result = planner.plan(*map, Point2i{0, 0}, Point2i{48, 48});
    EXPECT_FALSE(result.success);
}

TEST(JPS, InvalidGoalFails) {
    auto map = loadMap("data/maps/simple_50x50.txt");
    JPSPlanner planner(true);

    PlannerResult result = planner.plan(*map, Point2i{1, 1}, Point2i{0, 0});
    EXPECT_FALSE(result.success);
}

TEST(JPS, PathIsCollisionFree) {
    auto map = loadMap("data/maps/simple_50x50.txt");
    JPSPlanner planner(true);

    PlannerResult result = planner.plan(*map, Point2i{1, 1}, Point2i{48, 48});

    ASSERT_TRUE(result.success);
    for (const auto& p : result.path) {
        int x = static_cast<int>(p.x);
        int y = static_cast<int>(p.y);
        EXPECT_TRUE(map->isFree(x, y))
            << "Path point (" << x << ", " << y << ") is occupied";
    }
}

TEST(JPS, ExpandsFewerNodesThanAStar) {
    auto map = loadMap("data/maps/simple_50x50.txt");
    JPSPlanner jps(true);
    AStarPlanner astar(true);

    PlannerResult r1 = jps.plan(*map, Point2i{1, 1}, Point2i{48, 48});
    PlannerResult r2 = astar.plan(*map, Point2i{1, 1}, Point2i{48, 48});

    ASSERT_TRUE(r1.success);
    ASSERT_TRUE(r2.success);
    // JPS should expand fewer nodes on an open grid.
    EXPECT_LE(r1.expanded_nodes, r2.expanded_nodes);
}

TEST(JPS, FourConnectedWorks) {
    auto map = loadMap("data/maps/simple_50x50.txt");
    JPSPlanner planner(false);

    // Point to point in open area on same row — reliably reachable with 4-connectivity.
    PlannerResult result = planner.plan(*map, Point2i{16, 5}, Point2i{29, 5});

    EXPECT_TRUE(result.success);
    EXPECT_GT(result.path.size(), 0u);
}

TEST(JPS, FindsPathOnWarehouse) {
    auto map = loadMap("data/maps/warehouse_100x100.txt");
    JPSPlanner planner(true);

    PlannerResult result = planner.plan(*map, Point2i{1, 1}, Point2i{98, 98});

    EXPECT_TRUE(result.success);
    EXPECT_GT(result.path.size(), 0u);
}
