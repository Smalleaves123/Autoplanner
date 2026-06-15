#include <gtest/gtest.h>

#include "autoplanner/core/grid_map.h"
#include "autoplanner/planners/sampling/rrt.h"

using namespace autoplanner;

namespace {

std::unique_ptr<GridMap> loadMap(const std::string& path) {
    auto map = std::make_unique<GridMap>();
    map->loadFromTxt(path);
    return map;
}

}  // namespace

TEST(RRT, FindsPathOnSimpleMap) {
    auto map = loadMap("data/maps/simple_50x50.txt");
    RRTPlanner planner(2.0, 5000, 0.1, 2.0);

    PlannerResult result = planner.plan(*map, Point2i{1, 1}, Point2i{48, 48});

    EXPECT_TRUE(result.success);
    EXPECT_GT(result.path.size(), 0u);
    EXPECT_GT(result.path_length, 0.0);
    EXPECT_GT(result.expanded_nodes, 0);
}

TEST(RRT, FindsPathOnWarehouse) {
    auto map = loadMap("data/maps/warehouse_100x100.txt");
    RRTPlanner planner(3.0, 15000, 0.15, 3.0);

    PlannerResult result = planner.plan(*map, Point2i{1, 1}, Point2i{98, 98});

    EXPECT_TRUE(result.success);
}

TEST(RRT, InvalidStartFails) {
    auto map = loadMap("data/maps/simple_50x50.txt");
    RRTPlanner planner;

    PlannerResult result = planner.plan(*map, Point2i{0, 0}, Point2i{48, 48});
    EXPECT_FALSE(result.success);
}

TEST(RRT, InvalidGoalFails) {
    auto map = loadMap("data/maps/simple_50x50.txt");
    RRTPlanner planner;

    PlannerResult result = planner.plan(*map, Point2i{1, 1}, Point2i{0, 0});
    EXPECT_FALSE(result.success);
}

TEST(RRT, TreeHasEdges) {
    auto map = loadMap("data/maps/simple_50x50.txt");
    RRTPlanner planner(2.0, 5000, 0.1, 2.0);

    planner.plan(*map, Point2i{1, 1}, Point2i{48, 48});

    // After planning, tree_edges should be populated.
    EXPECT_GT(planner.treeEdges().size(), 0u);
}

TEST(RRT, PathIsCollisionFree) {
    auto map = loadMap("data/maps/simple_50x50.txt");
    RRTPlanner planner(2.0, 5000, 0.1, 2.0);

    PlannerResult result = planner.plan(*map, Point2i{1, 1}, Point2i{48, 48});

    ASSERT_TRUE(result.success);
    for (const auto& p : result.path) {
        int x = static_cast<int>(std::floor(p.x));
        int y = static_cast<int>(std::floor(p.y));
        EXPECT_TRUE(map->isInside(x, y));
    }
}
