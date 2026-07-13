#include <gtest/gtest.h>

#include "autoplanner/core/grid_map.h"
#include "autoplanner/planners/sampling/informed_rrt_star.h"

using namespace autoplanner;

namespace {

std::unique_ptr<GridMap> loadMap(const std::string& path) {
    auto map = std::make_unique<GridMap>();
    map->loadFromTxt(path);
    return map;
}

}  // namespace

TEST(InformedRRTStar, FindsPathOnSimpleMap) {
    auto map = loadMap("data/maps/simple_50x50.txt");
    InformedRRTStarPlanner planner(2.0, 5000, 0.1, 2.0, 5.0);

    PlannerResult result = planner.plan(*map, Point2i{1, 1}, Point2i{48, 48});

    EXPECT_TRUE(result.success);
    EXPECT_GT(result.path.size(), 0u);
    EXPECT_GT(result.path_length, 0.0);
}

TEST(InformedRRTStar, FindsPathOnWarehouse) {
    auto map = loadMap("data/maps/warehouse_100x100.txt");
    InformedRRTStarPlanner planner(3.0, 15000, 0.15, 3.0, 10.0);

    PlannerResult result = planner.plan(*map, Point2i{1, 1}, Point2i{98, 98});

    EXPECT_TRUE(result.success);
}

TEST(InformedRRTStar, InvalidStartFails) {
    auto map = loadMap("data/maps/simple_50x50.txt");
    InformedRRTStarPlanner planner;

    PlannerResult result = planner.plan(*map, Point2i{0, 0}, Point2i{48, 48});
    EXPECT_FALSE(result.success);
}

TEST(InformedRRTStar, InvalidGoalFails) {
    auto map = loadMap("data/maps/simple_50x50.txt");
    InformedRRTStarPlanner planner;

    PlannerResult result = planner.plan(*map, Point2i{1, 1}, Point2i{0, 0});
    EXPECT_FALSE(result.success);
}

TEST(InformedRRTStar, PathIsInFreeSpace) {
    auto map = loadMap("data/maps/simple_50x50.txt");
    InformedRRTStarPlanner planner(2.0, 5000, 0.1, 2.0, 5.0);

    PlannerResult result = planner.plan(*map, Point2i{1, 1}, Point2i{48, 48});

    ASSERT_TRUE(result.success);
    for (const auto& p : result.path) {
        int x = static_cast<int>(std::floor(p.x));
        int y = static_cast<int>(std::floor(p.y));
        EXPECT_TRUE(map->isInside(x, y));
    }
}

TEST(InformedRRTStar, TreeHasEdges) {
    auto map = loadMap("data/maps/simple_50x50.txt");
    InformedRRTStarPlanner planner(2.0, 5000, 0.1, 2.0, 5.0);

    planner.plan(*map, Point2i{1, 1}, Point2i{48, 48});

    EXPECT_GT(planner.treeEdges().size(), 0u);
}
