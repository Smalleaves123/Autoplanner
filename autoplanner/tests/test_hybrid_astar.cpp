#include <gtest/gtest.h>

#include "autoplanner/core/grid_map.h"
#include "autoplanner/planners/kinodynamic/hybrid_astar.h"

using namespace autoplanner;

namespace {

std::unique_ptr<GridMap> loadMap(const std::string& path) {
    auto map = std::make_unique<GridMap>();
    map->loadFromTxt(path);
    return map;
}

}  // namespace

TEST(HybridAStar, FindsPathOnSimpleMap) {
    auto map = loadMap("data/maps/simple_50x50.txt");
    HybridAStarPlanner planner(5.0, 2.0, 36);

    PlannerResult result = planner.plan(*map, Point2i{5, 5}, Point2i{45, 45});

    EXPECT_TRUE(result.success);
    EXPECT_GT(result.path.size(), 0u);
    EXPECT_GT(result.path_length, 0.0);
}

TEST(HybridAStar, InvalidStartFails) {
    auto map = loadMap("data/maps/simple_50x50.txt");
    HybridAStarPlanner planner;

    PlannerResult result = planner.plan(*map, Point2i{0, 0}, Point2i{45, 45});
    EXPECT_FALSE(result.success);
}

TEST(HybridAStar, InvalidGoalFails) {
    auto map = loadMap("data/maps/simple_50x50.txt");
    HybridAStarPlanner planner;

    PlannerResult result = planner.plan(*map, Point2i{5, 5}, Point2i{0, 0});
    EXPECT_FALSE(result.success);
}

TEST(HybridAStar, PathIsCollisionFree) {
    auto map = loadMap("data/maps/simple_50x50.txt");
    HybridAStarPlanner planner(5.0, 2.0, 36);

    PlannerResult result = planner.plan(*map, Point2i{5, 5}, Point2i{45, 45});

    ASSERT_TRUE(result.success);
    for (const auto& p : result.path) {
        int x = static_cast<int>(std::floor(p.x));
        int y = static_cast<int>(std::floor(p.y));
        EXPECT_TRUE(map->isInside(x, y));
    }
}

TEST(HybridAStar, FindsPathOnOpenArea) {
    auto map = loadMap("data/maps/simple_50x50.txt");
    HybridAStarPlanner planner(3.0, 2.0, 36);

    // Open-area path from one corner to another.
    PlannerResult result = planner.plan(*map, Point2i{2, 2}, Point2i{47, 47});

    EXPECT_TRUE(result.success);
    EXPECT_GT(result.path_length, 0.0);
}
