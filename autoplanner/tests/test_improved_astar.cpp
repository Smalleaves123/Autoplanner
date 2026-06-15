#include <gtest/gtest.h>

#include "autoplanner/core/grid_map.h"
#include "autoplanner/costmap/costmap_2d.h"
#include "autoplanner/costmap/obstacle_inflation.h"
#include "autoplanner/planners/graph_search/improved_astar.h"

using namespace autoplanner;

namespace {

std::unique_ptr<GridMap> loadSimpleMap() {
    auto map = std::make_unique<GridMap>();
    map->loadFromTxt("data/maps/simple_50x50.txt");
    return map;
}

}  // namespace

TEST(ImprovedAStar, FindsPathWithoutCostmap) {
    auto map = loadSimpleMap();
    ImprovedAStarPlanner planner(1.0, 0.0, 0.0, true);

    PlannerResult result = planner.plan(*map, Point2i{1, 1}, Point2i{48, 48});

    EXPECT_TRUE(result.success);
    EXPECT_GT(result.path.size(), 0u);
    EXPECT_GT(result.path_length, 0.0);
}

TEST(ImprovedAStar, FindsPathWithCostmap) {
    auto map = loadSimpleMap();
    Costmap2D costmap;
    costmap.buildFromGridMap(*map);
    costmap.inflateObstacles(1.0);

    ImprovedAStarPlanner planner(1.0, 2.0, 0.5, true);
    planner.setCostmap(&costmap);

    PlannerResult result = planner.plan(*map, Point2i{1, 1}, Point2i{48, 48});

    EXPECT_TRUE(result.success);
    EXPECT_GT(result.path.size(), 0u);
}

TEST(ImprovedAStar, PathIsCollisionFree) {
    auto map = loadSimpleMap();
    Costmap2D costmap;
    costmap.buildFromGridMap(*map);
    costmap.inflateObstacles(1.0);

    ImprovedAStarPlanner planner(1.0, 2.0, 0.5, true);
    planner.setCostmap(&costmap);

    PlannerResult result = planner.plan(*map, Point2i{1, 1}, Point2i{48, 48});

    ASSERT_TRUE(result.success);
    for (const auto& p : result.path) {
        int x = static_cast<int>(p.x);
        int y = static_cast<int>(p.y);
        EXPECT_TRUE(map->isFree(x, y))
            << "Path point (" << x << ", " << y << ") is occupied";
    }
}

TEST(ImprovedAStar, InvalidStartFails) {
    auto map = loadSimpleMap();
    ImprovedAStarPlanner planner;

    PlannerResult result = planner.plan(*map, Point2i{0, 0}, Point2i{48, 48});
    EXPECT_FALSE(result.success);
}

TEST(ImprovedAStar, FourConnectedWorks) {
    auto map = loadSimpleMap();
    ImprovedAStarPlanner planner(1.0, 0.0, 0.0, false);

    PlannerResult result = planner.plan(*map, Point2i{1, 1}, Point2i{48, 48});

    EXPECT_TRUE(result.success);
    EXPECT_GT(result.path.size(), 0u);
}
